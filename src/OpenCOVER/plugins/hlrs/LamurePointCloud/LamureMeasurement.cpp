#include "LamureMeasurement.h"

#include <cover/VRViewer.h>
#include <osgViewer/Viewer>
#include <osg/Stats>
#include <fstream>
#include <iostream>
#include <iomanip> // For std::fixed and std::setprecision
#include <cmath>
#include <numeric>
#include <algorithm>
#include <locale>

#include "Lamure.h"

#ifdef _WIN32
#include <windows.h>
// --- NVML dynamic loader ---
struct NvmlLoader {
    HMODULE h = nullptr;
    bool ok = false;
    // function pointers:
    typedef int (*nvmlInit_t)();
    typedef int (*nvmlShutdown_t)();
    typedef int (*nvmlDeviceGetHandleByIndex_t)(unsigned, void**);
    typedef int (*nvmlDeviceGetUtilizationRates_t)(void*, void*); // struct is opaque to us here
    typedef int (*nvmlDeviceGetMemoryInfo_t)(void*, void*);

    nvmlInit_t                     nvmlInit = nullptr;
    nvmlShutdown_t                 nvmlShutdown = nullptr;
    nvmlDeviceGetHandleByIndex_t   nvmlDeviceGetHandleByIndex = nullptr;
    nvmlDeviceGetUtilizationRates_t nvmlDeviceGetUtilizationRates = nullptr;
    nvmlDeviceGetMemoryInfo_t      nvmlDeviceGetMemoryInfo = nullptr;

    void* dev = nullptr;

    bool ensureLoaded() {
        if (ok) return true;
        if (!h) {
            h = LoadLibraryA("nvml.dll");
            if (!h) return false;
        }
        nvmlInit = (nvmlInit_t)GetProcAddress(h, "nvmlInit_v2");
        if (!nvmlInit) nvmlInit = (nvmlInit_t)GetProcAddress(h, "nvmlInit");
        nvmlShutdown = (nvmlShutdown_t)GetProcAddress(h, "nvmlShutdown");
        nvmlDeviceGetHandleByIndex =
            (nvmlDeviceGetHandleByIndex_t)GetProcAddress(h, "nvmlDeviceGetHandleByIndex_v2");
        if (!nvmlDeviceGetHandleByIndex)
            nvmlDeviceGetHandleByIndex =
            (nvmlDeviceGetHandleByIndex_t)GetProcAddress(h, "nvmlDeviceGetHandleByIndex");
        nvmlDeviceGetUtilizationRates =
            (nvmlDeviceGetUtilizationRates_t)GetProcAddress(h, "nvmlDeviceGetUtilizationRates");
        nvmlDeviceGetMemoryInfo =
            (nvmlDeviceGetMemoryInfo_t)GetProcAddress(h, "nvmlDeviceGetMemoryInfo");

        if (!nvmlInit || !nvmlShutdown || !nvmlDeviceGetHandleByIndex ||
            !nvmlDeviceGetUtilizationRates || !nvmlDeviceGetMemoryInfo)
            return false;

        if (nvmlInit() != 0) return false;

        // Device 0 reicht in den meisten Single-GPU-Fällen
        if (nvmlDeviceGetHandleByIndex(0, &dev) != 0) return false;

        ok = true;
        return true;
    }

    ~NvmlLoader() {
        if (ok && nvmlShutdown) nvmlShutdown();
        if (h) FreeLibrary(h);
    }
};

// Simple structs, damit wir keine NVML-Header brauchen
struct NvmlUtilization { unsigned gpu; unsigned memory; };
struct NvmlMemory     { unsigned long long total, free, used; };

static bool nvmlPoll(NvmlLoader& nv, double& gpuUtilPct,
    double& memUsedMB, double& memTotalMB)
{
    gpuUtilPct = memUsedMB = memTotalMB = 0.0;
    if (!nv.ensureLoaded()) return false;

    NvmlUtilization u{};
    if (nv.nvmlDeviceGetUtilizationRates(nv.dev, &u) == 0) {
        gpuUtilPct = double(u.gpu);
    }
    NvmlMemory m{};
    if (nv.nvmlDeviceGetMemoryInfo(nv.dev, &m) == 0) {
        memTotalMB = m.total / (1024.0 * 1024.0);
        memUsedMB  = m.used  / (1024.0 * 1024.0);
    }
    return true;
}
#endif

namespace {
    static bool getF(osg::Stats* s, unsigned f, const std::string& key, double& out)
    {
        return s && s->getAttribute(f, key, out);
    }

    static bool getDelta(osg::Stats* s, unsigned f,
        const std::string& beginKey,
        const std::string& endKey,
        double& outSeconds)
    {
        double b = 0.0, e = 0.0;
        if (!s) return false;
        if (!s->getAttribute(f, beginKey, b)) return false;
        if (!s->getAttribute(f, endKey, e)) return false;
        outSeconds = e - b;
        return (outSeconds >= 0.0);
    }

    // Priorisiert "... time taken", fällt auf (end-begin) zurück
    static bool getTimeTakenMs(osg::Stats* s, unsigned f,
        const std::string& timeTakenKey,
        const std::string& beginKey,
        const std::string& endKey,
        double& outMs)
    {
        double v = 0.0;
        if (getF(s, f, timeTakenKey, v)) { outMs = v * 1000.0; return true; }
        if (getDelta(s, f, beginKey, endKey, v)) { outMs = v * 1000.0; return true; }
        return false;
    }

    static unsigned pickStableFrame(osgViewer::ViewerBase* viewer,
        osg::Stats* viewerStats,
        const osgViewer::ViewerBase::Cameras& cams,
        const osg::FrameStamp* fs)
    {
        unsigned f = fs ? fs->getFrameNumber() : (viewerStats ? viewerStats->getLatestFrameNumber() : 0);

        if (!cams.empty())
        {
            if (auto* r = dynamic_cast<osgViewer::Renderer*>(cams.front()->getRenderer()))
            {
                if (!(r->getGraphicsThreadDoesCull()))
                    f = (f > 0) ? (f - 1) : 0;
            }
        }
        if (viewerStats)
            f = std::min(f, viewerStats->getLatestFrameNumber());
        for (auto* cam : cams)
            if (auto* cs = cam->getStats())
                f = std::min(f, cs->getLatestFrameNumber());
        return f;
    }

}


// === Konstruktor (aktiviert nur einmal die relevanten Stats) ===
LamureMeasurement::LamureMeasurement(
    Lamure*                      plugin,
    opencover::VRViewer*         viewer,
    const std::vector<Segment>&  segments,
    const std::string&           logfile)
    : m_plugin(plugin)
    , m_viewer(viewer)
    , m_segments(segments)
    , m_logfile(logfile)
{
    m_timeline.clear();
    m_verbose = false;       // true, wenn du alle N Frames eine Konsole willst
    m_logEveryN = 60;
    m_dumpAttrs = false;
    m_gpuBackSearch = 16;    // ggf. 24–32 bei stärkerem Lag
    m_exportTimeline = true;
    m_running = true;
    m_startTick = osg::Timer::instance()->tick();
    m_lastFrameTick = m_startTick;
    initCallbacks();

    if (auto* vs = m_viewer ? m_viewer->getViewerStats() : nullptr)
    {
        vs->collectStats("frame_rate", true);
        vs->collectStats("update",     true);
        vs->collectStats("sync",       true);
        vs->collectStats("swap",       true);
        vs->collectStats("finish",     true);
        vs->collectStats("isect",      true);
        vs->collectStats("plugin",     true);
        vs->collectStats("opencover",  true);
        // Optional:
        // vs->collectStats("scene",   true);
        // vs->collectStats("preframe",true);
    }

    osgViewer::ViewerBase::Cameras cams;
    if (m_viewer) m_viewer->getCameras(cams);
    for (auto* cam : cams)
        if (auto* cs = cam->getStats()) {
            cs->collectStats("rendering", true);
            cs->collectStats("gpu",       true);
            cs->collectStats("cull",      true); // harmless falls unbekannt
            cs->collectStats("draw",      true);
            // cs->collectStats("scene",  true); // optional
        }

    // Logging-Defaults (kannst du nach Bedarf umschalten)
    m_verbose   = false; // kein Konsolen-Spam
    m_logEveryN = 30;
    m_dumpAttrs = false;

    std::cout << "[Measurement] Measurement started\n";
}


LamureMeasurement::~LamureMeasurement()
{
    std::cout << "[Measurement] Measurement finished.\n";
}


void LamureMeasurement::stop() {
    m_running = false;
}


void LamureMeasurement::initCallbacks()
{
    auto cam = m_viewer->getCamera();
    m_preCB = new MarkCallback(this, true);
    m_postCB = new MarkCallback(this, false);
    cam->setPreDrawCallback(m_preCB);
    cam->setPostDrawCallback(m_postCB);
}


bool LamureMeasurement::hasStatAttribute(osg::Stats* stats,
    const std::string& key,
    unsigned int latestFrameNumber,
    unsigned int framesToCheck)
{
    if (!stats) return false;

    unsigned int firstFrame = (latestFrameNumber >= framesToCheck) ?
        latestFrameNumber - (framesToCheck - 1) : 0;

    double tmp = 0.0;
    for (unsigned int f = firstFrame; f <= latestFrameNumber; ++f)
    {
        if (stats->getAttribute(f, key, tmp)) {
            return true; // mindestens ein gültiger Wert gefunden
        }
    }
    return false;
}


bool LamureMeasurement::getTimeTakenMsBacksearch(
    osg::Stats* s,
    unsigned baseFrame,
    unsigned backSearch,
    const std::string& timeTakenKey,
    const std::string& beginKey,
    const std::string& endKey,
    double& outMs,
    unsigned& usedOffset,
    double* outBeginMs,
    double* outEndMs)
{
    if (!s) return false;

    const unsigned earliest = s->getEarliestFrameNumber();
    const unsigned latest   = s->getLatestFrameNumber();
    if (baseFrame > latest) baseFrame = latest;

    for (unsigned off = 0; off <= backSearch; ++off)
    {
        if (baseFrame < off) break;
        const unsigned f = baseFrame - off;
        if (f < earliest) break;

        double v = 0.0;
        // 1) direkter "time taken"
        if (s->getAttribute(f, timeTakenKey, v))
        {
            outMs = v * 1000.0;
            usedOffset = off;

            if (outBeginMs || outEndMs)
            {
                double b = 0.0, e = 0.0;
                if (outBeginMs && s->getAttribute(f, beginKey, b)) *outBeginMs = b * 1000.0;
                if (outEndMs   && s->getAttribute(f, endKey,   e)) *outEndMs   = e * 1000.0;
            }
            return true;
        }

        // 2) Fallback: (end - begin)
        double b = 0.0, e = 0.0;
        if (s->getAttribute(f, beginKey, b) && s->getAttribute(f, endKey, e) && e >= b)
        {
            outMs = (e - b) * 1000.0;
            usedOffset = off;

            if (outBeginMs) *outBeginMs = b * 1000.0;
            if (outEndMs)   *outEndMs   = e * 1000.0;
            return true;
        }
    }
    return false;
}


bool LamureMeasurement::tryAddBlock(
    osg::Stats* stats,
    unsigned int baseFrame,
    unsigned int backsearch,
    const std::string& statPrefix,
    const std::string& nameForCSV,
    const std::string& scope,
    int camIndex,
    std::vector<TimeBlock>& localBlocks)
{
    if (!stats) return false;

    double taken_ms = 0.0, begin_ms = 0.0, end_ms = 0.0;
    unsigned usedOffset = 0;

    if (!getTimeTakenMsBacksearch(stats,
        baseFrame,
        backsearch,
        statPrefix + " time taken",
        statPrefix + " begin time",
        statPrefix + " end time",
        taken_ms,
        usedOffset,
        &begin_ms,
        &end_ms))
    {
        return false;
    }

    TimeBlock tb;
    tb.frame       = baseFrame;                                        // Basisframe (passt zu frames.csv)
    tb.src_frame   = (usedOffset <= baseFrame) ? baseFrame - usedOffset : baseFrame; // Effektives Stats-Frame
    tb.camIndex    = camIndex;
    tb.scope       = scope;
    tb.name        = nameForCSV;
    tb.begin_ms    = begin_ms;
    tb.end_ms      = end_ms;
    tb.taken_ms    = taken_ms;
    tb.used_offset = usedOffset;

    localBlocks.push_back(std::move(tb));
    return true;
}


void LamureMeasurement::drawIncrement(bool preDraw, const osg::FrameStamp* frameStamp)
{
    if (!m_running) return;

    osg::Timer_t now = osg::Timer::instance()->tick();

    if (preDraw) {
        m_lastFrameTick = now;
        return;
    }

    if (!frameStamp) return;

    // Sammeln (gedrosseltes Debug)
    FrameStats s;
    const bool dbg = (m_verbose && (frameStamp->getFrameNumber() % m_logEveryN == 0));
    if (collectFrameStats(m_viewer, frameStamp, s, dbg))
        m_stats.push_back(s);

    // Segmente etc. (unverändert)
    osg::Timer_t now2 = osg::Timer::instance()->tick();
    double dt = osg::Timer::instance()->delta_s(m_lastFrameTick, now2);
    m_segmentTime += dt;

    while (m_currentSegment < m_segments.size())
    {
        const auto& seg = m_segments[m_currentSegment];
        float dist = seg.tra.length();
        double transDur = seg.transSpeed > 0.f ? dist / seg.transSpeed : 0.0;
        float maxAngle = std::max({ std::abs(seg.rot.x()), std::abs(seg.rot.y()), std::abs(seg.rot.z()) });
        double rotDur = seg.rotSpeed > 0.f ? maxAngle / seg.rotSpeed : 0.0;
        double segDuration = std::max(transDur, rotDur);

        if (m_segmentTime < segDuration) break;

        m_segmentTime   -= segDuration;
        m_cumulativeTra += seg.tra;
        m_cumulativeRot += seg.rot;
        ++m_currentSegment;
    }

    if (m_currentSegment >= m_segments.size()) {
        m_running = false;
        writeLogAndStop();
        return;
    }

    const auto& seg = m_segments[m_currentSegment];
    float dist = seg.tra.length();
    double transDur = seg.transSpeed > 0.f ? dist / seg.transSpeed : 0.0;
    float maxAngle = std::max({ std::abs(seg.rot.x()), std::abs(seg.rot.y()), std::abs(seg.rot.z()) });
    double rotDur = seg.rotSpeed > 0.f ? maxAngle / seg.rotSpeed : 0.0;
    double segDuration = std::max(transDur, rotDur);
    double frac = segDuration > 0.0 ? std::min(m_segmentTime / segDuration, 1.0) : 1.0;

    osg::Vec3 tra = m_cumulativeTra + seg.tra * static_cast<float>(frac);
    osg::Vec3 rot = m_cumulativeRot + seg.rot * static_cast<float>(frac);
    updateCamera(tra, rot);
}


bool LamureMeasurement::collectFrameStats(osgViewer::ViewerBase* viewer,
    const osg::FrameStamp* fs,
    FrameStats& stats,
    bool debugPrint)
{
    if (!viewer) return false;

    osg::Stats* viewerStats = viewer->getViewerStats();

    osgViewer::ViewerBase::Cameras cams;
    viewer->getCameras(cams);

    const unsigned f = pickStableFrame(viewer, viewerStats, cams, fs);
    stats = FrameStats{};
    stats.frame_number = f;

    // --- Viewer: FPS, Framezeit, Rendering traversals ---
    if (viewerStats)
    {
        double hz = 0.0;
        if (viewerStats->getAttribute(f, "Frame rate", hz)) stats.frame_rate = hz;

        double ft = 0.0;
        if (viewerStats->getAttribute(f, "Frame duration", ft)) {
            stats.frame_duration_ms = ft * 1000.0;
        } else {
            double rt = 0.0; unsigned dummyOff = 0;
            if (getTimeTakenMsBacksearch(viewerStats, f, m_gpuBackSearch,
                "Rendering traversals time taken",
                "Rendering traversals begin time", "Rendering traversals end time",
                rt, dummyOff))
            {
                stats.frame_duration_ms = rt;
            }
        }

        double rtv=0.0; unsigned rtoff=0;
        if (getTimeTakenMsBacksearch(viewerStats, f, m_gpuBackSearch,
            "Rendering traversals time taken",
            "Rendering traversals begin time","Rendering traversals end time",
            rtv, rtoff))
        {
            stats.rendering_traversals_ms = rtv;
        }
    }

    // --- Viewer-Timings + Timeline-Blocks (in eine lokale Liste) ---
    std::vector<TimeBlock> blocks;

    auto addViewer = [&](const std::string& prefix, const char* csvName){
        double ms=0.0; unsigned off=0;
        if (viewerStats && getTimeTakenMsBacksearch(viewerStats, f, m_gpuBackSearch,
            prefix + " time taken", prefix + " begin time", prefix + " end time", ms, off))
        {
            if      (prefix == "Update traversal") stats.cpu_update_ms = ms;
            else if (prefix == "sync")             stats.sync_time_ms  = ms;
            else if (prefix == "swap")             stats.swap_time_ms  = ms;
            else if (prefix == "finish")           stats.finish_ms     = ms;
            else if (prefix == "Plugin")           stats.plugin_ms     = ms;
            else if (prefix == "Isect")            stats.isect_ms      = ms;
            else if (prefix == "opencover")        stats.opencover_ms  = ms;
        }
        if (viewerStats)
            (void)tryAddBlock(viewerStats, f, m_gpuBackSearch, prefix, csvName, "viewer", -1, blocks);
        };

    addViewer("Update traversal", "Update traversal");
    addViewer("sync",            "sync");
    addViewer("swap",            "swap");
    addViewer("finish",          "finish");
    addViewer("Plugin",          "Plugin");
    addViewer("Isect",           "Isect");
    addViewer("opencover",       "opencover");

    // --- Kamera: Cull/Draw/GPU aggregiert + Blocks ---
    double sumCull_ms = 0.0, sumDraw_ms = 0.0, sumGpu_ms = 0.0;

    for (size_t i = 0; i < cams.size(); ++i)
    {
        osg::Camera* cam = cams[i];
        osg::Stats* cs = cam ? cam->getStats() : nullptr;
        if (!cs) continue;

        // Wenn GraphicsThread nicht cullt: Kamera-Stats typischerweise 1 Frame hinterher
        unsigned adjustedBase = stats.frame_number;
        if (auto* rnd = dynamic_cast<osgViewer::Renderer*>(cam->getRenderer()))
            if (!rnd->getGraphicsThreadDoesCull() && adjustedBase > 0)
                --adjustedBase;

        double cull_ms = 0.0, draw_ms = 0.0, gpu_ms = 0.0;
        unsigned offCull = 0, offDraw = 0;

        // CULL
        getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
            "Cull traversal time taken",
            "Cull traversal begin time", "Cull traversal end time",
            cull_ms, offCull);
        (void)tryAddBlock(cs, adjustedBase, m_gpuBackSearch,
            "Cull traversal", "Cull traversal", "camera", int(i), blocks);
        stats.backoff_cull = std::max(stats.backoff_cull, offCull);

        // DRAW
        getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
            "Draw traversal time taken",
            "Draw traversal begin time", "Draw traversal end time",
            draw_ms, offDraw);
        (void)tryAddBlock(cs, adjustedBase, m_gpuBackSearch,
            "Draw traversal", "Draw traversal", "camera", int(i), blocks);
        stats.backoff_draw = std::max(stats.backoff_draw, offDraw);

        // GPU: zuerst „GPU draw…“ versuchen, nur wenn *nicht* vorhanden, „GPU time…“
        unsigned offGpuTmp = 0;
        if (!getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
            "GPU draw time taken",
            "GPU draw begin time", "GPU draw end time",
            gpu_ms, offGpuTmp))
        {
            getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
                "GPU time taken",
                "GPU begin time", "GPU end time",
                gpu_ms, offGpuTmp);
        }
        stats.backoff_gpu = std::max(stats.backoff_gpu, offGpuTmp);

        // Blocks: exakt eine GPU-Variante eintragen
        if (!tryAddBlock(cs, adjustedBase, m_gpuBackSearch, "GPU draw", "GPU draw", "camera", int(i), blocks))
            (void)tryAddBlock(cs, adjustedBase, m_gpuBackSearch, "GPU", "GPU time", "camera", int(i), blocks);

        sumCull_ms += cull_ms;
        sumDraw_ms += draw_ms;
        sumGpu_ms  += gpu_ms;
    }

    stats.cpu_cull_ms = sumCull_ms;
    stats.cpu_draw_ms = sumDraw_ms;
    stats.gpu_time_ms = sumGpu_ms;

    // --- Viewer-GPU Telemetrie (falls verfügbar / NVML überschreibt ggf.) ---
    double v=0.0;
    if (viewerStats && viewerStats->getAttribute(f, "GPU Clock MHz", v))      stats.gpu_clock = v;
    if (viewerStats && viewerStats->getAttribute(f, "GPU Mem Clock MHz", v))  stats.gpu_mem_clock = v;
    if (viewerStats && viewerStats->getAttribute(f, "GPU Utilization", v))    stats.gpu_util = v;
    if (viewerStats && viewerStats->getAttribute(f, "GPU PCIe rx KB/s", v))   stats.gpu_pci = v;

    // Pose
    if (!cams.empty() && cams.front())
    {
        osg::Matrixd vm = cams.front()->getViewMatrix();
        stats.position    = vm.getTrans();
        stats.orientation = vm.getRotate();
    }
    else
    {
        osg::Matrixd vm = m_viewer->getCamera()->getViewMatrix();
        stats.position    = vm.getTrans();
        stats.orientation = vm.getRotate();
    }

    // Plugin-Infos
    const auto& ri = m_plugin->getRenderInfo();
    stats.rendered_splats         = ri.rendered_splats;
    stats.rendered_nodes          = ri.rendered_nodes;
    stats.rendered_bounding_boxes = ri.rendered_bounding_boxes;

    // Timeline sammeln (wir sortieren/deduplizieren erst beim Schreiben)
    if (m_exportTimeline && !blocks.empty())
        m_timeline.insert(m_timeline.end(), blocks.begin(), blocks.end());

    if (debugPrint || (m_verbose && ((stats.frame_number % m_logEveryN) == 0)))
    {
        std::cout << std::fixed << std::setprecision(3)
            << "[LamureMeasurement] f=" << stats.frame_number
            << " | frame=" << stats.frame_duration_ms
            << " | upd="   << stats.cpu_update_ms
            << " | cull="  << stats.cpu_cull_ms
            << " | draw="  << stats.cpu_draw_ms
            << " | gpu="   << stats.gpu_time_ms
            << " | swap="  << stats.swap_time_ms
            << " | sync="  << stats.sync_time_ms
            << " | plug="  << stats.plugin_ms
            << " | cover=" << stats.opencover_ms
            << "\n";
    }

#ifdef _WIN32
    // NVML-Poll ggf. hier (dein vorhandener Code)
    static NvmlLoader g_nvml;
    static unsigned nvmlCounter = 0;
    if ((nvmlCounter++ % 15) == 0) {
        double util=0, used=0, total=0;
        if (nvmlPoll(g_nvml, util, used, total)) {
            if (util > 0.0)  stats.gpu_util = util;
            if (total > 0.0) { stats.gpu_mem_total_mb = total; stats.gpu_mem_used_mb = used; }
        }
    }
#endif

    return true;
}


void LamureMeasurement::updateCamera(const osg::Vec3& tra, const osg::Vec3& rot)
{
    double rx = osg::DegreesToRadians(rot.x());
    double ry = osg::DegreesToRadians(rot.y());
    double rz = osg::DegreesToRadians(rot.z());

    osg::Matrix mrx, mry, mrz;
    mrx.makeRotate(rx, 1, 0, 0);
    mry.makeRotate(ry, 0, 1, 0);
    mrz.makeRotate(rz, 0, 0, 1);
    osg::Matrix rotMat = mrx * mry * mrz;

    osg::Matrix dcs;
    dcs.postMult(rotMat);

    osg::Vec3 vp = opencover::cover->getViewerMat().getTrans();

    dcs.postMult(osg::Matrix::translate(-vp));
    dcs.postMult(osg::Matrix::translate(tra));
    dcs.postMult(osg::Matrix::translate(vp));

    opencover::cover->setXformMat(dcs);
}


void LamureMeasurement::printDebugStats(unsigned int num)
{
    if (num == 0) return;
    if (m_stats.empty()) {
        std::cout << "[Measurement] No stats collected to print." << std::endl;
        return;
    }

    auto print_frame = [](const FrameStats& s, int frame_num) {
        std::cout << std::fixed << std::setprecision(4)
            << "Frame #"          << std::setw(4) << s.frame_number << "\n"
            << "FrameRate: "      << std::setw(8) << s.frame_rate << "\n"
            << "FrameDuration: "  << std::setw(8) << s.frame_duration_ms << "\n"
            << "Splats: "         << std::setw(8) << s.rendered_splats << "\n"
            << "Nodes: "          << std::setw(8) << s.rendered_nodes << "\n"
            << "Boxes: "          << std::setw(8) << s.rendered_bounding_boxes << "\n"
            << "CPU Cull: "       << std::setw(8) << s.cpu_cull_ms << "\n"
            << "CPU Draw: "       << std::setw(8) << s.cpu_draw_ms << "\n"
            << "CPU Update: "         << std::setw(8) << s.cpu_update_ms << "\n"
            << "GPU Time: "        << std::setw(8) << s.gpu_time_ms << "\n"
            << "GPU Clock: "         << std::setw(8) << s.gpu_clock << "\n"
            << "GPU Mem Clock: "      << std::setw(8) << s.gpu_mem_clock << "\n"
            << "GPU Util: "        << std::setw(8) << s.gpu_util << "\n"
            << "GPU PCI: "         << std::setw(8) << s.gpu_pci << "\n"
            << "Sync: "           << std::setw(8) << s.sync_time_ms << "\n"
            << "Swap: "           << std::setw(8) << s.swap_time_ms << "\n"
            << "Isect: "          << std::setw(8) << s.isect_ms << "\n"
            << "Plugin: "         << std::setw(8) << s.plugin_ms << "\n"
            << "OpenCov: "        << std::setw(8) << s.opencover_ms << "\n"
            << "Pos: ("           << std::setw(6) << s.position.x() 
            << ", "               << std::setw(6) << s.position.y() 
            << ", "               << std::setw(6) << s.position.z() << ")\n"
            << "Quat: ("          << std::setw(6) << s.orientation.x() 
            << ", "               << std::setw(6) << s.orientation.y() 
            << ", "               << std::setw(6) << s.orientation.z() 
            << ", "               << std::setw(6) << s.orientation.w() << ")\n"
            << std::endl;
        };
    std::cout << "--- Measurement Debug Stats ---" << std::endl;
    size_t count = m_stats.size();

    if (count <= num*2) {
        for (size_t i = 0; i < count; ++i) {
            print_frame(m_stats[i], static_cast<int>(i + 1));
        }
    } else {
        // First 3 frames
        for (size_t i = 0; i < num; ++i) {
            print_frame(m_stats[i], static_cast<int>(i + 1));
        }
        std::cout << "..." << std::endl;
        // Last 3 frames
        for (size_t i = count - num; i < count; ++i) {
            print_frame(m_stats[i], static_cast<int>(i + 1));
        }
    }

    std::cout << "-------------------------------" << std::endl;
}


void LamureMeasurement::writeLogAndStop()
{
    if (m_stats.empty())
        return;

    printDebugStats(0);

    std::string base_path = m_logfile;
    std::string::size_type pos = base_path.find_last_of(".");
    if (pos != std::string::npos) base_path = base_path.substr(0, pos);

    const std::string frames_path   = base_path + "_frames.csv";
    const std::string summary_path  = base_path + "_summary.csv";
    const std::string timeline_path = base_path + "_timeline.csv"; // NEU

    // ---------- Frames CSV ----------
    {
        std::ofstream frames_out(frames_path);
        if (!frames_out) {
            std::cerr << "[Measurement] Fehler beim Öffnen von " << frames_path << "\n";
        } else {
            frames_out.imbue(std::locale::classic());
            frames_out << std::fixed << std::setprecision(4);

            // ACHTUNG: neue Spalten-Reihenfolge (cpu_known_sum_ms und residual_time_ms direkt nach frame_time_ms)
            frames_out
                << "frame_number;"
                << "frame_rate;"                  // Hz
                << "frame_time_ms;"               // Frame duration
                << "cpu_known_sum_ms;"            // NEU: Summe der bekannten CPU-Zeiten
                << "residual_time_ms;"            // NEU: frame_time_ms - cpu_known_sum_ms (≥0)
                << "rendering_traversals_ms;"
                << "cpu_update_ms;"
                << "cpu_cull_ms;"
                << "cpu_draw_ms;"
                << "gpu_time_ms;"
                << "gpu_clock;"
                << "gpu_mem_clock;"
                << "gpu_util;"
                << "gpu_pci;"
                << "sync_time_ms;"
                << "swap_time_ms;"
                << "finish_ms;"
                << "isect_ms;"
                << "plugin_ms;"
                << "opencover_ms;"
                << "rendered_splats;"
                << "rendered_nodes;"
                << "rendered_bounding_boxes;"
                << "pos_x;pos_y;pos_z;"
                << "quat_x;quat_y;quat_z;quat_w;"
                << "backoff_cull;backoff_draw;backoff_gpu"
                << "\n";

            for (const auto& s : m_stats) {
                // “bekannte” CPU-Zeiten aufsummieren
                double cpu_known_sum_ms =
                    s.cpu_update_ms +
                    s.cpu_cull_ms +
                    s.cpu_draw_ms +
                    s.sync_time_ms +
                    s.swap_time_ms +
                    s.finish_ms +
                    s.plugin_ms +
                    s.isect_ms +
                    s.opencover_ms;

                // Residual = Framezeit minus bekannte CPU-Zeiten (nicht-negativ klemmen)
                double residual_time_ms = s.frame_duration_ms - cpu_known_sum_ms;
                if (residual_time_ms < 0.0) residual_time_ms = 0.0;

                frames_out
                    << s.frame_number << ";"
                    << s.frame_rate << ";"
                    << s.frame_duration_ms << ";"
                    << cpu_known_sum_ms << ";"
                    << residual_time_ms << ";"
                    << s.rendering_traversals_ms << ";"
                    << s.cpu_update_ms << ";"
                    << s.cpu_cull_ms << ";"
                    << s.cpu_draw_ms << ";"
                    << s.gpu_time_ms << ";"
                    << s.gpu_clock << ";"
                    << s.gpu_mem_clock << ";"
                    << s.gpu_util << ";"
                    << s.gpu_pci << ";"
                    << s.sync_time_ms << ";"
                    << s.swap_time_ms << ";"
                    << s.finish_ms << ";"
                    << s.isect_ms << ";"
                    << s.plugin_ms << ";"
                    << s.opencover_ms << ";"
                    << s.rendered_splats << ";"
                    << s.rendered_nodes << ";"
                    << s.rendered_bounding_boxes << ";"
                    << s.position.x() << ";"
                    << s.position.y() << ";"
                    << s.position.z() << ";"
                    << s.orientation.x() << ";"
                    << s.orientation.y() << ";"
                    << s.orientation.z() << ";"
                    << s.orientation.w() << ";"
                    << s.backoff_cull << ";"
                    << s.backoff_draw << ";"
                    << s.backoff_gpu
                    << "\n";
            }
        }
    }

    // ---------- Summary CSV (unverändert, nur falls du das brauchst) ----------
    {
        std::ofstream summary_out(summary_path);
        if (!summary_out) {
            std::cerr << "[Measurement] Fehler beim Öffnen von " << summary_path << "\n";
        } else {
            summary_out.imbue(std::locale::classic());
            summary_out << std::fixed << std::setprecision(4);

            const size_t n = m_stats.size();
            double total_duration_ms = 0, total_rate = 0;
            uint64_t total_splats = 0, total_nodes = 0, total_boxes = 0;
            double total_cpu_update = 0, total_cpu_cull = 0, total_cpu_draw = 0, total_gpu = 0, total_sync = 0, total_swap = 0;
            double total_isect = 0, total_plugin = 0, total_opencover = 0;
            double total_gpu_clock = 0, total_gpu_mem_clock = 0, total_gpu_util = 0, total_gpu_pci = 0;

            for (const auto& s : m_stats) {
                total_rate         += s.frame_rate;
                total_duration_ms  += s.frame_duration_ms;
                total_splats       += s.rendered_splats;
                total_nodes        += s.rendered_nodes;
                total_boxes        += s.rendered_bounding_boxes;
                total_cpu_update   += s.cpu_update_ms;
                total_cpu_cull     += s.cpu_cull_ms;
                total_cpu_draw     += s.cpu_draw_ms;
                total_gpu          += s.gpu_time_ms;
                total_sync         += s.sync_time_ms;
                total_swap         += s.swap_time_ms;
                total_isect        += s.isect_ms;
                total_plugin       += s.plugin_ms;
                total_opencover    += s.opencover_ms;
                total_gpu_clock    += s.gpu_clock;
                total_gpu_mem_clock+= s.gpu_mem_clock;
                total_gpu_util     += s.gpu_util;
                total_gpu_pci      += s.gpu_pci;
            }

            const double avg_frame_duration = total_duration_ms / n;
            const double avg_fps            = total_rate / n;

            summary_out << "Metric;Value\n";
            summary_out << "Total Frames;" << n << "\n";
            summary_out << "Total Time (s);" << total_duration_ms / 1000.0 << "\n";
            summary_out << "Avg Frame Time (ms);" << avg_frame_duration << "\n";
            summary_out << "Avg FPS;" << avg_fps << "\n";
            summary_out << "Avg Splats/Frame;" << (n ? double(total_splats)/n : 0.0) << "\n";
            summary_out << "Avg Nodes/Frame;"  << (n ? double(total_nodes)/n  : 0.0) << "\n";
            summary_out << "Avg Boxes/Frame;"  << (n ? double(total_boxes)/n  : 0.0) << "\n";
            summary_out << "Avg CPU Update (ms);" << (n ? total_cpu_update/n : 0.0) << "\n";
            summary_out << "Avg CPU Cull (ms);"   << (n ? total_cpu_cull/n   : 0.0) << "\n";
            summary_out << "Avg CPU Draw (ms);"   << (n ? total_cpu_draw/n   : 0.0) << "\n";
            summary_out << "Avg GPU Time (ms);"   << (n ? total_gpu/n        : 0.0) << "\n";
            summary_out << "Avg Sync Time (ms);"  << (n ? total_sync/n       : 0.0) << "\n";
            summary_out << "Avg Swap Time (ms);"  << (n ? total_swap/n       : 0.0) << "\n";
            summary_out << "Avg Isect (ms);"      << (n ? total_isect/n      : 0.0) << "\n";
            summary_out << "Avg Plugin (ms);"     << (n ? total_plugin/n     : 0.0) << "\n";
            summary_out << "Avg OpenCOVER (ms);"  << (n ? total_opencover/n  : 0.0) << "\n";
            summary_out << "Avg GPU Clock (MHz);"     << (n ? total_gpu_clock/n     : 0.0) << "\n";
            summary_out << "Avg GPU Mem Clock (MHz);" << (n ? total_gpu_mem_clock/n : 0.0) << "\n";
            summary_out << "Avg GPU Util (%);"        << (n ? total_gpu_util/n      : 0.0) << "\n";
            summary_out << "Avg GPU PCIe RX (KB/s);"  << (n ? total_gpu_pci/n       : 0.0) << "\n";
        }
    }


    if (m_exportTimeline) {
        writeTimelineCSV(timeline_path); // NEU
    }

    std::cout << "Measurement written:\n"
        << summary_path << "\n"
        << frames_path  << "\n"
        << (m_exportTimeline ? timeline_path : std::string("(timeline disabled)")) << "\n";

    // Aufräumen
    m_stats.clear();
    m_timeline.clear(); // NEU: doppelte Einträge bei Folge-Läufen vermeiden
    m_segmentTime = 0.0;
    m_currentSegment = 0;
    m_cumulativeRot = osg::Vec3(0.0f, 0.0f, 0.0f);
}


void LamureMeasurement::writeTimelineCSV(const std::string& path)
{
    std::stable_sort(m_timeline.begin(), m_timeline.end(),
        [](const TimeBlock& a, const TimeBlock& b){
            if (a.frame != b.frame)        return a.frame < b.frame;
            if (a.begin_ms != b.begin_ms)  return a.begin_ms < b.begin_ms;
            if (a.camIndex != b.camIndex)  return a.camIndex < b.camIndex;
            return a.name < b.name;
        });

    std::unordered_map<unsigned,double> frameTimeByFrame;
    frameTimeByFrame.reserve(m_stats.size());
    for (const auto& fs : m_stats)
        frameTimeByFrame[fs.frame_number] = fs.frame_duration_ms;

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[Measurement] Fehler beim Öffnen von " << path << "\n";
        return;
    }
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(4);

    // Header mit frame_time_ms direkt nach taken_ms
    out << "frame_number;"
        << "cam_index;"
        << "scope;"
        << "name;"
        << "begin_ms;"
        << "end_ms;"
        << "taken_ms;"
        << "frame_time_ms;"   // jetzt hier
        << "used_offset"
        << "\n";

    for (const auto& tb : m_timeline)
    {
        const double frame_time_ms = [&]{
            auto it = frameTimeByFrame.find(tb.frame);
            return (it != frameTimeByFrame.end()) ? it->second : 0.0;
            }();

        out << tb.frame << ";"
            << tb.camIndex << ";"
            << tb.scope << ";"
            << tb.name << ";"
            << tb.begin_ms << ";"
            << tb.end_ms << ";"
            << tb.taken_ms << ";"        // direkt davor
            << frame_time_ms << ";"
            << tb.used_offset
            << "\n";
    }
}