#include "LamureMeasurement.h"

#include <cover/VRViewer.h>
#include <osgViewer/Viewer>
#include <osg/Stats>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <locale>
#include <unordered_map>
#include <cover/coVRPluginSupport.h>
#include <map>
#include <set>
#include <filesystem>
#include <system_error>

#include "Lamure.h"

#include <lamure/ren/model_database.h>
#include <lamure/ren/bvh.h>

#ifdef _WIN32
#include <windows.h>
#include <cover/coVRConfig.h>

#ifndef LM_STR_HELPER
#define LM_STR_HELPER(x) #x
#define LM_STR(x) LM_STR_HELPER(x)
#endif

// --- GL_NVX_gpu_memory_info (für VRAM der *aktuellen* GL-GPU) ---
#ifndef GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX
#define GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX      0x9048
#define GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX    0x9049
#endif

// --- NVML dynamic loader ---
struct NvmlLoader {
    HMODULE h = nullptr; bool ok = false; void* dev = nullptr;
    using nvmlInit_t = int(*)(); using nvmlShutdown_t = int(*)();
    using nvmlDeviceGetHandleByIndex_t = int(*)(unsigned, void**);
    using nvmlDeviceGetUtilizationRates_t = int(*)(void*, void*);
    using nvmlDeviceGetMemoryInfo_t = int(*)(void*, void*);
    nvmlInit_t nvmlInit=nullptr; nvmlShutdown_t nvmlShutdown=nullptr;
    nvmlDeviceGetHandleByIndex_t nvmlDeviceGetHandleByIndex=nullptr;
    nvmlDeviceGetUtilizationRates_t nvmlDeviceGetUtilizationRates=nullptr;
    nvmlDeviceGetMemoryInfo_t nvmlDeviceGetMemoryInfo=nullptr;
    bool ensureLoaded(){
        if (ok) return true;
        if (!h) { h = LoadLibraryA("nvml.dll"); if (!h) return false; }
        nvmlInit = (nvmlInit_t)GetProcAddress(h,"nvmlInit_v2"); if(!nvmlInit) nvmlInit=(nvmlInit_t)GetProcAddress(h,"nvmlInit");
        nvmlShutdown=(nvmlShutdown_t)GetProcAddress(h,"nvmlShutdown");
        nvmlDeviceGetHandleByIndex=(nvmlDeviceGetHandleByIndex_t)GetProcAddress(h,"nvmlDeviceGetHandleByIndex_v2");
        if(!nvmlDeviceGetHandleByIndex) nvmlDeviceGetHandleByIndex=(nvmlDeviceGetHandleByIndex_t)GetProcAddress(h,"nvmlDeviceGetHandleByIndex");
        nvmlDeviceGetUtilizationRates=(nvmlDeviceGetUtilizationRates_t)GetProcAddress(h,"nvmlDeviceGetUtilizationRates");
        nvmlDeviceGetMemoryInfo=(nvmlDeviceGetMemoryInfo_t)GetProcAddress(h,"nvmlDeviceGetMemoryInfo");
        if(!nvmlInit||!nvmlShutdown||!nvmlDeviceGetHandleByIndex||!nvmlDeviceGetUtilizationRates||!nvmlDeviceGetMemoryInfo) return false;
        if (nvmlInit()!=0) return false;
        if (nvmlDeviceGetHandleByIndex(0,&dev)!=0) return false;
        ok=true; return true;
    }
    ~NvmlLoader(){ if(ok&&nvmlShutdown) nvmlShutdown(); if(h) FreeLibrary(h); }
};
struct NvmlUtilization { unsigned gpu; unsigned memory; };
struct NvmlMemory { unsigned long long total, free, used; };
static bool nvmlPoll(NvmlLoader& nv, double& gpuUtilPct, double& memUsedMB, double& memTotalMB){
    gpuUtilPct=memUsedMB=memTotalMB=0.0; if(!nv.ensureLoaded()) return false;
    NvmlUtilization u{}; if(nv.nvmlDeviceGetUtilizationRates(nv.dev,&u)==0) gpuUtilPct=double(u.gpu);
    NvmlMemory m{}; if(nv.nvmlDeviceGetMemoryInfo(nv.dev,&m)==0){ memTotalMB=m.total/1048576.0; memUsedMB=m.used/1048576.0; }
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
        unsigned f = fs ? fs->getFrameNumber()
            : (viewerStats ? viewerStats->getLatestFrameNumber() : 0);

        if (!cams.empty()) {
            if (auto* r = dynamic_cast<osgViewer::Renderer*>(cams.front()->getRenderer())) {
                if (!r->getGraphicsThreadDoesCull())
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

    inline void ensureParentDir(const std::filesystem::path& p) {
        const auto dir = p.has_filename() ? p.parent_path() : p;
        if (dir.empty()) return;
        std::error_code ec;
        (void)std::filesystem::create_directories(dir, ec);
        if (ec) {
            std::cerr << "[Measurement] create_directories failed for "
                << dir.string() << ": " << ec.message() << "\n";
        }
    }

    inline bool openCsv(std::ofstream& out, const std::filesystem::path& p) {
        ensureParentDir(p);
        // Unicode-sicher öffnen (C++17: ofstream::open(path) ist überladen)
        out.open(p, std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            std::error_code ec;
            auto abs = std::filesystem::absolute(p, ec);
            std::filesystem::path cwd;
            std::error_code ec2;
            cwd = std::filesystem::current_path(ec2);
            std::cerr << "[Measurement] Failed to open for write: "
                << (ec ? p.string() : abs.string())
                << " | cwd=" << (ec2 ? std::string("<unk>") : cwd.string())
                << "\n";
            return false;
        }
        out.imbue(std::locale::classic());
        out << std::fixed << std::setprecision(4);
        return true;
    }

    static inline double clampd(double v,double lo,double hi){ return std::max(lo,std::min(hi,v)); }
    static inline double pi() { return 3.14159265358979323846; }

    // Dichte/Abdeckung aus rohen Stats
    static inline void estimateCoverageCPU(
        uint64_t prims, double r_world, double z_view, double near_plane,
        double H_over_tmb, int W, int H, double k_orient,
        double s_min_px, double s_max_px, double scale_radius,
        double& out_C1, double& out_D, double& out_mult)
    {
        out_C1 = out_D = out_mult = 0.0;
        if (prims==0 || W<=0 || H<=0) return;
        if (!(z_view < -1e-6) || near_plane <= 0.0 || H_over_tmb <= 0.0) return;

        const double pixScale = (near_plane / -z_view) * H_over_tmb;
        const double diam_px  = clampd(2.0 * r_world * pixScale * scale_radius, s_min_px, s_max_px);
        const double r_px     = 0.5 * diam_px;
        if (r_px <= 0.0) return;

        const double A_one = k_orient * pi() * r_px * r_px;
        const double S = double(W) * double(H);
        const double D = (A_one * double(prims)) / S;
        const double C = 1.0 - std::exp(-D);

        out_D = std::max(0.0, D);
        out_C1 = clampd(C, 0.0, 1.0);
        out_mult = (C > 1e-12) ? (D / C) : 0.0;
    }

    inline const char* primitiveStrFromSettings(const Lamure::Settings& s) {
        if (s.point)       return "point";
        if (s.surfel)      return "surfel";
        if (s.splatting)   return "splat";
        using ST = LamureRenderer::ShaderType;
        switch (s.shader_type) {
        case ST::Point:
        case ST::PointColor:
        case ST::PointColorLighting: return "point";
        default:                     return "surfel";
        }
    }
}


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
    m_verbose = false;
    m_logEveryN = 60;
    m_dumpAttrs = false;
    m_gpuBackSearch = 16;
    m_exportTimeline = true;
    m_exportReport = true;
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
        vs->collectStats("gpu",        true);
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

    m_verbose   = false; // kein Konsolen-Spam
    m_logEveryN = 30;
    m_dumpAttrs = false;
    m_gpu_static_captured = false;
    m_gpu_mem_used_mb_static = m_gpu_mem_total_mb_static = 0.0;
    m_gpu_mem_used_mb_nvml_static = m_gpu_mem_total_mb_nvml_static = 0.0;
    m_gpu_mem_used_mb_gl_static = m_gpu_mem_total_mb_gl_static = 0.0;

    std::cout << "[Measurement] Measurement started\n";
}


LamureMeasurement::~LamureMeasurement()
{
    std::cout << "[Measurement] Measurement finished.\n";
}

bool LamureMeasurement::isActive() const noexcept {
    return m_running;
}

int LamureMeasurement::findStatsIndexForFrame(uint64_t frame_no) const {
    // Suche rückwärts im letzten Fenster – meist ist das der letzte Eintrag
    for (int i = int(m_stats.size()) - 1; i >= 0 && i >= int(m_stats.size()) - 32; --i) {
        if (m_stats[size_t(i)].frame_number == frame_no) return i;
    }
    return m_stats.empty() ? -1 : int(m_stats.size()) - 1;
}


void LamureMeasurement::stop() {
    m_running = false;
    if (!m_written) {
        writeLogAndStop();
    }
}


void LamureMeasurement::initCallbacks()
{
    auto cam = m_viewer->getCamera();
    m_preCB = new MarkCallback(this, true);
    m_postCB = new MarkCallback(this, false);
    cam->setPreDrawCallback(m_preCB);
    cam->setPostDrawCallback(m_postCB);
}

void LamureMeasurement::cacheStaticGpuInfo()
{
    if (m_gpu_static_captured)
        return;
    double used_nvml = 0.0, total_nvml = 0.0;
    double used_gl   = 0.0, total_gl   = 0.0;
    bool haveNvml = false, haveGl = false;
#ifdef _WIN32
    {
        static NvmlLoader g_nvml_once;
        double util = 0.0, used = 0.0, total = 0.0;
        if (nvmlPoll(g_nvml_once, util, used, total)) {
            used_nvml  = used;
            total_nvml = total;
            haveNvml   = true;
        }
    }
#endif
    if (glewIsSupported("GL_NVX_gpu_memory_info")) {
        GLint totalKB = 0, freeKB = 0;
        glGetIntegerv(GL_GPU_MEMORY_INFO_TOTAL_AVAILABLE_MEMORY_NVX,   &totalKB);
        glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &freeKB);
        if (totalKB > 0 && freeKB >= 0) {
            total_gl = totalKB / 1024.0;
            const double free_gl = freeKB / 1024.0;
            used_gl   = std::max(0.0, total_gl - free_gl);
            haveGl    = true;
        }
    }
    if (haveNvml) {
        m_gpu_mem_used_mb_nvml_static  = used_nvml;
        m_gpu_mem_total_mb_nvml_static = total_nvml;
    }
    if (haveGl) {
        m_gpu_mem_used_mb_gl_static    = used_gl;
        m_gpu_mem_total_mb_gl_static   = total_gl;
    }
    if (haveNvml) {
        m_gpu_mem_used_mb_static  = m_gpu_mem_used_mb_nvml_static;
        m_gpu_mem_total_mb_static = m_gpu_mem_total_mb_nvml_static;
    } else if (haveGl) {
        m_gpu_mem_used_mb_static  = m_gpu_mem_used_mb_gl_static;
        m_gpu_mem_total_mb_static = m_gpu_mem_total_mb_gl_static;
    } else {
        m_gpu_mem_used_mb_static  = 0.0;
        m_gpu_mem_total_mb_static = 0.0;
    }
    m_gpu_static_captured = true;
    std::cout << "[Measurement] Cached static GPU memory: primary used/total="
        << m_gpu_mem_used_mb_static << "/"
        << m_gpu_mem_total_mb_static << " MB"
        << (haveNvml ? " (NVML)" : (haveGl ? " (GL_NVX)" : " (none)"))
        << "\n";
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
    outMs = 0.0; usedOffset = 0;
    if (outBeginMs) *outBeginMs = 0.0;
    if (outEndMs)   *outEndMs   = 0.0;
    if (!s) return false;

    const unsigned earliest = s->getEarliestFrameNumber();
    const unsigned latest   = s->getLatestFrameNumber();
    if (baseFrame > latest) baseFrame = latest;

    for (unsigned off = 0; off <= backSearch; ++off)
    {
        if (baseFrame < off) break;
        const unsigned f = baseFrame - off;
        if (f < earliest) break;

        double vTaken = 0.0;
        if (s->getAttribute(f, timeTakenKey, vTaken)) {
            outMs = vTaken * 1000.0;
            usedOffset = off;
            if (outBeginMs || outEndMs) {
                double b=0.0, e=0.0;
                if (outBeginMs && s->getAttribute(f, beginKey, b)) *outBeginMs = b * 1000.0;
                if (outEndMs   && s->getAttribute(f, endKey,   e)) *outEndMs   = e * 1000.0;
            }
            return true;
        }

        double b=0.0, e=0.0;
        if (s->getAttribute(f, beginKey, b) && s->getAttribute(f, endKey, e) && e>=b) {
            outMs = (e-b) * 1000.0;
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

    if (!getTimeTakenMsBacksearch(stats, baseFrame, backsearch,
        statPrefix + " time taken",
        statPrefix + " begin time",
        statPrefix + " end time",
        taken_ms, usedOffset, &begin_ms, &end_ms))
        return false;

    TimeBlock tb;
    tb.frame       = baseFrame;
    tb.src_frame   = (usedOffset <= baseFrame) ? baseFrame - usedOffset : baseFrame;
    tb.camIndex    = camIndex;
    tb.scope       = scope;
    tb.name        = nameForCSV;
    tb.begin_ms    = begin_ms;
    tb.end_ms      = end_ms;
    tb.taken_ms    = taken_ms;
    tb.used_offset = usedOffset;
    // Ordne jeden Block dem Ursprungsframe (src_frame) zu – unabhängig vom erfassten Offset.
    // So bleibt die Frame-Zuordnung stabil gemäß Startzeit.
    tb.frame = tb.src_frame;

    localBlocks.push_back(std::move(tb));
    return true;
}

void LamureMeasurement::drawIncrement(bool preDraw, const osg::FrameStamp* frameStamp)
{
    if (!m_running) return;
    if (preDraw) {
        if (!m_gpu_static_tried) { cacheStaticGpuInfo(); m_gpu_static_tried = true; }
        return;
    }
    if (!frameStamp) return;

    // Stats sammeln
    {
        FrameStats s;
        const bool dbg = (m_verbose && (frameStamp->getFrameNumber() % m_logEveryN == 0));
        if (collectFrameStats(m_viewer, frameStamp, s, dbg))
            m_stats.push_back(s);
    }

    const double tNow = frameStamp->getReferenceTime();
    if (!m_haveSegmentStart) { m_segmentStartRefTime = tNow; m_haveSegmentStart = true; }

    // Hilfen für effektive Trans-Speed (wegen Scale*Gain in updateCamera)
    auto colLen = [](const osg::Matrix& M, int c){
        osg::Vec3 v(M(0,c), M(1,c), M(2,c));
        return std::max(1e-9, (double)v.length());
        };
    auto computeScale = [&](){
        const osg::Matrix dcsNow  = opencover::cover->getXformMat();
        const osg::Matrix baseM   = opencover::cover->getBaseMat();
        double sx = colLen(baseM, 0), sy = colLen(baseM, 1), sz = colLen(baseM, 2);
        const double avgBase = (sx + sy + sz) / 3.0;
        if (avgBase > 0.999 && avgBase < 1.001) {
            const osg::Matrix worldM = baseM * dcsNow;
            sx = colLen(worldM, 0);
            sy = colLen(worldM, 1);
            sz = colLen(worldM, 2);
        }
        return osg::Vec3d(sx, sy, sz);
        };
    auto dirScale = [&](const osg::Vec3& traMeters, const osg::Vec3d& S){
        const double L = osg::Vec3d(traMeters).length();
        if (L <= 1e-12) return 1.0;
        const osg::Vec3d u = osg::Vec3d(traMeters) / L;
        const double sx = S.x(), sy = S.y(), sz = S.z();
        return std::sqrt( (u.x()*sx)*(u.x()*sx) + (u.y()*sy)*(u.y()*sy) + (u.z()*sz)*(u.z()*sz) );
        };

    // Segmente vorrücken, wenn fertig
    while (m_currentSegment < m_segments.size())
    {
        const auto& seg = m_segments[m_currentSegment];

        const float  dist = seg.tra.length();
        const osg::Vec3d S = computeScale();
        const double sEff  = dirScale(seg.tra, S);
        const double transSpeedEff = (seg.transSpeed > 0.f ? seg.transSpeed * sEff : 0.0);
        const double transDur = (transSpeedEff > 0.0 && dist > 0.0) ? (dist / transSpeedEff) : 0.0;

        const double angleMag = std::max({ std::abs((double)seg.rot.x()),
            std::abs((double)seg.rot.y()),
            std::abs((double)seg.rot.z()) });
        const double rotDur   = (seg.rotSpeed > 0.f && angleMag > 0.0) ? (angleMag / seg.rotSpeed) : 0.0;

        // Finish-Together: die längere (≠0) bestimmt die Segmentdauer
        const double segDuration = (transDur > 0.0 && rotDur > 0.0)
            ? std::max(transDur, rotDur)
            : ((transDur > 0.0) ? transDur : rotDur);

        const double elapsed = tNow - m_segmentStartRefTime;
        if (elapsed < segDuration) break;

        m_cumulativeTra += seg.tra;
        m_cumulativeRot += seg.rot;
        ++m_currentSegment;
        m_segmentStartRefTime += segDuration;
    }

    if (m_currentSegment >= m_segments.size()) { stop(); return; }

    // Aktuelles Segment interpolieren
    const auto& seg = m_segments[m_currentSegment];

    const float  dist = seg.tra.length();
    const osg::Vec3d S = computeScale();
    const double sEff  = dirScale(seg.tra, S);
    const double transSpeedEff = (seg.transSpeed > 0.f ? seg.transSpeed * sEff : 0.0);
    const double transDur = (transSpeedEff > 0.0 && dist > 0.0) ? (dist / transSpeedEff) : 0.0;

    const double angleMag = std::max({ std::abs((double)seg.rot.x()),
        std::abs((double)seg.rot.y()),
        std::abs((double)seg.rot.z()) });
    const double rotDur   = (seg.rotSpeed > 0.f && angleMag > 0.0) ? (angleMag / seg.rotSpeed) : 0.0;

    const double segDuration = (transDur > 0.0 && rotDur > 0.0)
        ? std::max(transDur, rotDur)
        : ((transDur > 0.0) ? transDur : rotDur);

    const double elapsed = std::max(0.0, tNow - m_segmentStartRefTime);
    const double frac = (segDuration > 0.0) ? std::clamp(elapsed / segDuration, 0.0, 1.0) : 1.0;

    const osg::Vec3 tra = m_cumulativeTra + seg.tra * static_cast<float>(frac);
    const osg::Vec3 rot = m_cumulativeRot + seg.rot * static_cast<float>(frac);

    updateCamera(tra, rot);
}

void LamureMeasurement::updateCamera(const osg::Vec3& traAbs, const osg::Vec3& rotAbsDeg)
{
    if (!m_havePoseDeltas) {
        m_lastTraApplied = traAbs;
        m_lastRotApplied = rotAbsDeg;
        m_havePoseDeltas = true;
        return;
    }

    // Inkremente seit letztem Frame (Meter bzw. Grad)
    const osg::Vec3 dTra = traAbs - m_lastTraApplied;
    const osg::Vec3 dRot = rotAbsDeg - m_lastRotApplied;

    m_lastTraApplied = traAbs;
    m_lastRotApplied = rotAbsDeg;

    const double dPitch = osg::DegreesToRadians(dRot.x()); // um lokale X
    const double dRoll  = osg::DegreesToRadians(dRot.y()); // um lokale Y
    const double dYaw   = osg::DegreesToRadians(dRot.z()); // um lokale Z

    osg::Matrix dcs = opencover::cover->getXformMat();

    // Lokale Achsen aus aktueller Viewer-Orientierung holen
    const osg::Matrixd viewerM = opencover::cover->getViewerMat();
    const osg::Vec3 viewerPos  = viewerM.getTrans();
    osg::Vec3 axisX(viewerM(0,0), viewerM(1,0), viewerM(2,0));
    osg::Vec3 axisY(viewerM(0,1), viewerM(1,1), viewerM(2,1));
    osg::Vec3 axisZ(viewerM(0,2), viewerM(1,2), viewerM(2,2));
    axisX.normalize(); axisY.normalize(); axisZ.normalize();

    // Rotation um eigene Achsen (in dieser Reihenfolge: Yaw -> Pitch -> Roll)
    dcs.postMult(osg::Matrix::translate(-viewerPos));
    if (std::abs(dYaw)   > 0.0) dcs.postMult(osg::Matrix::rotate(dYaw,   axisZ));
    if (std::abs(dPitch) > 0.0) dcs.postMult(osg::Matrix::rotate(dPitch, axisX));
    if (std::abs(dRoll)  > 0.0) dcs.postMult(osg::Matrix::rotate(dRoll,  axisY));
    dcs.postMult(osg::Matrix::translate(viewerPos));

    // ---- Translation: Meter -> Welt (Skalierung berücksichtigen, Multiplikations-Variante) ----
    auto colLen = [](const osg::Matrix& M, int c){
        osg::Vec3 v(M(0,c), M(1,c), M(2,c));
        return std::max(1e-9, (double)v.length());
        };

    const osg::Matrix baseM  = opencover::cover->getBaseMat();
    double sx = colLen(baseM, 0), sy = colLen(baseM, 1), sz = colLen(baseM, 2);
    const double avgBase = (sx + sy + sz) / 3.0;
    if (avgBase > 0.999 && avgBase < 1.001) {
        const osg::Matrix worldM = baseM * dcs;
        sx = colLen(worldM, 0);
        sy = colLen(worldM, 1);
        sz = colLen(worldM, 2);
    }

    const osg::Vec3 dTraLocal(
        static_cast<float>(dTra.x() * sx),
        static_cast<float>(dTra.y() * sy),
        static_cast<float>(dTra.z() * sz)
    );

    dcs.postMult(osg::Matrix::translate(dTraLocal));
    opencover::cover->setXformMat(dcs);
}


bool LamureMeasurement::collectFrameStats(osgViewer::ViewerBase* viewer,
    const osg::FrameStamp* fs, FrameStats& stats, bool debugPrint)
{
    if (!viewer) return false;

    osg::Stats* viewerStats = viewer->getViewerStats();
    osgViewer::ViewerBase::Cameras cams;
    viewer->getCameras(cams);

    const unsigned f = pickStableFrame(viewer, viewerStats, cams, fs);
    stats = FrameStats{};
    stats.frame_number = f;

    if (viewerStats)
    {
        double hz = 0.0;
        if (viewerStats->getAttribute(f, "Frame rate", hz)) stats.frame_rate = hz;

        double ft = 0.0;
        if (viewerStats->getAttribute(f, "Frame duration", ft)) stats.frame_duration_ms = ft * 1000.0;

        double rtv=0.0; unsigned rtoff=0;
        if (getTimeTakenMsBacksearch(viewerStats, f, m_gpuBackSearch,
            "Rendering traversals time taken",
            "Rendering traversals begin time","Rendering traversals end time",
            rtv, rtoff))
        {
            stats.rendering_traversals_ms = rtv;
            if (stats.frame_duration_ms<=0.0) stats.frame_duration_ms = rtv;
        }
    }

    std::vector<TimeBlock> blocks;
    auto addViewer = [&](const std::string& prefix, const char* csvName){
        double ms=0.0; unsigned off=0;
        if (viewerStats && getTimeTakenMsBacksearch(viewerStats, f, m_gpuBackSearch,
            prefix + " time taken", prefix + " begin time", prefix + " end time", ms, off))
        {
            if      (std::string(csvName) == "Update traversal") stats.cpu_update_ms = ms;
            else if (std::string(csvName) == "sync")             stats.sync_time_ms  = ms;
            else if (std::string(csvName) == "swap")             stats.swap_time_ms  = ms;
            else if (std::string(csvName) == "finish")           stats.finish_ms     = ms;
            else if (std::string(csvName) == "Plugin")           stats.plugin_ms     = ms;
            else if (std::string(csvName) == "Isect")            stats.isect_ms      = ms;
            else if (std::string(csvName) == "opencover")        stats.opencover_ms  = ms;
        }
        if (viewerStats) (void)tryAddBlock(viewerStats, f, m_gpuBackSearch, prefix, csvName, "viewer", -1, blocks);
        };
    addViewer("Update traversal", "Update traversal");
    addViewer("sync",            "sync");
    addViewer("swap",            "swap");
    addViewer("finish",          "finish");
    addViewer("Plugin",          "Plugin");
    addViewer("Isect",           "Isect");
    addViewer("opencover",       "opencover");

    double sumCull_ms = 0.0, sumDraw_ms = 0.0, sumGpu_ms = 0.0;

    for (size_t i = 0; i < cams.size(); ++i)
    {
        osg::Camera* cam = cams[i];
        osg::Stats*  cs  = cam ? cam->getStats() : nullptr;
        if (!cs) continue;

        unsigned adjustedBase = stats.frame_number;
        if (auto* rnd = dynamic_cast<osgViewer::Renderer*>(cam->getRenderer()))
            if (!rnd->getGraphicsThreadDoesCull() && adjustedBase > 0) --adjustedBase;

        double cull_ms = 0.0, draw_ms = 0.0, gpu_ms = 0.0;
        unsigned offCull = 0, offDraw = 0, offGpu = 0;

        if (getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
            "Cull traversal time taken",
            "Cull traversal begin time", "Cull traversal end time",
            cull_ms, offCull))
        {
            stats.backoff_cull = std::max(stats.backoff_cull, offCull);
            sumCull_ms += cull_ms;
        }
        (void)tryAddBlock(cs, adjustedBase, m_gpuBackSearch, "Cull traversal", "Cull traversal", "camera", int(i), blocks);

        if (getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
            "Draw traversal time taken",
            "Draw traversal begin time", "Draw traversal end time",
            draw_ms, offDraw))
        {
            stats.backoff_draw = std::max(stats.backoff_draw, offDraw);
            sumDraw_ms += draw_ms;
        }
        (void)tryAddBlock(cs, adjustedBase, m_gpuBackSearch, "Draw traversal", "Draw traversal", "camera", int(i), blocks);

        if (!getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
            "GPU draw time taken",
            "GPU draw begin time", "GPU draw end time",
            gpu_ms, offGpu))
        {
            getTimeTakenMsBacksearch(cs, adjustedBase, m_gpuBackSearch,
                "GPU time taken", "GPU begin time", "GPU end time",
                gpu_ms, offGpu);
        }
        if (gpu_ms > 0.0) {
            stats.backoff_gpu = std::max(stats.backoff_gpu, offGpu);
            sumGpu_ms += gpu_ms;
        }

        if (!tryAddBlock(cs, adjustedBase, m_gpuBackSearch, "GPU draw", "GPU draw", "camera", int(i), blocks))
            (void)tryAddBlock(cs, adjustedBase, m_gpuBackSearch, "GPU", "GPU time", "camera", int(i), blocks);
    }

    stats.cpu_cull_ms = sumCull_ms;
    stats.cpu_draw_ms = sumDraw_ms;
    stats.gpu_time_ms = sumGpu_ms;

    double v=0.0;
    if (viewerStats && viewerStats->getAttribute(f, "GPU Clock MHz", v))      stats.gpu_clock = v;
    if (viewerStats && viewerStats->getAttribute(f, "GPU Mem Clock MHz", v))  stats.gpu_mem_clock = v;
    if (viewerStats && viewerStats->getAttribute(f, "GPU Utilization", v))    stats.gpu_util = v;
    if (viewerStats && viewerStats->getAttribute(f, "GPU PCIe rx KB/s", v))   stats.gpu_pci = v;

    cacheStaticGpuInfo();

    const osg::Camera* camPose = (!cams.empty() && cams.front()) ? cams.front()
        : (m_viewer ? m_viewer->getCamera() : nullptr);

    if (camPose) {
        const osg::Matrixd worldToView = opencover::cover->getBaseMat() * camPose->getViewMatrix();
        osg::Matrixd viewToWorld;
        if (viewToWorld.invert(worldToView)) {
            osg::Vec3d T, S; osg::Quat R, SO;
            viewToWorld.decompose(T, R, S, SO);

            if (m_haveLastQuat) {
                const double dot = R.x()*m_lastQuat.x() + R.y()*m_lastQuat.y()
                    + R.z()*m_lastQuat.z() + R.w()*m_lastQuat.w();
                if (dot < 0.0) R.set(-R.x(), -R.y(), -R.z(), -R.w());
            }
            m_lastQuat     = R;
            m_haveLastQuat = true;

            stats.position    = T;
            stats.orientation = R;
        }
    }

    const auto ri = m_plugin->getRenderInfo();

    stats.rendered_splats         = ri.rendered_primitives;
    stats.rendered_nodes          = ri.rendered_nodes;
    stats.rendered_bounding_boxes = ri.rendered_bounding_boxes;

    stats.est_screen_px    = ri.est_screen_px;
    stats.est_sum_area_px  = ri.est_sum_area_px;
    stats.est_density      = ri.est_density;
    stats.est_coverage     = ri.est_coverage;
    stats.est_coverage_px  = ri.est_coverage_px;
    stats.est_overdraw     = ri.est_overdraw;
    stats.avg_area_px_per_prim = ri.avg_area_px_per_prim;

    if ((stats.est_density <= 0.0f || stats.est_coverage <= 0.0f) &&
        (stats.est_sum_area_px > 0.0f && stats.est_screen_px > 0.0f))
    {
        const double D = double(stats.est_sum_area_px) / std::max(1.0, double(stats.est_screen_px));
        stats.est_density  = float(std::max(0.0, D));
        stats.est_coverage = float(1.0 - std::exp(-D));
        stats.est_coverage_px = stats.est_coverage * stats.est_screen_px;
        stats.est_overdraw    = (stats.est_coverage_px > 0.0f)
            ? float(double(stats.est_sum_area_px) / double(stats.est_coverage_px))
            : 0.0f;
    }

    if (m_exportTimeline && !blocks.empty())
        m_timeline.insert(m_timeline.end(), blocks.begin(), blocks.end());

    if (debugPrint || (m_verbose && ((stats.frame_number % m_logEveryN) == 0))) {
        std::cout << std::fixed << std::setprecision(3)
            << "[LamureMeasurement] f=" << stats.frame_number
            << " | frame=" << stats.frame_duration_ms
            << " | upd=" << stats.cpu_update_ms
            << " | cull=" << stats.cpu_cull_ms
            << " | draw=" << stats.cpu_draw_ms
            << " | gpu=" << stats.gpu_time_ms
            << " | swap=" << stats.swap_time_ms
            << " | sync=" << stats.sync_time_ms
            << " | plug=" << stats.plugin_ms
            << " | cover=" << stats.opencover_ms
            << "\n";
    }

    const double ft = (stats.frame_duration_ms > 0.0) ? stats.frame_duration_ms : stats.rendering_traversals_ms;
    const double cpu_main =
        stats.cpu_update_ms + stats.cpu_cull_ms + stats.cpu_draw_ms +
        stats.plugin_ms + stats.isect_ms + stats.opencover_ms;
    const double waits = stats.sync_time_ms + stats.swap_time_ms + stats.finish_ms;

    stats.cpu_main_ms = cpu_main;
    stats.wait_ms     = waits;

    if (ft > 0.0) {
        stats.cpu_busy_pct_proxy = std::clamp(100.0 * (cpu_main / ft), 0.0, 100.0);
        const double gpu_clip    = std::min(stats.gpu_time_ms, ft);
        stats.gpu_busy_pct_proxy = std::clamp(100.0 * (gpu_clip / ft), 0.0, 100.0);
        stats.wait_frac_pct      = std::max(0.0, 100.0 * (waits / ft));
    } else {
        stats.cpu_busy_pct_proxy = 0.0;
        stats.gpu_busy_pct_proxy = 0.0;
        stats.wait_frac_pct      = 0.0;
    }

    auto isNa = [](double v){ return !(v==v); };
    if (ft <= 0.0 || isNa(stats.cpu_busy_pct_proxy) || isNa(stats.gpu_busy_pct_proxy)) {
        stats.boundness = "unknown";
    } else if (stats.gpu_busy_pct_proxy > 70.0 && stats.cpu_busy_pct_proxy < 60.0 && stats.wait_frac_pct < 10.0) {
        stats.boundness = "GPU-bound";
    } else if (stats.cpu_busy_pct_proxy > 70.0 && stats.gpu_busy_pct_proxy < 60.0) {
        stats.boundness = "CPU-bound";
    } else if (stats.wait_frac_pct > 20.0) {
        stats.boundness = "Wait/Sync-bound";
    } else {
        stats.boundness = "mixed";
    }

    // <<< FrameMarks direkt aus dem Plugin in die Stats übernehmen
    {
        const FrameMarks& fm = m_plugin->getFrameMarks();
        stats.mark_draw_impl_ms     = fm.draw_cb_ms;
        stats.mark_pass1_ms         = fm.pass1_ms;
        stats.mark_pass2_ms         = fm.pass2_ms;
        stats.mark_pass3_ms         = fm.pass3_ms;
        stats.mark_singlepass_ms    = fm.singlepass_ms;
        stats.mark_dispatch_ms      = fm.dispatch_ms;
        stats.mark_context_bind_ms  = fm.context_bind_ms;
        stats.mark_estimates_ms     = fm.estimates_ms;
    }
    // >>>

    stats.segment_index = static_cast<int>(m_currentSegment);
    return true;
}


static inline std::string csvQuote(const std::string& s) {
    if (s.find_first_of(",;\t\" \n\r") == std::string::npos) return s;
    std::string r; r.reserve(s.size()+8);
    r.push_back('"');
    for (char c : s) r += (c=='"') ? std::string("\"\"") : std::string(1,c);
    r.push_back('"');
    return r;
}

void LamureMeasurement::writeTimelineCSV(const std::string& path)
{
    // Ordner sicherstellen
    try {
        std::filesystem::path p(path);
        if (p.has_parent_path())
            std::filesystem::create_directories(p.parent_path());
    } catch (...) {
        // ignorieren – wir versuchen einfach zu schreiben
    }

    cacheStaticGpuInfo();

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::cerr << "[Measurement] Failed to open timeline CSV: " << path << "\n";
        return;
    }

    out.setf(std::ios::fixed);
    out << std::setprecision(3);

    std::vector<TimeBlock> blocks = m_timeline;
    std::sort(blocks.begin(), blocks.end(), [](const TimeBlock& a, const TimeBlock& b){
        if (a.src_frame != b.src_frame) return a.src_frame < b.src_frame;
        if (a.begin_ms  != b.begin_ms)  return a.begin_ms  < b.begin_ms;
        if (a.camIndex  != b.camIndex)  return a.camIndex  < b.camIndex;
        return a.name < b.name;
        });

    out << "frame;src_frame;cam;scope;name;begin_ms;end_ms;taken_ms;used_offset\n";

    for (const auto& b : blocks) {
        out << b.frame << ';'
            << b.src_frame << ';'
            << b.camIndex << ';'
            << csvQuote(b.scope) << ';'
            << csvQuote(b.name)  << ';'
            << b.begin_ms << ';'
            << b.end_ms   << ';'
            << b.taken_ms << ';'
            << b.used_offset
            << '\n';
    }

    out.flush();
    std::cout << "[Measurement] Timeline CSV written: " << path << " (" << blocks.size() << " rows)\n";
}


void LamureMeasurement::writeLogAndStop()
{
    if (m_written) {
        std::cerr << "[Measurement] writeLogAndStop() already executed, skipping.\n";
        return;
    }
    m_written = true;

    cacheStaticGpuInfo();

    std::filesystem::path base_path(m_logfile);
    if (base_path.empty()) {
        base_path = std::filesystem::path("lamure_measurement");
    }
    if (base_path.has_extension())
        base_path.replace_extension();

    const std::filesystem::path frames_path   = base_path.string() + "_frames.csv";
    const std::filesystem::path summary_path  = base_path.string() + "_summary.csv";
    const std::filesystem::path timeline_path = base_path.string() + "_timeline.csv";
    const std::filesystem::path md_path       = base_path.string() + "_report.md";
    m_reportMDPath = md_path.string();

    // --- GPU-Zeit je Frame aus Timeline aggregieren (alle GPU-Blöcke aufsummieren) ---
    std::unordered_map<unsigned, double> gpuMsByFrame;
    gpuMsByFrame.reserve(m_timeline.size());
    for (const auto &tb : m_timeline) {
        if (tb.name.find("GPU") != std::string::npos) {
            gpuMsByFrame[tb.frame] += tb.taken_ms;
        }
    }

    // --- Coverage-Aggregate für Summary/Markdown ---
    double sum_covC1 = 0.0;
    double sum_covD  = 0.0;
    int    cnt_cov   = 0;

    // --- Frames CSV ---
    {
        std::ofstream frames_out;
        if (!openCsv(frames_out, frames_path)) {
            return;
        }

        frames_out
            << "frame_number;"
            << "frame_rate;"
            << "frame_time_ms;"
            << "cpu_known_sum_ms;"
            << "residual_time_ms;"
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
            << "cpu_main_ms;"
            << "cpu_busy_pct_proxy;"
            << "gpu_busy_pct_proxy;"
            << "wait_ms;"
            << "wait_frac_pct;"
            << "boundness;"
            << "rendered_splats;"
            << "rendered_nodes;"
            << "rendered_bounding_boxes;"
            << "est_screen_px;est_sum_area_px;est_density;est_coverage;est_coverage_px;est_overdraw;"
            << "pos_x;pos_y;pos_z;"
            << "quat_x;quat_y;quat_z;quat_w;"
            << "backoff_cull;backoff_draw;backoff_gpu;"
            << "segment_index;"
            << "cov_C1;cov_D;cov_mult;"
            << "rpts_points_per_ms;"
            << "avg_area_px_per_prim;"
            << "k_orient_used;"
            // --- Zeitmarken/Scopes in FrameStats ---
            << "mark_draw_impl_ms;"
            << "mark_pass1_ms;"
            << "mark_pass2_ms;"
            << "mark_pass3_ms;"
            << "mark_singlepass_ms;"
            << "mark_dispatch_ms;"
            << "mark_context_bind_ms;"
            << "mark_estimates_ms"
            << "\n";

        // Map für schnellen Zugriff pro Frame
        std::map<unsigned, FrameStats> statsByFrame;
        for (const auto &ms : m_stats) {
            statsByFrame.emplace(ms.frame_number, ms);
        }

        // Menge aller Frames, die vorkommen (Stats ∪ GPU-Aggregate)
        std::set<unsigned> frameIds;
        for (const auto &kv : statsByFrame)  frameIds.insert(kv.first);
        for (const auto &kv : gpuMsByFrame)  frameIds.insert(kv.first);

        for (unsigned fid : frameIds) {
            FrameStats s{};
            if (auto itS = statsByFrame.find(fid); itS != statsByFrame.end())
                s = itS->second;
            else
                s.frame_number = fid;

            // GPU-Zeit für dieses Frame
            const double gpu_ms_for_frame = [&](){
                auto it = gpuMsByFrame.find(fid);
                return (it != gpuMsByFrame.end()) ? it->second : 0.0;
                }();

            const double rpts_points_per_ms =
                (gpu_ms_for_frame > 0.0) ? (static_cast<double>(s.rendered_splats) / gpu_ms_for_frame) : 0.0;

            // Coverage direkt aus Renderer-Estimates
            double cov_C1 = 0.0, cov_D = 0.0, cov_mult = 0.0;
            if (s.est_density > 0.0 || s.est_coverage > 0.0) {
                cov_D  = std::max(0.0, double(s.est_density));
                cov_C1 = (s.est_coverage > 0.0) ? double(s.est_coverage)
                    : (cov_D > 0.0 ? 1.0 - std::exp(-cov_D) : 0.0);
                cov_mult = (cov_C1 > 1e-12) ? (cov_D / cov_C1) : 0.0;
            }

            if (cov_C1 > 0.0 || cov_D > 0.0) {
                sum_covC1 += cov_C1;
                sum_covD  += cov_D;
                ++cnt_cov;
            }

            const double cpu_known_sum_ms =
                s.cpu_update_ms + s.cpu_cull_ms + s.cpu_draw_ms +
                s.sync_time_ms + s.swap_time_ms + s.finish_ms +
                s.isect_ms + s.plugin_ms + s.opencover_ms;

            double residual_time_ms = s.frame_duration_ms - cpu_known_sum_ms;
            if (residual_time_ms < 0.0) residual_time_ms = 0.0;

            // k_orient_used: analog zum Renderer aus Settings ableiten
            double k_orient_used = 0.70;
            {
                const auto& st = m_plugin->getSettings();
                if (st.point) k_orient_used = 1.0;
                else if (st.surfel || st.splatting) k_orient_used = 0.70;
            }

            frames_out
                << fid << ";"
                << s.frame_rate << ";"
                << s.frame_duration_ms << ";"
                << cpu_known_sum_ms << ";"
                << residual_time_ms << ";"
                << s.rendering_traversals_ms << ";"
                << s.cpu_update_ms << ";"
                << s.cpu_cull_ms << ";"
                << s.cpu_draw_ms << ";"
                << gpu_ms_for_frame << ";"
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
                << s.cpu_main_ms << ";"
                << s.cpu_busy_pct_proxy << ";"
                << s.gpu_busy_pct_proxy << ";"
                << s.wait_ms << ";"
                << s.wait_frac_pct << ";"
                << s.boundness << ";"
                << s.rendered_splats << ";"
                << s.rendered_nodes << ";"
                << s.rendered_bounding_boxes << ";"
                << s.est_screen_px << ";" << s.est_sum_area_px << ";" << s.est_density << ";" << s.est_coverage << ";" << s.est_coverage_px << ";" << s.est_overdraw << ";"
                << s.position.x() << ";" << s.position.y() << ";" << s.position.z() << ";"
                << s.orientation.x() << ";" << s.orientation.y() << ";" << s.orientation.z() << ";" << s.orientation.w() << ";"
                << s.backoff_cull << ";" << s.backoff_draw << ";" << s.backoff_gpu << ";"
                << s.segment_index << ";"
                << cov_C1 << ";" << cov_D << ";" << cov_mult << ";"
                << rpts_points_per_ms << ";"
                << double(s.avg_area_px_per_prim) << ";"
                << k_orient_used << ";"
                << s.mark_draw_impl_ms << ";"
                << s.mark_pass1_ms << ";"
                << s.mark_pass2_ms << ";"
                << s.mark_pass3_ms << ";"
                << s.mark_singlepass_ms << ";"
                << s.mark_dispatch_ms << ";"
                << s.mark_context_bind_ms << ";"
                << s.mark_estimates_ms
                << "\n";
        }
        frames_out.flush();
    }

    // === Perzentile & Jitter & Stutter ===
    std::vector<double> ft_ms; ft_ms.reserve(m_stats.size());
    for (const auto &ms : m_stats) {
        double ft = (ms.frame_duration_ms > 0.0) ? ms.frame_duration_ms : ms.rendering_traversals_ms;
        if (ft > 0.0) ft_ms.push_back(ft);
    }
    auto percentile = [](std::vector<double> v, double p)->double{
        if (v.empty()) return 0.0;
        p = std::max(0.0, std::min(1.0, p));
        const size_t k = static_cast<size_t>(p * (v.size() - 1));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
        };
    const double p50 = percentile(ft_ms, 0.50);
    const double p95 = percentile(ft_ms, 0.95);
    const double p99 = percentile(ft_ms, 0.99);
    const double jitter_J = (p99 > p50) ? (p99 - p50) : 0.0;

    int stutter_count_2xmedian = 0;
    int stutter_count_33ms = 0; // 30 FPS-Schwelle ~33.33 ms
    for (double ft : ft_ms) {
        if (ft > 2.0 * p50)   ++stutter_count_2xmedian;
        if (ft > 33.333)      ++stutter_count_33ms;
    }

    // === Ø-Rpts über Frames ===
    double sum_rpts = 0.0;
    int    cnt_rpts = 0;
    for (const auto &ms : m_stats) {
        auto it = gpuMsByFrame.find(ms.frame_number);
        const double gpu_ms = (it != gpuMsByFrame.end()) ? it->second : 0.0;
        if (gpu_ms > 0.0 && ms.rendered_splats > 0) {
            sum_rpts += static_cast<double>(ms.rendered_splats) / gpu_ms;
            ++cnt_rpts;
        }
    }
    const double avg_rpts_points_per_ms = (cnt_rpts > 0) ? (sum_rpts / cnt_rpts) : 0.0;

    // --- Coverage-Durchschnitt ---
    const double avg_covC1 = (cnt_cov > 0) ? (sum_covC1 / cnt_cov) : 0.0;
    const double avg_covD  = (cnt_cov > 0) ? (sum_covD  / cnt_cov) : 0.0;

    // === Renderer-/Estimate-Ø (für Summary/Report) ===
    const size_t nFrames = m_stats.size();
    double total_est_screen_px   = 0.0;
    double total_est_sum_area_px = 0.0;
    double total_est_density     = 0.0;
    double total_est_coverage    = 0.0;
    double total_est_coverage_px = 0.0;
    double total_est_overdraw    = 0.0;
    double total_avg_area_px_per_prim = 0.0;

    // Zeitmarken-Summen
    double sum_mark_draw_impl_ms    = 0.0;
    double sum_mark_pass1_ms        = 0.0;
    double sum_mark_pass2_ms        = 0.0;
    double sum_mark_pass3_ms        = 0.0;
    double sum_mark_dispatch_ms     = 0.0;
    double sum_mark_context_bind_ms = 0.0;
    double sum_mark_estimates_ms    = 0.0;
    double sum_mark_singlepass_ms = 0.0;

    for (const auto& s : m_stats) {
        total_est_screen_px        += s.est_screen_px;
        total_est_sum_area_px      += s.est_sum_area_px;
        total_est_density          += s.est_density;
        total_est_coverage         += s.est_coverage;
        total_est_coverage_px      += s.est_coverage_px;
        total_est_overdraw         += s.est_overdraw;
        total_avg_area_px_per_prim += double(s.avg_area_px_per_prim);

        sum_mark_draw_impl_ms    += s.mark_draw_impl_ms;
        sum_mark_pass1_ms        += s.mark_pass1_ms;
        sum_mark_pass2_ms        += s.mark_pass2_ms;
        sum_mark_pass3_ms        += s.mark_pass3_ms;
        sum_mark_dispatch_ms     += s.mark_dispatch_ms;
        sum_mark_context_bind_ms += s.mark_context_bind_ms;
        sum_mark_estimates_ms    += s.mark_estimates_ms;
        sum_mark_singlepass_ms   += s.mark_singlepass_ms;
    }

    const double avg_est_screen_px        = (nFrames ? total_est_screen_px        / nFrames : 0.0);
    const double avg_est_sum_area_px      = (nFrames ? total_est_sum_area_px      / nFrames : 0.0);
    const double avg_est_density          = (nFrames ? total_est_density          / nFrames : 0.0);
    const double avg_est_coverage         = (nFrames ? total_est_coverage         / nFrames : 0.0);
    const double avg_est_coverage_px      = (nFrames ? total_est_coverage_px      / nFrames : 0.0);
    const double avg_est_overdraw         = (nFrames ? total_est_overdraw         / nFrames : 0.0);
    const double avg_avg_area_px_per_prim = (nFrames ? total_avg_area_px_per_prim / nFrames : 0.0);

    // Zeitmarken-Mittel
    const double avg_mark_draw_impl_ms    = (nFrames ? sum_mark_draw_impl_ms    / nFrames : 0.0);
    const double avg_mark_pass1_ms        = (nFrames ? sum_mark_pass1_ms        / nFrames : 0.0);
    const double avg_mark_pass2_ms        = (nFrames ? sum_mark_pass2_ms        / nFrames : 0.0);
    const double avg_mark_pass3_ms        = (nFrames ? sum_mark_pass3_ms        / nFrames : 0.0);
    const double avg_mark_dispatch_ms     = (nFrames ? sum_mark_dispatch_ms     / nFrames : 0.0);
    const double avg_mark_context_bind_ms = (nFrames ? sum_mark_context_bind_ms / nFrames : 0.0);
    const double avg_mark_estimates_ms    = (nFrames ? sum_mark_estimates_ms    / nFrames : 0.0);
    const double avg_mark_singlepass_ms   = (nFrames ? sum_mark_singlepass_ms / nFrames : 0.0);

    // --- Summary CSV ---
    {
        std::ofstream summary_out;
        if (!openCsv(summary_out, summary_path)) {
            std::cerr << "[Measurement] Could not write summary to "
                << summary_path.string() << "\n";
        } else {
            size_t n = m_stats.size();
            double total_duration_ms = 0, total_rate = 0;
            uint64_t total_splats = 0, total_nodes = 0, total_boxes = 0;
            double total_cpu_update = 0, total_cpu_cull = 0, total_cpu_draw = 0, total_gpu = 0, total_sync = 0, total_swap = 0;
            double total_isect = 0, total_plugin = 0, total_opencover = 0, total_finish = 0;
            double total_cpu_main = 0, total_wait_ms = 0, total_cpu_busy_pct = 0, total_gpu_busy_pct = 0, total_wait_frac_pct = 0;

            for (const auto& s : m_stats) {
                total_rate        += s.frame_rate;
                total_duration_ms += s.frame_duration_ms;
                total_splats      += s.rendered_splats;
                total_nodes       += s.rendered_nodes;
                total_boxes       += s.rendered_bounding_boxes;
                total_cpu_update  += s.cpu_update_ms;
                total_cpu_cull    += s.cpu_cull_ms;
                total_cpu_draw    += s.cpu_draw_ms;
                auto it = gpuMsByFrame.find(s.frame_number);
                total_gpu         += (it != gpuMsByFrame.end()) ? it->second : 0.0;
                total_sync        += s.sync_time_ms;
                total_swap        += s.swap_time_ms;
                total_isect       += s.isect_ms;
                total_plugin      += s.plugin_ms;
                total_opencover   += s.opencover_ms;
                total_finish      += s.finish_ms;
                total_cpu_main    += s.cpu_main_ms;
                total_wait_ms     += s.wait_ms;
                total_cpu_busy_pct+= s.cpu_busy_pct_proxy;
                total_gpu_busy_pct+= s.gpu_busy_pct_proxy;
                total_wait_frac_pct+= s.wait_frac_pct;
            }

            summary_out << std::fixed << std::setprecision(4);
            summary_out << "Metric;Value\n";
            summary_out << "Total Frames;" << n << "\n";
            summary_out << "Total Time (s);" << total_duration_ms / 1000.0 << "\n";
            summary_out << "Avg Frame Time (ms);" << (n ? total_duration_ms / n : 0.0) << "\n";
            summary_out << "Avg FPS;" << (n ? total_rate / n : 0.0) << "\n";
            summary_out << "Avg Splats/Frame;" << (n ? double(total_splats) / n : 0.0) << "\n";
            summary_out << "Avg Nodes/Frame;"  << (n ? double(total_nodes)  / n : 0.0) << "\n";
            summary_out << "Avg Boxes/Frame;"  << (n ? double(total_boxes)  / n : 0.0) << "\n";
            summary_out << "Avg CPU Update (ms);" << (n ? total_cpu_update / n : 0.0) << "\n";
            summary_out << "Avg CPU Cull (ms);"   << (n ? total_cpu_cull   / n : 0.0) << "\n";
            summary_out << "Avg CPU Draw (ms);"   << (n ? total_cpu_draw   / n : 0.0) << "\n";
            summary_out << "Avg GPU Time (ms);"   << (n ? total_gpu        / n : 0.0) << "\n";
            summary_out << "Avg Sync Time (ms);"  << (n ? total_sync       / n : 0.0) << "\n";
            summary_out << "Avg Swap Time (ms);"  << (n ? total_swap       / n : 0.0) << "\n";
            summary_out << "Avg Isect (ms);"      << (n ? total_isect      / n : 0.0) << "\n";
            summary_out << "Avg Plugin (ms);"     << (n ? total_plugin     / n : 0.0) << "\n";
            summary_out << "Avg OpenCOVER (ms);"  << (n ? total_opencover  / n : 0.0) << "\n";
            summary_out << "Avg Finish (ms);"     << (n ? total_finish     / n : 0.0) << "\n";
            summary_out << "Avg CPU Main (ms);"           << (n ? total_cpu_main / n : 0.0) << "\n";
            summary_out << "Avg CPU Busy Proxy (%);"      << (n ? total_cpu_busy_pct / n : 0.0) << "\n";
            summary_out << "Avg GPU Busy Proxy (%);"      << (n ? total_gpu_busy_pct / n : 0.0) << "\n";
            summary_out << "Avg Wait (ms);"               << (n ? total_wait_ms / n : 0.0) << "\n";
            summary_out << "Avg Wait Fraction Proxy (%);" << (n ? total_wait_frac_pct / n : 0.0) << "\n";
            summary_out << "FT p50 (ms);" << p50 << "\n";
            summary_out << "FT p95 (ms);" << p95 << "\n";
            summary_out << "FT p99 (ms);" << p99 << "\n";
            summary_out << "FT Jitter J = p99-p50 (ms);" << jitter_J << "\n";
            summary_out << "Stutter Count (>2x median);" << stutter_count_2xmedian << "\n";
            summary_out << "Stutter Count (>33.33 ms);" << stutter_count_33ms << "\n";
            summary_out << "Avg Rpts (points/ms);" << avg_rpts_points_per_ms << "\n";
            summary_out << "Avg Coverage C1;" << avg_covC1 << "\n";
            summary_out << "Avg Screen Density D;" << avg_covD << "\n";

            // --- Renderer Estimates Averages ---
            summary_out << "Avg est_screen_px;"   << avg_est_screen_px   << "\n";
            summary_out << "Avg est_sum_area_px;" << avg_est_sum_area_px << "\n";
            summary_out << "Avg est_density;"     << avg_est_density     << "\n";
            summary_out << "Avg est_coverage;"    << avg_est_coverage    << "\n";
            summary_out << "Avg est_coverage_px;" << avg_est_coverage_px << "\n";
            summary_out << "Avg est_overdraw;"    << avg_est_overdraw    << "\n";
            summary_out << "Avg avg_area_px_per_prim;" << avg_avg_area_px_per_prim << "\n";

            // --- Ø Zeitmarken ---
            summary_out << "Avg mark_draw_impl_ms;"    << avg_mark_draw_impl_ms    << "\n";
            summary_out << "Avg mark_pass1_ms;"        << avg_mark_pass1_ms        << "\n";
            summary_out << "Avg mark_pass2_ms;"        << avg_mark_pass2_ms        << "\n";
            summary_out << "Avg mark_pass3_ms;"        << avg_mark_pass3_ms        << "\n";
            summary_out << "Avg mark_dispatch_ms;"     << avg_mark_dispatch_ms     << "\n";
            summary_out << "Avg mark_context_bind_ms;" << avg_mark_context_bind_ms << "\n";
            summary_out << "Avg mark_estimates_ms;"    << avg_mark_estimates_ms    << "\n";
            summary_out << "Avg mark_singlepass_ms;" << avg_mark_singlepass_ms << "\n";

            // --- POLICY Budgets & Provenance ---
            if (auto* pol = lamure::ren::policy::get_instance()) {
                summary_out << "Policy max_upload_budget_in_mb;"  << pol->max_upload_budget_in_mb()  << "\n";
                summary_out << "Policy render_budget_in_mb;"      << pol->render_budget_in_mb()      << "\n";
                summary_out << "Policy out_of_core_budget_in_mb;" << pol->out_of_core_budget_in_mb() << "\n";
                summary_out << "Policy size_of_provenance;"       << pol->size_of_provenance()       << "\n";
                const bool provenance_enabled = (pol->size_of_provenance() > 0);
                summary_out << "Policy provenance_enabled;"        << (provenance_enabled ? 1 : 0)    << "\n";
            }

            // --- GPU Memory (static once) ---
            summary_out << "GPU Mem Used MB (primary);"  << m_gpu_mem_used_mb_static  << "\n";
            summary_out << "GPU Mem Total MB (primary);" << m_gpu_mem_total_mb_static << "\n";
            summary_out << "GPU Mem Used MB (NVML);"     << m_gpu_mem_used_mb_nvml_static  << "\n";
            summary_out << "GPU Mem Total MB (NVML);"    << m_gpu_mem_total_mb_nvml_static << "\n";
            summary_out << "GPU Mem Used MB (GL_NVX);"   << m_gpu_mem_used_mb_gl_static    << "\n";
            summary_out << "GPU Mem Total MB (GL_NVX);"  << m_gpu_mem_total_mb_gl_static   << "\n";

            // --- Compile-time Config als Key;Value ---
            writeLamureConfigCsv(summary_out);

            summary_out.flush();
        }
    }

    // --- Timeline CSV (bestehend) ---
    if (m_exportTimeline) {
        writeTimelineCSV(timeline_path.string());
    }

    // --- Markdown-Report (inkl. Zeitmarken) ---
    {
        std::ofstream md;
        if (!openCsv(md, md_path)) {
            std::cerr << "[Measurement] Konnte Markdown-Report nicht schreiben: "
                << md_path.string() << "\n";
        } else {
            md << "# Measurement Report\n\n";
            md << "- Frames: " << m_stats.size() << "\n";
            size_t n = m_stats.size();
            double sum_ft=0, sum_cpu_main=0, sum_gpu=0, sum_wait=0, sum_cpu_bp=0, sum_gpu_bp=0, sum_wait_pct=0;
            size_t cnt_gpu=0, cnt_cpu=0, cnt_wait=0, cnt_mixed=0, cnt_unknown=0;
            for (const auto& s : m_stats) {
                sum_ft       += s.frame_duration_ms;
                sum_cpu_main += s.cpu_main_ms;
                auto it = gpuMsByFrame.find(s.frame_number);
                sum_gpu += (it != gpuMsByFrame.end()) ? it->second : 0.0;
                sum_wait     += s.wait_ms;
                sum_cpu_bp   += s.cpu_busy_pct_proxy;
                sum_gpu_bp   += s.gpu_busy_pct_proxy;
                sum_wait_pct += s.wait_frac_pct;
                if      (s.boundness == "GPU-bound")       ++cnt_gpu;
                else if (s.boundness == "CPU-bound")       ++cnt_cpu;
                else if (s.boundness == "Wait/Sync-bound") ++cnt_wait;
                else if (s.boundness == "mixed")           ++cnt_mixed;
                else                                       ++cnt_unknown;
            }
            md << "- Avg Frame Time (ms): " << (n? sum_ft/n : 0.0) << "\n";
            md << "- Avg CPU Main (ms): "   << (n? sum_cpu_main/n : 0.0) << "\n";
            md << "- Avg GPU Time (ms): "   << (n? sum_gpu/n : 0.0) << "\n";
            md << "- Avg Wait (ms): "       << (n? sum_wait/n : 0.0) << "\n";
            md << "- Avg CPU Busy (%): "    << (n? sum_cpu_bp/n : 0.0) << "\n";
            md << "- Avg GPU Busy (%): "    << (n? sum_gpu_bp/n : 0.0) << "\n";
            md << "- Avg Wait Fraction (%): "<< (n? sum_wait_pct/n : 0.0) << "\n\n";
            md << "## Boundness Verteilung\n";
            md << "- GPU-bound: "       << cnt_gpu << "\n";
            md << "- CPU-bound: "       << cnt_cpu << "\n";
            md << "- Wait/Sync-bound: " << cnt_wait << "\n";
            md << "- mixed: "           << cnt_mixed << "\n";
            md << "- unknown: "         << cnt_unknown << "\n";
            md << "\n## Frametime-Perzentile & Jitter\n";
            md << "- p50 (ms): " << p50 << "\n";
            md << "- p95 (ms): " << p95 << "\n";
            md << "- p99 (ms): " << p99 << "\n";
            md << "- Jitter J (p99 - p50) (ms): " << jitter_J << "\n";
            md << "- Stutter >2×Median (Count): " << stutter_count_2xmedian << "\n";
            md << "- Stutter >33.33 ms (Count): " << stutter_count_33ms << "\n";

            md << "\n## Durchsatz/Effizienz\n";
            md << "- Avg Rpts (points/ms): " << avg_rpts_points_per_ms << "\n";
            md << "- Avg avg_area_px_per_prim: " << avg_avg_area_px_per_prim << "\n";

            md << "\n## Coverage (aus Estimates)\n";
            md << "- Avg Coverage C1: " << avg_covC1 << "\n";
            md << "- Avg Screen Density D: " << avg_covD << "\n";

            md << "\n## Renderer Estimates (avg)\n";
            md << "- est_screen_px: "   << avg_est_screen_px   << "\n";
            md << "- est_sum_area_px: " << avg_est_sum_area_px << "\n";
            md << "- est_density: "     << avg_est_density     << "\n";
            md << "- est_coverage: "    << avg_est_coverage    << "\n";
            md << "- est_coverage_px: " << avg_est_coverage_px << "\n";
            md << "- est_overdraw: "    << avg_est_overdraw    << "\n";

            md << "\n## Renderer-Zeitmarken (Durchschnitt pro Frame)\n";
            md << "- draw_impl_ms: "    << avg_mark_draw_impl_ms    << "\n";
            md << "- pass1_ms: "        << avg_mark_pass1_ms        << "\n";
            md << "- pass2_ms: "        << avg_mark_pass2_ms        << "\n";
            md << "- pass3_ms: "        << avg_mark_pass3_ms        << "\n";
            md << "- dispatch_ms: "     << avg_mark_dispatch_ms     << "\n";
            md << "- context_bind_ms: " << avg_mark_context_bind_ms << "\n";
            md << "- estimates_ms: "    << avg_mark_estimates_ms    << "\n";
            md << "- singlepass_ms: "   << avg_mark_singlepass_ms   << "\n";

            // --- Policy Budgets ---
            if (auto* pol = lamure::ren::policy::get_instance()) {
                md << "\n## Policy Budgets\n";
                md << "- max_upload_budget_in_mb: "  << pol->max_upload_budget_in_mb()  << "\n";
                md << "- render_budget_in_mb: "      << pol->render_budget_in_mb()      << "\n";
                md << "- out_of_core_budget_in_mb: " << pol->out_of_core_budget_in_mb() << "\n";
                md << "- size_of_provenance: "       << pol->size_of_provenance()       << "\n";
                const bool provenance_enabled = (pol->size_of_provenance() > 0);
                md << "- provenance_enabled: "       << (provenance_enabled ? "true" : "false") << "\n";
            }

            md << "\n## GPU Memory (static)\n";
            md << "- Primary used/total (MB): "
                << m_gpu_mem_used_mb_static << " / " << m_gpu_mem_total_mb_static << "\n";
            md << "- NVML used/total (MB):   "
                << m_gpu_mem_used_mb_nvml_static << " / " << m_gpu_mem_total_mb_nvml_static << "\n";
            md << "- GL_NVX used/total (MB): "
                << m_gpu_mem_used_mb_gl_static << " / " << m_gpu_mem_total_mb_gl_static << "\n";

            writeLamureConfigMarkdown(md);
            md.flush();
        }

        const std::filesystem::path settings_path = base_path.string() + "_settings.json";
        m_plugin->writeSettingsJson(m_plugin->getSettings(), settings_path.string());
    }

    // Cleanup
    m_stats.clear();
    m_timeline.clear();
    m_segmentTime = 0.0;
    m_currentSegment = 0;
    m_cumulativeRot = osg::Vec3(0.0f, 0.0f, 0.0f);
}


void LamureMeasurement::writeLamureConfigMarkdown(std::ostream& md) {
    md << "\n## Lamure Config (compile-time)\n";

#ifdef LAMURE_ENABLE_INFO
    md << "- LAMURE_ENABLE_INFO: ON\n";
#else
    md << "- LAMURE_ENABLE_INFO: OFF\n";
#endif

#ifdef LAMURE_RENDERING_USE_SPLIT_SCREEN
    md << "- LAMURE_RENDERING_USE_SPLIT_SCREEN: ON\n";
#else
    md << "- LAMURE_RENDERING_USE_SPLIT_SCREEN: OFF\n";
#endif

#ifdef LAMURE_RENDERING_ENABLE_MULTI_VIEW_TEST
    md << "- LAMURE_RENDERING_ENABLE_MULTI_VIEW_TEST: ON\n";
#else
    md << "- LAMURE_RENDERING_ENABLE_MULTI_VIEW_TEST: OFF\n";
#endif

#ifdef LAMURE_RENDERING_ENABLE_LAZY_MODELS_TEST
    md << "- LAMURE_RENDERING_ENABLE_LAZY_MODELS_TEST: ON\n";
#else
    md << "- LAMURE_RENDERING_ENABLE_LAZY_MODELS_TEST: OFF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_MEASURE_SYSTEM_PERFORMANCE
    md << "- LAMURE_CUT_UPDATE_ENABLE_MEASURE_SYSTEM_PERFORMANCE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_MEASURE_SYSTEM_PERFORMANCE: OFF\n";
#endif

#ifdef LAMURE_DEFAULT_COLOR_R
    md << "- LAMURE_DEFAULT_COLOR_R: " << LAMURE_DEFAULT_COLOR_R << "\n";
#else
    md << "- LAMURE_DEFAULT_COLOR_R: UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_COLOR_G
    md << "- LAMURE_DEFAULT_COLOR_G: " << LAMURE_DEFAULT_COLOR_G << "\n";
#else
    md << "- LAMURE_DEFAULT_COLOR_G: UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_COLOR_B
    md << "- LAMURE_DEFAULT_COLOR_B: " << LAMURE_DEFAULT_COLOR_B << "\n";
#else
    md << "- LAMURE_DEFAULT_COLOR_B: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_MODEL_TIMEOUT
    md << "- LAMURE_CUT_UPDATE_ENABLE_MODEL_TIMEOUT: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_MODEL_TIMEOUT: OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT
    md << "- LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT: " << LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_CUT_UPDATE_EXPERIMENTAL_MODE
    md << "- LAMURE_CUT_UPDATE_ENABLE_CUT_UPDATE_EXPERIMENTAL_MODE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_CUT_UPDATE_EXPERIMENTAL_MODE: OFF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS
    md << "- LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS: " << LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_SHOW_OOC_CACHE_USAGE
    md << "- LAMURE_CUT_UPDATE_ENABLE_SHOW_OOC_CACHE_USAGE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_SHOW_OOC_CACHE_USAGE: OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_ENABLE_SHOW_GPU_CACHE_USAGE
    md << "- LAMURE_CUT_UPDATE_ENABLE_SHOW_GPU_CACHE_USAGE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_SHOW_GPU_CACHE_USAGE: OFF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_REPEAT_MODE
    md << "- LAMURE_CUT_UPDATE_ENABLE_REPEAT_MODE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_REPEAT_MODE: OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME
    md << "- LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME: " << LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_SPLIT_AGAIN_MODE
    md << "- LAMURE_CUT_UPDATE_ENABLE_SPLIT_AGAIN_MODE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_SPLIT_AGAIN_MODE: OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_MUST_COLLAPSE_OUTSIDE_FRUSTUM
    md << "- LAMURE_CUT_UPDATE_MUST_COLLAPSE_OUTSIDE_FRUSTUM: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_MUST_COLLAPSE_OUTSIDE_FRUSTUM: OFF\n";
#endif

#ifdef LAMURE_DATABASE_SAFE_MODE
    md << "- LAMURE_DATABASE_SAFE_MODE: ON\n";
#else
    md << "- LAMURE_DATABASE_SAFE_MODE: OFF\n";
#endif

#ifdef LAMURE_DEFAULT_IMPORTANCE
    md << "- LAMURE_DEFAULT_IMPORTANCE: " << LAMURE_DEFAULT_IMPORTANCE << "\n";
#else
    md << "- LAMURE_DEFAULT_IMPORTANCE: UNDEF\n";
#endif
#ifdef LAMURE_MIN_IMPORTANCE
    md << "- LAMURE_MIN_IMPORTANCE: " << LAMURE_MIN_IMPORTANCE << "\n";
#else
    md << "- LAMURE_MIN_IMPORTANCE: UNDEF\n";
#endif
#ifdef LAMURE_MAX_IMPORTANCE
    md << "- LAMURE_MAX_IMPORTANCE: " << LAMURE_MAX_IMPORTANCE << "\n";
#else
    md << "- LAMURE_MAX_IMPORTANCE: UNDEF\n";
#endif

#ifdef LAMURE_DEFAULT_THRESHOLD
    md << "- LAMURE_DEFAULT_THRESHOLD: " << LAMURE_DEFAULT_THRESHOLD << "\n";
#else
    md << "- LAMURE_DEFAULT_THRESHOLD: UNDEF\n";
#endif
#ifdef LAMURE_MIN_THRESHOLD
    md << "- LAMURE_MIN_THRESHOLD: " << LAMURE_MIN_THRESHOLD << "\n";
#else
    md << "- LAMURE_MIN_THRESHOLD: UNDEF\n";
#endif
#ifdef LAMURE_MAX_THRESHOLD
    md << "- LAMURE_MAX_THRESHOLD: " << LAMURE_MAX_THRESHOLD << "\n";
#else
    md << "- LAMURE_MAX_THRESHOLD: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_PREFETCHING
    md << "- LAMURE_CUT_UPDATE_ENABLE_PREFETCHING: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_PREFETCHING: OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_PREFETCH_FACTOR
    md << "- LAMURE_CUT_UPDATE_PREFETCH_FACTOR: " << LAMURE_CUT_UPDATE_PREFETCH_FACTOR << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_PREFETCH_FACTOR: UNDEF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_PREFETCH_BUDGET
    md << "- LAMURE_CUT_UPDATE_PREFETCH_BUDGET: " << LAMURE_CUT_UPDATE_PREFETCH_BUDGET << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_PREFETCH_BUDGET: UNDEF\n";
#endif

#ifdef LAMURE_MIN_UPLOAD_BUDGET
    md << "- LAMURE_MIN_UPLOAD_BUDGET: " << LAMURE_MIN_UPLOAD_BUDGET << "\n";
#else
    md << "- LAMURE_MIN_UPLOAD_BUDGET: UNDEF\n";
#endif
#ifdef LAMURE_MIN_VIDEO_MEMORY_BUDGET
    md << "- LAMURE_MIN_VIDEO_MEMORY_BUDGET: " << LAMURE_MIN_VIDEO_MEMORY_BUDGET << "\n";
#else
    md << "- LAMURE_MIN_VIDEO_MEMORY_BUDGET: UNDEF\n";
#endif
#ifdef LAMURE_MIN_MAIN_MEMORY_BUDGET
    md << "- LAMURE_MIN_MAIN_MEMORY_BUDGET: " << LAMURE_MIN_MAIN_MEMORY_BUDGET << "\n";
#else
    md << "- LAMURE_MIN_MAIN_MEMORY_BUDGET: UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_UPLOAD_BUDGET
    md << "- LAMURE_DEFAULT_UPLOAD_BUDGET: " << LAMURE_DEFAULT_UPLOAD_BUDGET << "\n";
#else
    md << "- LAMURE_DEFAULT_UPLOAD_BUDGET: UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET
    md << "- LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET: " << LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET << "\n";
#else
    md << "- LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET: UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_MAIN_MEMORY_BUDGET
    md << "- LAMURE_DEFAULT_MAIN_MEMORY_BUDGET: " << LAMURE_DEFAULT_MAIN_MEMORY_BUDGET << "\n";
#else
    md << "- LAMURE_DEFAULT_MAIN_MEMORY_BUDGET: UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_SIZE_OF_PROVENANCE
    md << "- LAMURE_DEFAULT_SIZE_OF_PROVENANCE: " << LAMURE_DEFAULT_SIZE_OF_PROVENANCE << "\n";
#else
    md << "- LAMURE_DEFAULT_SIZE_OF_PROVENANCE: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_NUM_LOADING_THREADS
    md << "- LAMURE_CUT_UPDATE_NUM_LOADING_THREADS: " << LAMURE_CUT_UPDATE_NUM_LOADING_THREADS << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_NUM_LOADING_THREADS: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_CACHE_MAINTENANCE
    md << "- LAMURE_CUT_UPDATE_ENABLE_CACHE_MAINTENANCE: ON\n";
#else
    md << "- LAMURE_CUT_UPDATE_ENABLE_CACHE_MAINTENANCE: OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER
    md << "- LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER: " << LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER: UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE
    md << "- LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE: " << LM_STR(LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE) << "\n";
#else
    md << "- LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE: UNDEF\n";
#endif

#ifdef LAMURE_WYSIWYG_SPLAT_SCALE
    md << "- LAMURE_WYSIWYG_SPLAT_SCALE: " << LAMURE_WYSIWYG_SPLAT_SCALE << "\n";
#else
    md << "- LAMURE_WYSIWYG_SPLAT_SCALE: UNDEF\n";
#endif

}

void LamureMeasurement::writeLamureConfigCsv(std::ostream& csv)
{
#ifdef LAMURE_ENABLE_INFO
    csv << "LAMURE_ENABLE_INFO;ON\n";
#else
    csv << "LAMURE_ENABLE_INFO;OFF\n";
#endif

#ifdef LAMURE_RENDERING_USE_SPLIT_SCREEN
    csv << "LAMURE_RENDERING_USE_SPLIT_SCREEN;ON\n";
#else
    csv << "LAMURE_RENDERING_USE_SPLIT_SCREEN;OFF\n";
#endif

#ifdef LAMURE_RENDERING_ENABLE_MULTI_VIEW_TEST
    csv << "LAMURE_RENDERING_ENABLE_MULTI_VIEW_TEST;ON\n";
#else
    csv << "LAMURE_RENDERING_ENABLE_MULTI_VIEW_TEST;OFF\n";
#endif

#ifdef LAMURE_RENDERING_ENABLE_LAZY_MODELS_TEST
    csv << "LAMURE_RENDERING_ENABLE_LAZY_MODELS_TEST;ON\n";
#else
    csv << "LAMURE_RENDERING_ENABLE_LAZY_MODELS_TEST;OFF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_MEASURE_SYSTEM_PERFORMANCE
    csv << "LAMURE_CUT_UPDATE_ENABLE_MEASURE_SYSTEM_PERFORMANCE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_MEASURE_SYSTEM_PERFORMANCE;OFF\n";
#endif

#ifdef LAMURE_DEFAULT_COLOR_R
    csv << "LAMURE_DEFAULT_COLOR_R;" << LAMURE_DEFAULT_COLOR_R << "\n";
#else
    csv << "LAMURE_DEFAULT_COLOR_R;UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_COLOR_G
    csv << "LAMURE_DEFAULT_COLOR_G;" << LAMURE_DEFAULT_COLOR_G << "\n";
#else
    csv << "LAMURE_DEFAULT_COLOR_G;UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_COLOR_B
    csv << "LAMURE_DEFAULT_COLOR_B;" << LAMURE_DEFAULT_COLOR_B << "\n";
#else
    csv << "LAMURE_DEFAULT_COLOR_B;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_MODEL_TIMEOUT
    csv << "LAMURE_CUT_UPDATE_ENABLE_MODEL_TIMEOUT;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_MODEL_TIMEOUT;OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT
    csv << "LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT;" << LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_MAX_MODEL_TIMEOUT;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_CUT_UPDATE_EXPERIMENTAL_MODE
    csv << "LAMURE_CUT_UPDATE_ENABLE_CUT_UPDATE_EXPERIMENTAL_MODE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_CUT_UPDATE_EXPERIMENTAL_MODE;OFF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS
    csv << "LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS;" << LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_NUM_CUT_UPDATE_THREADS;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_SHOW_OOC_CACHE_USAGE
    csv << "LAMURE_CUT_UPDATE_ENABLE_SHOW_OOC_CACHE_USAGE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_SHOW_OOC_CACHE_USAGE;OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_ENABLE_SHOW_GPU_CACHE_USAGE
    csv << "LAMURE_CUT_UPDATE_ENABLE_SHOW_GPU_CACHE_USAGE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_SHOW_GPU_CACHE_USAGE;OFF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_REPEAT_MODE
    csv << "LAMURE_CUT_UPDATE_ENABLE_REPEAT_MODE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_REPEAT_MODE;OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME
    csv << "LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME;" << LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_MAX_NUM_UPDATES_PER_FRAME;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_SPLIT_AGAIN_MODE
    csv << "LAMURE_CUT_UPDATE_ENABLE_SPLIT_AGAIN_MODE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_SPLIT_AGAIN_MODE;OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_MUST_COLLAPSE_OUTSIDE_FRUSTUM
    csv << "LAMURE_CUT_UPDATE_MUST_COLLAPSE_OUTSIDE_FRUSTUM;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_MUST_COLLAPSE_OUTSIDE_FRUSTUM;OFF\n";
#endif

#ifdef LAMURE_DATABASE_SAFE_MODE
    csv << "LAMURE_DATABASE_SAFE_MODE;ON\n";
#else
    csv << "LAMURE_DATABASE_SAFE_MODE;OFF\n";
#endif

#ifdef LAMURE_DEFAULT_IMPORTANCE
    csv << "LAMURE_DEFAULT_IMPORTANCE;" << LAMURE_DEFAULT_IMPORTANCE << "\n";
#else
    csv << "LAMURE_DEFAULT_IMPORTANCE;UNDEF\n";
#endif
#ifdef LAMURE_MIN_IMPORTANCE
    csv << "LAMURE_MIN_IMPORTANCE;" << LAMURE_MIN_IMPORTANCE << "\n";
#else
    csv << "LAMURE_MIN_IMPORTANCE;UNDEF\n";
#endif
#ifdef LAMURE_MAX_IMPORTANCE
    csv << "LAMURE_MAX_IMPORTANCE;" << LAMURE_MAX_IMPORTANCE << "\n";
#else
    csv << "LAMURE_MAX_IMPORTANCE;UNDEF\n";
#endif

#ifdef LAMURE_DEFAULT_THRESHOLD
    csv << "LAMURE_DEFAULT_THRESHOLD;" << LAMURE_DEFAULT_THRESHOLD << "\n";
#else
    csv << "LAMURE_DEFAULT_THRESHOLD;UNDEF\n";
#endif
#ifdef LAMURE_MIN_THRESHOLD
    csv << "LAMURE_MIN_THRESHOLD;" << LAMURE_MIN_THRESHOLD << "\n";
#else
    csv << "LAMURE_MIN_THRESHOLD;UNDEF\n";
#endif
#ifdef LAMURE_MAX_THRESHOLD
    csv << "LAMURE_MAX_THRESHOLD;" << LAMURE_MAX_THRESHOLD << "\n";
#else
    csv << "LAMURE_MAX_THRESHOLD;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_PREFETCHING
    csv << "LAMURE_CUT_UPDATE_ENABLE_PREFETCHING;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_PREFETCHING;OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_PREFETCH_FACTOR
    csv << "LAMURE_CUT_UPDATE_PREFETCH_FACTOR;" << LAMURE_CUT_UPDATE_PREFETCH_FACTOR << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_PREFETCH_FACTOR;UNDEF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_PREFETCH_BUDGET
    csv << "LAMURE_CUT_UPDATE_PREFETCH_BUDGET;" << LAMURE_CUT_UPDATE_PREFETCH_BUDGET << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_PREFETCH_BUDGET;UNDEF\n";
#endif

#ifdef LAMURE_MIN_UPLOAD_BUDGET
    csv << "LAMURE_MIN_UPLOAD_BUDGET;" << LAMURE_MIN_UPLOAD_BUDGET << "\n";
#else
    csv << "LAMURE_MIN_UPLOAD_BUDGET;UNDEF\n";
#endif
#ifdef LAMURE_MIN_VIDEO_MEMORY_BUDGET
    csv << "LAMURE_MIN_VIDEO_MEMORY_BUDGET;" << LAMURE_MIN_VIDEO_MEMORY_BUDGET << "\n";
#else
    csv << "LAMURE_MIN_VIDEO_MEMORY_BUDGET;UNDEF\n";
#endif
#ifdef LAMURE_MIN_MAIN_MEMORY_BUDGET
    csv << "LAMURE_MIN_MAIN_MEMORY_BUDGET;" << LAMURE_MIN_MAIN_MEMORY_BUDGET << "\n";
#else
    csv << "LAMURE_MIN_MAIN_MEMORY_BUDGET;UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_UPLOAD_BUDGET
    csv << "LAMURE_DEFAULT_UPLOAD_BUDGET;" << LAMURE_DEFAULT_UPLOAD_BUDGET << "\n";
#else
    csv << "LAMURE_DEFAULT_UPLOAD_BUDGET;UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET
    csv << "LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET;" << LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET << "\n";
#else
    csv << "LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET;UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_MAIN_MEMORY_BUDGET
    csv << "LAMURE_DEFAULT_MAIN_MEMORY_BUDGET;" << LAMURE_DEFAULT_MAIN_MEMORY_BUDGET << "\n";
#else
    csv << "LAMURE_DEFAULT_MAIN_MEMORY_BUDGET;UNDEF\n";
#endif
#ifdef LAMURE_DEFAULT_SIZE_OF_PROVENANCE
    csv << "LAMURE_DEFAULT_SIZE_OF_PROVENANCE;" << LAMURE_DEFAULT_SIZE_OF_PROVENANCE << "\n";
#else
    csv << "LAMURE_DEFAULT_SIZE_OF_PROVENANCE;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_NUM_LOADING_THREADS
    csv << "LAMURE_CUT_UPDATE_NUM_LOADING_THREADS;" << LAMURE_CUT_UPDATE_NUM_LOADING_THREADS << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_NUM_LOADING_THREADS;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_ENABLE_CACHE_MAINTENANCE
    csv << "LAMURE_CUT_UPDATE_ENABLE_CACHE_MAINTENANCE;ON\n";
#else
    csv << "LAMURE_CUT_UPDATE_ENABLE_CACHE_MAINTENANCE;OFF\n";
#endif
#ifdef LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER
    csv << "LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER;" << LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_CACHE_MAINTENANCE_COUNTER;UNDEF\n";
#endif

#ifdef LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE
    csv << "LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE;" << LM_STR(LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE) << "\n";
#else
    csv << "LAMURE_CUT_UPDATE_LOADING_QUEUE_MODE;UNDEF\n";
#endif

#ifdef LAMURE_WYSIWYG_SPLAT_SCALE
    csv << "LAMURE_WYSIWYG_SPLAT_SCALE;" << LAMURE_WYSIWYG_SPLAT_SCALE << "\n";
#else
    csv << "LAMURE_WYSIWYG_SPLAT_SCALE;UNDEF\n";
#endif
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
