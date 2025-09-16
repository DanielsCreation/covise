#pragma once

// Include GLEW first
#include <GL/glew.h>

// Then other headers
#include <cover/ui/ButtonGroup.h>
#include <cover/ui/Button.h>
#include <cover/coVRTui.h>
#include <cover/VRViewer.h>
#include <osg/Timer>
#include <osg/Camera>
#include <osgViewer/Viewer>
#include <vector>
#include <utility>
#include <string>
#include <cover/VRSceneGraph.h>
#include <cover/coVRCollaboration.h>
#include <cover/coVRPluginSupport.h>

class Lamure; // Forward declaration

class LamureMeasurement
{
public:

    struct TimeBlock {
        unsigned frame     = 0;   // Basisframe (aus pickStableFrame)
        unsigned src_frame = 0;   // Effektives Stats-Frame (frame - used_offset)
        int      camIndex  = -1;  // -1 = viewer, sonst Kameraindex
        std::string scope;        // "viewer" | "camera"
        std::string name;         // z.B. "Draw traversal", "GPU draw"
        double begin_ms = 0.0;    // ms (falls vorhanden)
        double end_ms   = 0.0;    // ms (falls vorhanden)
        double taken_ms = 0.0;    // ms (immer, wenn time taken verfügbar)
        unsigned used_offset = 0; // wie viele Frames zurückgegriffen
    };

    struct Segment
    {
        osg::Vec3 tra;     // Positionsänderung relativ zum Segment-Anfang
        osg::Vec3 rot;     // Drehwinkel-Delta in Grad (Pitch, Yaw, Roll)
        float     transSpeed;   // Translationstempo in Einheiten/s
        float     rotSpeed;     // Rotationsgeschwindigkeit in °/s
    };

    struct FrameStats {
        unsigned int frame_number   = 0;
        double frame_rate           = 0.0;
        double frame_duration_ms    = 0.0;
        double rendering_traversals_ms = 0.0;

        uint64_t rendered_splats    = 0;
        uint64_t rendered_nodes     = 0;
        uint64_t rendered_bounding_boxes = 0;

        double cpu_update_ms       = 0.0;
        double cpu_cull_ms         = 0.0;
        double cpu_draw_ms         = 0.0;

        double gpu_time_ms         = 0.0;
        double sync_time_ms        = 0.0;
        double swap_time_ms        = 0.0;
        double finish_ms           = 0.0;

        double gpu_clock           = 0.0;
        double gpu_mem_clock       = 0.0;
        double gpu_util            = 0.0;
        double gpu_pci             = 0.0;

        double plugin_ms           = 0.0;
        double isect_ms            = 0.0;
        double opencover_ms        = 0.0;

        osg::Vec3d position;
        osg::Quat  orientation;

        unsigned backoff_cull = 0;
        unsigned backoff_draw = 0;
        unsigned backoff_gpu  = 0;

        // optional (NVML, Windows/NVIDIA), sonst 0
        double gpu_mem_used_mb   = 0.0;
        double gpu_mem_total_mb  = 0.0;

        double gpu_mem_used_mb_nvml  = 0.0;
        double gpu_mem_total_mb_nvml = 0.0;
        double gpu_mem_used_mb_gl    = 0.0;
        double gpu_mem_total_mb_gl   = 0.0;

        // --- Derived metrics (pro Frame) ---
        double cpu_main_ms         = 0.0;  // cpu_update + cpu_cull + cpu_draw + plugin + isect + opencover
        double cpu_busy_pct_proxy  = 0.0;  // 100 * cpu_main_ms / frame_duration_ms (0..100 geclippt)
        double gpu_busy_pct_proxy  = 0.0;  // 100 * min(gpu_time_ms, frame_duration_ms)/frame_duration_ms
        double wait_ms             = 0.0;  // sync + swap + finish
        double wait_frac_pct       = 0.0;  // 100 * wait_ms / frame_duration_ms
        std::string boundness;             // "GPU-bound" | "CPU-bound" | "Wait/Sync-bound" | "mixed" | "unknown"
    };

    bool collectFrameStats(osgViewer::ViewerBase* viewer,
        const osg::FrameStamp* fs,
        FrameStats& out,
        bool debugPrint = false);

    LamureMeasurement(
        Lamure*                      plugin,
        opencover::VRViewer*         viewer,
        const std::vector<Segment>&  segments,
        const std::string&           logfile
    );
    ~LamureMeasurement();

    void stop();
    void writeLogAndStop();
    bool isRunning() const { return m_running; }

    // Timeline-Zugriff (nur lesen)
    const std::vector<TimeBlock>& getTimeline() const { return m_timeline; }

    // Optionaler Markdown-Report
    void setReportMarkdown(const std::string& path);
    void writeLamureConfigMarkdown(std::ostream& md);

    // Bequeme Getter für TimeBlock
    unsigned            getTimeBlockFrame(const TimeBlock& tb) const { return tb.frame; }
    unsigned            getTimeBlockSrcFrame(const TimeBlock& tb) const { return tb.src_frame; }
    int                 getTimeBlockCamIndex(const TimeBlock& tb) const { return tb.camIndex; }
    const std::string&  getTimeBlockScope(const TimeBlock& tb) const { return tb.scope; }
    const std::string&  getTimeBlockName(const TimeBlock& tb) const { return tb.name; }
    double              getTimeBlockBeginMs(const TimeBlock& tb) const { return tb.begin_ms; }
    double              getTimeBlockEndMs(const TimeBlock& tb) const { return tb.end_ms; }
    double              getTimeBlockTakenMs(const TimeBlock& tb) const { return tb.taken_ms; }
    unsigned            getTimeBlockUsedOffset(const TimeBlock& tb) const { return tb.used_offset; }

private:
    // Konsolen-/Export-Optionen
    bool     m_exportReport   = true;
    bool     m_verbose        = false;   // Konsole drosseln
    unsigned m_logEveryN      = 50;      // nur jedes N-te Frame loggen
    bool     m_dumpAttrs      = false;   // Attribute-Dumps aus
    double   m_warnTolMs      = 5.0;     // Debug-Warnschwelle
    unsigned m_gpuBackSearch  = 16;      // größerer Backsearch (GPU/Timing)
    bool     m_exportTimeline = true;    // Timeline-CSV schreiben
    bool   m_gpu_static_captured = false;
    bool m_gpu_static_tried = false;
    double m_gpu_mem_used_mb_static        = 0.0;
    double m_gpu_mem_total_mb_static       = 0.0;
    double m_gpu_mem_used_mb_nvml_static   = 0.0;
    double m_gpu_mem_total_mb_nvml_static  = 0.0;
    double m_gpu_mem_used_mb_gl_static     = 0.0;
    double m_gpu_mem_total_mb_gl_static    = 0.0;

    osg::Vec3 m_lastTraApplied{0,0,0};
    osg::Vec3 m_lastRotApplied{0,0,0};
    bool      m_havePoseDeltas = false;

    std::string m_reportMDPath; // wenn gesetzt, schreibe Markdown-Report am Ende

    std::vector<TimeBlock> m_timeline;

    void writeLamureConfigCsv(std::ostream& csv);
    void cacheStaticGpuInfo();

    bool     m_written{false};
    bool     m_haveLastQuat{false};
    osg::Quat m_lastQuat;

    // interne Helfer
    bool getAttributeForFrame(osg::Stats* stats,
        const std::string& attributeName,
        double& value,
        unsigned int frameNumber);
    bool getAttributeForFrame(osg::Stats* stats,
        const std::string& attributeName,
        double& value);

    // Backsearch-Helfer: bevorzugt "time taken", fällt auf (end-begin) zurück
    bool getTimeTakenMsBacksearch(osg::Stats* s,
        unsigned baseFrame,
        unsigned backSearch,
        const std::string& timeTakenKey,
        const std::string& beginKey,
        const std::string& endKey,
        double& outMs,
        unsigned& usedOffset,
        double* outBeginMs = nullptr,
        double* outEndMs   = nullptr);

    // Timeline-Eintrag erzeugen (TimeBlock), trägt auch in m_timeline ein (wenn aktiv)
    bool tryAddBlock(osg::Stats* stats,
        unsigned int baseFrame,
        unsigned int backsearch,
        const std::string& statPrefix,
        const std::string& nameForCSV,
        const std::string& scope,
        int camIndex,
        std::vector<TimeBlock>& localBlocks);

    void initCallbacks();

    void drawIncrement(bool preDraw, const osg::FrameStamp* frameStamp);

    void updateCamera(const osg::Vec3& pos, const osg::Vec3& rot);

    // Timeline-Export
    void writeTimelineCSV(const std::string& path);

    void printDebugStats(unsigned int num);

    int m_originalStatsType = 0;

    Lamure*                           m_plugin;
    opencover::VRViewer*              m_viewer;
    const std::vector<Segment>        m_segments;
    size_t                            m_currentSegment{0};
    double                            m_segmentTime{0.0};
    osg::Vec3                         m_cumulativeTra{0.0f,0.0f,0.0f};  // akkumulierte Deltas
    osg::Vec3                         m_cumulativeRot{0.0f,0.0f,0.0f};
    osg::Vec3                         m_initialPos;                     // Start-Position der Kamera
    osg::Matrix                       m_initialXform;

    std::string                       m_logfile;
    bool                              m_running{false};
    osg::Timer_t                      m_startTick;
    osg::Timer_t                      m_lastFrameTick;
    std::vector<FrameStats>           m_stats;

    struct MarkCallback : public osg::Camera::DrawCallback {
        LamureMeasurement* m_meas;
        bool         m_pre;
        MarkCallback(LamureMeasurement* m, bool pre) : m_meas(m), m_pre(pre) {}
        virtual void operator()(osg::RenderInfo& ri) const override {
            m_meas->drawIncrement(m_pre, ri.getState()->getFrameStamp());
        }
    };
    MarkCallback* m_preCB = nullptr;
    MarkCallback* m_postCB = nullptr;
};
