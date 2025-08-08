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
        unsigned frame = 0;       // Basisframe (pickStableFrame)
        unsigned src_frame = 0;   // effektives Stats-Frame (frame - used_offset)
        int camIndex = -1;        // -1 = viewer, sonst Kameraindex
        std::string scope;        // "viewer" | "camera"
        std::string name;         // z.B. "Draw traversal", "GPU draw"
        double begin_ms = 0.0;    // ms
        double end_ms   = 0.0;    // ms
        double taken_ms = 0.0;    // ms
        unsigned used_offset = 0; // wie viele Frames zurückgegriffen
    };

    struct Segment
    {
        osg::Vec3 tra;     // **neu:** Positionsänderung relativ zum Segment-Anfang
        osg::Vec3 rot;     // Drehwinkel-Delta in Grad (Pitch, Yaw, Roll)
        float     transSpeed;   // Translationstempo in Einheiten/s
        float     rotSpeed;     // Rotationsgeschwindigkeit in °/s 
    };

    struct FrameStats {
        unsigned int frame_number   = 0;
        double frame_rate           = 0.0;
        double frame_duration_ms    = 0.0;
        double rendering_traversals_ms = 0.0;   // NEU

        uint64_t rendered_splats    = 0;
        uint64_t rendered_nodes     = 0;
        uint64_t rendered_bounding_boxes = 0;

        double cpu_update_ms       = 0.0;
        double cpu_cull_ms         = 0.0;
        double cpu_draw_ms         = 0.0;

        double gpu_time_ms         = 0.0;
        double sync_time_ms        = 0.0;
        double swap_time_ms        = 0.0;
        double finish_ms           = 0.0;       // NEU

        double gpu_clock           = 0.0;
        double gpu_mem_clock       = 0.0;
        double gpu_util            = 0.0;
        double gpu_pci             = 0.0;

        double plugin_ms           = 0.0;
        double isect_ms            = 0.0;
        double opencover_ms        = 0.0;

        osg::Vec3d position;
        osg::Quat  orientation;

        unsigned backoff_cull = 0;              // NEU
        unsigned backoff_draw = 0;              // NEU
        unsigned backoff_gpu  = 0;              // NEU


        double gpu_mem_used_mb   = 0.0;
        double gpu_mem_total_mb  = 0.0;
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

    const std::vector<TimeBlock>& getTimeline() const { return m_timeline; }

    unsigned getTimeBlockFrame(const TimeBlock& tb) const { return tb.frame; }
    int      getTimeBlockCamIndex(const TimeBlock& tb) const { return tb.camIndex; }
    const std::string& getTimeBlockScope(const TimeBlock& tb) const { return tb.scope; }
    const std::string& getTimeBlockName(const TimeBlock& tb) const { return tb.name; }
    double   getTimeBlockBeginMs(const TimeBlock& tb) const { return tb.begin_ms; }
    double   getTimeBlockEndMs(const TimeBlock& tb) const { return tb.end_ms; }
    double   getTimeBlockTakenMs(const TimeBlock& tb) const { return tb.taken_ms; }
    unsigned getTimeBlockUsedOffset(const TimeBlock& tb) const { return tb.used_offset; }

    

private:
    bool     m_verbose        = false;   // Konsole drosseln
    unsigned m_logEveryN      = 50;      // nur jedes N-te Frame loggen
    bool     m_dumpAttrs      = false;   // Attribute-Dumps aus
    double   m_warnTolMs      = 5.0;     // nur für Debug-Warnungen
    unsigned m_gpuBackSearch  = 16;      // WICHTIG: größerer Backsearch
    bool     m_exportTimeline = true;    // Timeline-CSV schreiben


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

    bool tryAddBlock(osg::Stats* stats,
        unsigned int baseFrame,
        unsigned int backsearch,
        const std::string& statPrefix,
        const std::string& nameForCSV,
        const std::string& scope,
        int camIndex,
        std::vector<TimeBlock>& localBlocks);

    std::vector<TimeBlock> m_timeline;


    bool getAttributeForFrame(osg::Stats* stats, const std::string& attributeName, double& value, unsigned int frameNumber);
    void logAndCollectGPUStats(osgViewer::ViewerBase* viewer, const osg::FrameStamp* frameStamp);
    bool getAttributeForFrame(osg::Stats* stats, const std::string& attributeName, double& value);
    bool getAveragedAttribute(osg::Stats* stats, const std::string& attributeName, double& value);
    bool getLatestAttribute(osg::Stats* stats, const std::string& attributeName, double& value);
    void initCallbacks();
    bool hasStatAttribute(osg::Stats* stats, const std::string& key, unsigned int latestFrameNumber, unsigned int framesToCheck);
    void logAndCollectGPUStats(osgViewer::ViewerBase* viewer, unsigned int frameNumber);
    void drawIncrement(bool preDraw, const osg::FrameStamp* frameStamp);
    void drawIncrement(bool preDraw);
    void updateCamera(const osg::Vec3& pos, const osg::Vec3& rot);
    void writeTimelineCSV(const std::string& path);

    //void tryAddBlock(osg::Stats* stats, unsigned int baseFrame, unsigned int backsearch, const std::string& statPrefix, const std::string& nameForCSV, const std::string& scope, int camIndex, std::vector<LamureMeasurement::TimeBlock>& localBlocks);

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