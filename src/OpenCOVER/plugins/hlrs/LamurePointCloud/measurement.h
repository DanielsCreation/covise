#pragma once
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


class Measurement
{
public:
    struct Segment
    {
        osg::Vec3 tra;     // **neu:** Positionsänderung relativ zum Segment-Anfang
        osg::Vec3 rot;     // Drehwinkel-Delta in Grad (Pitch, Yaw, Roll)
        float     transSpeed;   // Translationstempo in Einheiten/s
        float     rotSpeed;     // Rotationsgeschwindigkeit in °/s 
    };

    Measurement(opencover::VRViewer*         viewer,
                const std::vector<Segment>&  segments,
                const std::string&           logfile,
                opencover::ui::Button*       measureButton,
                std::function<void(bool)>    buttonCB);
    ~Measurement();

    void writeLogAndStop();
    bool isRunning() const { return _running; }

private:
    void initCallbacks();
    void drawIncrement(bool preDraw);
    void updateCamera(const osg::Vec3& pos, const osg::Vec3& rot);

    opencover::ui::Button*            _measureButton;
    opencover::VRViewer*              _viewer;
    const std::vector<Segment>        _segments;
    size_t                            _currentSegment{0};
    double                            _segmentTime{0.0};
    osg::Vec3                         _cumulativeTra{0.0f,0.0f,0.0f};  // akkumulierte Deltas
    osg::Vec3                         _cumulativeRot{0.0f,0.0f,0.0f};
    osg::Vec3                         _initialPos;                     // Start-Position der Kamera
    osg::Matrix                         _initialXform;

    std::string                       _logfile;
    opencover::ui::Button*            _button;
    std::function<void(bool)>         _buttonCB;
    bool                              _running{false};
    osg::Timer_t                      _startTick;
    std::vector<std::pair<bool,double>> _drawTiming;

    struct MarkCallback : public osg::Camera::DrawCallback {
        Measurement* _meas;
        bool         _pre;
        MarkCallback(Measurement* m, bool pre) : _meas(m), _pre(pre) {}
        virtual void operator()(osg::RenderInfo& ri) const override {
            _meas->drawIncrement(_pre);
        }
    };
    MarkCallback* _preCB = nullptr;
    MarkCallback* _postCB = nullptr;
};
