// measurement.cpp
#include "measurement.h"
#include <fstream>
#include <iostream>
#include <cmath>


Measurement::Measurement(opencover::VRViewer* viewer,
    const std::vector<Segment>& segments,
    const std::string& logfile,
    opencover::ui::Button* measureButton,
    std::function<void(bool)> buttonCB)
    : _viewer(viewer)
    , _segments(segments)
    , _logfile(logfile)
    , _button(measureButton)
    , _buttonCB(buttonCB)
{
    _running = true;
    _startTick = osg::Timer::instance()->tick();
    initCallbacks();
    std::cout << "Measurement(flight mode) started\n";
}


Measurement::~Measurement()
{
    if (_viewer && _viewer->getCamera()) {
        _viewer->getCamera()->setPreDrawCallback(nullptr);
        _viewer->getCamera()->setPostDrawCallback(nullptr);
    }
    //opencover::coVRNavigationManager::instance()->setRotationPoint(0.0f, 0.0f, 0.0f, 1.0f);
    //opencover::coVRNavigationManager::instance()->setRotationPointVisible(true);
}


void Measurement::initCallbacks()
{
    auto cam = _viewer->getCamera();
    _preCB = new MarkCallback(this, true);
    _postCB = new MarkCallback(this, false);
    cam->setPreDrawCallback(_preCB);
    cam->setPostDrawCallback(_postCB);
}


void Measurement::drawIncrement(bool preDraw)
{
    if (!_running) return;

    osg::Timer_t now = osg::Timer::instance()->tick();
    double t = osg::Timer::instance()->delta_s(_startTick, now);
    _drawTiming.emplace_back(preDraw, t);

    if (!preDraw)
    {
        // 1) dt berechnen
        double dt = 0;
        for (int i = int(_drawTiming.size()) - 2; i >= 0; --i) {
            if (!_drawTiming[i].first) {
                dt = t - _drawTiming[i].second;
                break;
            }
        }

        // 2) Segment-Zeit inkrementieren
        _segmentTime += dt;

        // 3) Segment-Wechsel
        while (_currentSegment < _segments.size())
        {
            const auto& seg = _segments[_currentSegment];
            // Dauer Translation
            float dist = seg.tra.length();
            double transDur = seg.transSpeed > 0.f ? dist / seg.transSpeed : 0.0;
            // Dauer Rotation (nach größtem Winkel)
            float maxAngle = std::max({ std::abs(seg.rot.x()), std::abs(seg.rot.y()), std::abs(seg.rot.z()) });
            double rotDur = seg.rotSpeed > 0.f ? maxAngle / seg.rotSpeed : 0.0;
            // Segment-Dauer = Maximum aus Translation und Rotation
            double segDuration = std::max(transDur, rotDur);

            // wenn noch nicht fertig, abbrechen
            if (_segmentTime < segDuration)
                break;

            // Segment abgeschlossen
            _segmentTime -= segDuration;
            _cumulativeTra += seg.tra;
            _cumulativeRot += seg.rot;
            ++_currentSegment;
        }

        // 4) Alle Segmente durch? -> Stop
        if (_currentSegment >= _segments.size())
        {
            _running = false;
            if (_button) {
                _buttonCB(false);
                _button->setState(false);
            }
            return;
        }

        // 5) Fraction berechnen für aktuellen Segment-Fortschritt
        const auto& seg = _segments[_currentSegment];
        float dist = seg.tra.length();
        double transDur = seg.transSpeed > 0.f ? dist / seg.transSpeed : 0.0;
        float maxAngle = std::max({ std::abs(seg.rot.x()), std::abs(seg.rot.y()), std::abs(seg.rot.z()) });
        double rotDur = seg.rotSpeed > 0.f ? maxAngle / seg.rotSpeed : 0.0;
        double segDuration = std::max(transDur, rotDur);
        double frac = segDuration > 0.0 ? std::min(_segmentTime / segDuration, 1.0) : 1.0;

        // 6) Interpolierte Position & Rotation
        osg::Vec3 tra = _cumulativeTra + seg.tra * float(frac);
        osg::Vec3 rot = _cumulativeRot + seg.rot * float(frac);

        // 7) Kamera aktualisieren
        updateCamera(tra, rot);
    }
}


void Measurement::updateCamera(const osg::Vec3& tra, const osg::Vec3& rot)
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


void Measurement::writeLogAndStop()
{
    if (_drawTiming.empty())
        return;

    std::ostream* out = nullptr;
    std::ofstream ofs;
    if (!_logfile.empty()) {
        ofs.open(_logfile);
        if (!ofs) { std::cerr << "[Measurement] Fehler beim Öffnen von " << _logfile << "\n"; }
        else { out = &ofs; }
    }
    else { out = &std::cout; }
    if (out) {
        *out << "# preDraw(1)/postDraw(0)  time[s]\n";
        for (auto& m : _drawTiming) {
            *out << (m.first ? 1 : 0) << "  " << m.second << "\n";
        }
    }
    _drawTiming.clear();
    _segmentTime = 0.0;
    _currentSegment = 0;
    _cumulativeRot = osg::Vec3(0.0f, 0.0f, 0.0f);
}
