#pragma once

#include <osg/ref_ptr>
#include <osgUtil/IntersectionVisitor>
#include <osg/Drawable>
#include "cover/coIntersection.h"
#include <osg/Vec3>

namespace opencover {
    class coIntersector;
}

class LamurePointCloudInteractor : public opencover::IntersectionHandler
{
public:
    LamurePointCloudInteractor();
    ~LamurePointCloudInteractor() override;

    // Hier filterst Du, ob das Drawable für Dich interessant ist:
    bool canHandleDrawable(osg::Drawable* drawable) const override;

    // Wird aufgerufen, wenn für ein Drawable ein Hit gefunden wurde:
    void intersect(osgUtil::IntersectionVisitor& iv, opencover::coIntersector& isect, osg::Drawable* drawable) override;

    // Hier speicherst Du Dein eigenes Hit-Resultat
    osg::Vec3                    hitPointWorld_;
    osg::Vec3                    hitNormalWorld_;
    osg::ref_ptr<osg::Drawable>  hitDrawable_;
};
