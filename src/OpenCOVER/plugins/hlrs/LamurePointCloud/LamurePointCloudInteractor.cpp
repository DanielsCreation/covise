#include "LamurePointCloudInteractor.h"
#include <cover/coIntersection.h>
#include <iostream>

LamurePointCloudInteractor::LamurePointCloudInteractor() {}
LamurePointCloudInteractor::~LamurePointCloudInteractor() {}

bool LamurePointCloudInteractor::canHandleDrawable(osg::Drawable* drawable) const
{
    return drawable && drawable->getName() == "LamurePointCloud";
}

void LamurePointCloudInteractor::intersect(osgUtil::IntersectionVisitor& iv, opencover::coIntersector& isect, osg::Drawable* drawable)
{
    std::multiset<osgUtil::LineSegmentIntersector::Intersection> hits  = isect.getIntersections();
    for (const auto& hit : hits)
    {
        // prüfen, ob das Hit tatsächlich zu unserem Drawable gehört
        if (hit.drawable == drawable)
        {
            // Weltkoordinate des Treffpunkts
            hitPointWorld_ = hit.getWorldIntersectPoint();
            // Welt-Normalenvektor
            hitNormalWorld_ = hit.getWorldIntersectNormal();
            // das getroffene Drawable merken
            hitDrawable_ = drawable;
            // und abbrechen – wir nehmen nur den nächsten Treffer
            std::cout << "[LPInteractor]   Treffer in Weltkoords: "
                << hitPointWorld_.x() << ", "
                << hitPointWorld_.y() << ", "
                << hitPointWorld_.z() << "\n";
            break;
        }
    }
}

