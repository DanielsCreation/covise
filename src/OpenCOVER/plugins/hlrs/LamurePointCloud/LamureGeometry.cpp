/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 // Local:
#include "LamurePointCloud.h"
#include "LamureGeometry.h"

#include <iostream>
#include <osg/Point>
#include <osg/Image>
#include <osg/AlphaFunc>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <cover/coVRFileManager.h>

using namespace osg;
using namespace std;

// should be using a file pointer instead of loading all data into memory
LamureGeometry::LamureGeometry(PointSet* pointData)
{

    // load data outside of the drawable
    setUseDisplayList(false);
    setSupportsDisplayList(false);
    setUseVertexBufferObjects(true);

    // save copy of pointData pointer
    pointSet = pointData;
    vertexBufferArray = getOrCreateVertexBufferObject();

    // set color and vertexArrays
    colors = new Vec3Array(pointSet->size, (osg::Vec3*)pointSet->colors);
    points = new Vec3Array(pointSet->size, (osg::Vec3*)pointSet->points);

    box.init();
    setInitialBound(box);
    ::Point* data = pointSet->points;
    for (int i = 0; i < pointSet->size; i++)
    {
        box.expandBy(data[i].coordinates.x(), data[i].coordinates.y(), data[i].coordinates.z());
    }
    setVertexArray(points);
    setColorArray(colors);
    addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, (int)(pointSet->size)));

    stateset = new StateSet();
    stateset->setMode(GL_LIGHTING, StateAttribute::ON);
    stateset->setMode(GL_BLEND, StateAttribute::ON);
    stateset->setMode(GL_POINT_SMOOTH, StateAttribute::ON);
    stateset->setAttribute(new osg::Point(10.0f), osg::StateAttribute::ON);
    setStateSet(stateset);
}

#if OSG_VERSION_GREATER_OR_EQUAL(3, 3, 2)
BoundingBox LamureGeometry::computeBoundingBox() const
#else
BoundingBox PointCloudGeometry::computeBound() const
#endif
{
    return box;
}

