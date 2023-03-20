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

LamureGeometry::LamureGeometry()
{
}

// should be using a file pointer instead of loading all data into memory
LamureGeometry::LamureGeometry(PointSet* pointData)
{
    // load data outside of the drawable
    setUseDisplayList(false);
    setSupportsDisplayList(false);
    setUseVertexBufferObjects(true);

    // maximum point size
    maxPointSize = 3.0;

    // save copy of pointData pointer
    pointSet = pointData;

    vertexBufferArray = getOrCreateVertexBufferObject();

    // set color and vertexArrays
    colors = new Vec3Array(pointSet->size, (osg::Vec3*)pointSet->colors);
    points = new Vec3Array(pointSet->size, (osg::Vec3*)pointSet->points);

    /*for (int n = 0; n < 1024; ++n)
    {
        std::cout << pointData[0].colors[n].r << std::endl;
        std::cout << pointData[0].colors[n].g << std::endl;
        std::cout << pointData[0].colors[n].b << std::endl;

        std::cout << pointData[0].points[n].coordinates.x() << std::endl;
        std::cout << pointData[0].points[n].coordinates.y() << std::endl;
        std::cout << pointData[0].points[n].coordinates.z() << std::endl;
    }*/

    vertexBufferArray->setArray(0, points);
    vertexBufferArray->setArray(1, colors);

    // bind color per vertex
    points->setBinding(osg::Array::BIND_PER_VERTEX);
    colors->setBinding(osg::Array::BIND_PER_VERTEX);

    setVertexArray(points);
    setColorArray(colors);

    // default initalization (modes 1,2,4,8)
    subsample = 0.3;

    // init bounding box
    box.init();

    // after test move stateset higher up in the tree
    stateset = new StateSet();
    //stateset->setMode(GL_PROGRAM_POINT_SIZE_EXT, StateAttribute::ON);
    stateset->setMode(GL_LIGHTING, StateAttribute::OFF);
    stateset->setMode(GL_DEPTH_TEST, StateAttribute::ON);
    stateset->setMode(GL_ALPHA_TEST, StateAttribute::ON);
    stateset->setMode(GL_BLEND, StateAttribute::OFF);
    AlphaFunc* alphaFunc = new AlphaFunc(AlphaFunc::GREATER, 0.5);
    stateset->setAttributeAndModes(alphaFunc, StateAttribute::ON);

    pointstate = new osg::Point();
    pointstate->setSize(maxPointSize);
    stateset->setAttributeAndModes(pointstate, StateAttribute::ON);

    osg::PointSprite* sprite = new osg::PointSprite();


    setStateSet(stateset);

    updateBounds();

}


#if OSG_VERSION_GREATER_OR_EQUAL(3, 3, 2)
BoundingBox LamureGeometry::computeBoundingBox() const
#else
BoundingBox PointCloudGeometry::computeBound() const
#endif
{
    return box;
}

void LamureGeometry::updateBounds()
{
    // init bounding box
    box.init();

    //expand box
    ::Point* data = pointSet->points;

    for (int i = 0; i < pointSet->size; i++)
    {
        box.expandBy(data[i].coordinates.x(), data[i].coordinates.y(), data[i].coordinates.z());
    }
    setInitialBound(box);
}

void LamureGeometry::updateCoords()
{
    std::cout << "updateCoords" << std::endl;
    memcpy(&points->at(0), pointSet->points, pointSet->size * sizeof(pointSet->points[0]));
    points->dirty();
    updateBounds();
}


// need to recode in a better way to automatically adjust primitives based on depth  //TODO
void LamureGeometry::changeLod(float sampleNum)
{
    std::cout << "changeLod" << std::endl;
    if (sampleNum < 0)
    {
        sampleNum = 0.;
    }
    if (sampleNum > 1.)
    {
        sampleNum = 1.;
    }

    if (subsample == sampleNum)
    {
        return;
    }

    subsample = sampleNum;

    if (getNumPrimitiveSets() == 0)
    {
        addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, (int)(pointSet->size * sampleNum)));
    }
    else
    {
        auto prim = getPrimitiveSet(0);
        auto arr = static_cast<osg::DrawArrays*>(prim);
        arr->setCount(pointSet->size * sampleNum);
    }

    setPointSize(pointSize);
}

void LamureGeometry::setPointSize(float newPointSize)
{
    std::cout << "setPointSize" << std::endl;
    pointSize = newPointSize;
    pointstate->setSize(pointSize / ((subsample / 4.0) + (3.0 / 4.0)));
}
