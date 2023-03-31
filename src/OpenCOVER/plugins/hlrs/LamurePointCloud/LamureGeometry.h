/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _LAMUREGEOMETRY_DRAWABLE_H_
#define _LAMUREGEOMETRY_DRAWABLE_H_

#include <osg/Version>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/BufferObject>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

#include "C:\src\covise\src\OpenCOVER\plugins\hlrs\LamurePointCloud\Points.h"

class LamureGeometry : public osg::Geometry
{
public:
    LamureGeometry(::PointSet*);
    LamureGeometry(const LamureGeometry& drawimage, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);
    void changeLod(float sampleNum); // adjust point density
    void setPointSize(float newPointSize); // adjust point size
    void updateCoords();

protected:
    //LamureGeometry(); // hide default constructor
    LamureGeometry& operator=(const LamureGeometry&)
    {
        return *this;
    }

#if OSG_VERSION_GREATER_OR_EQUAL(3, 3, 2)
    virtual osg::BoundingBox computeBoundingBox() const;
#else
    virtual osg::BoundingBox computeBound() const;
#endif

private:
    PointSet* pointSet;
    float subsample = 1.;
    float pointSize = 1.;
    float maxPointSize;
    osg::Point* pointstate;
    osg::StateSet* stateset;
    osg::BoundingBox box;
    osg::Vec3Array* points;
    osg::Vec3Array* colors;
    osg::VertexBufferObject* vertexBufferArray;
    osg::ElementBufferObject* primitiveBufferArray;
};
#endif
