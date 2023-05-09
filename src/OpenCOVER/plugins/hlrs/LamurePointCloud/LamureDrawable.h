#ifndef LAMUREDRAWABLE_H
#define LAMUREDRAWABLE_H

/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */



#include <osg/Drawable>
#include <osg/Object>
#include <osg/RenderInfo>
#include <osg/CopyOp>

#include "LamurePointCloud.h"

class LamureDrawable : public osg::Drawable
{

public:
    LamureDrawable::LamureDrawable();
    ~LamureDrawable();

    void drawImplementation(osg::RenderInfo& renderInfo) const override;

    osg::Object* LamureDrawable::cloneType() const override;

    osg::Object* LamureDrawable::clone(const osg::CopyOp&) const override;

};

#endif
