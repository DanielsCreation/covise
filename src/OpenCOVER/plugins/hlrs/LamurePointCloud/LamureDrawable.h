#ifndef LAMUREDRAWABLE_H
#define LAMUREDRAWABLE_H

/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */



#include <string>
#include <iostream>

#include <osg/Drawable>
#include <osg/Object>
#include <osg/RenderInfo>
#include <osg/CopyOp>
#include <osg/BoundingBox>
#include <osg/Version>



class LamureDrawable : public osg::Drawable
{
public:
    LamureDrawable::LamureDrawable();
    ~LamureDrawable();

    void drawImplementation(osg::RenderInfo& renderInfo) const override;

    osg::Object* LamureDrawable::cloneType() const override;

    osg::Object* LamureDrawable::clone(const osg::CopyOp&) const override;

    osg::ref_ptr<LamureDrawable> drawable_lmr = NULL;

protected:
#if OSG_VERSION_GREATER_OR_EQUAL(3, 3, 2)
    virtual osg::BoundingBox computeBoundingBox() const;
#else
    virtual osg::BoundingBox computeBound() const;
#endif

private:

    osg::BoundingBox box;
    osg::ref_ptr<osg::ElementBufferObject> ebo = NULL;

};

static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader, unsigned int osgid);

static unsigned int CompileShader(unsigned int type, const std::string& source, unsigned int osgid);

#endif


