/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 //
 
#include "LamureDrawable.h"

LamureDrawable* LamureDrawable::lmrNode = NULL;

LamureDrawable::LamureDrawable()
{
    lmrNode = this;
}


void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    glBegin(GL_TRIANGLES);
    {
        glVertex2f(-500.0f, -500.0f);
        glVertex2f(500.0f, 500.0f);
        glVertex2f(500.0f, -500);

        
    }
    glEnd();
}



osg::Object* LamureDrawable::cloneType() const
{
    return new LamureDrawable();
}

osg::Object* LamureDrawable::clone(const osg::CopyOp&) const
{
    return new LamureDrawable();
}


LamureDrawable::~LamureDrawable()
{
    delete lmrNode;
}