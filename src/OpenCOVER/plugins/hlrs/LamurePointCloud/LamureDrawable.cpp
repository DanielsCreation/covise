/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 //
 
#include "LamureDrawable.h"


LamureDrawable::LamureDrawable()
{
}


void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
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
}