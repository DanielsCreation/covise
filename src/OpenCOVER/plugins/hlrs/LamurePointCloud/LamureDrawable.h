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

#include <cover/coVRPluginSupport.h>
#include <cover/coVRMSController.h>
#include <cover/coVRPluginList.h>
#include <cover/coVRCommunication.h>
#include <cover/coVRConfig.h>
#include <cover/coVRTui.h>
#include <cover/coVRShader.h>
#include <cover/VRViewer.h>
#include <cover/PluginMenu.h>
#include <cover/ui/ButtonGroup.h>
#include <cover/ui/Button.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Action.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Manager.h>
#include <cover/ui/Owner.h>
#include <cover/ui/SelectionList.h>
#include <cover/coVRStatsDisplay.h>
#include <cover/VRSceneGraph.h>
#include "cover/OpenCOVER.h"
#include <cover/VRWindow.h>
#include <cover/VRViewer.h>
#include <cover/coVRFileManager.h>


class LamureDrawable : public osg::Drawable
{
private:
    bool display; // true if CoviseConfig.displayVideo is set
    bool videoMode;
    bool renderTextures;
    bool flipH;
    bool flipV;
    bool hasDataLeft;
    bool hasDataRight;
    bool mirrorDataLeft;
    bool mirrorDataRight;
    bool fb_stereo;
    int fb_width;
    int fb_height;
    void* data;
    std::string pcl_node_m;

protected:


public:
    LamureDrawable* pcl = NULL;

    LamureDrawable();
    ~LamureDrawable();


    virtual void drawImplementation(osg::RenderInfo& renderInfo) const;

    /** Clone the type of an object, with Object* return type.
    Must be defined by derived classes.*/
    virtual osg::Object* cloneType() const;
    /** Clone the an object, with Object* return type.
        Must be defined by derived classes.*/
    virtual osg::Object* clone(const osg::CopyOp&) const;

    void config();
    //void update();
};


//class LamureDrawable : public osg::Drawable
//{
//public:
//    LamureDrawable::LamureDrawable();
//    ~LamureDrawable();
//
//    void drawImplementation(osg::RenderInfo& renderInfo) const override;
//
//    osg::Object* LamureDrawable::cloneType() const override;
//
//    osg::Object* LamureDrawable::clone(const osg::CopyOp&) const override;
//
//    osg::ref_ptr<LamureDrawable> drawable_lmr = NULL;
//
//protected:
//#if OSG_VERSION_GREATER_OR_EQUAL(3, 3, 2)
//    virtual osg::BoundingBox computeBoundingBox() const;
//#else
//    virtual osg::BoundingBox computeBound() const;
//#endif
//
//private:
//
//    osg::BoundingBox box;
//    osg::ref_ptr<osg::ElementBufferObject> ebo = NULL;
//
//};

static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader, uint8_t ctx_id);

static unsigned int CompileShader(unsigned int type, const std::string& source, uint8_t ctx_id);



#endif


