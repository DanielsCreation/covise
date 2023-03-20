/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 //
 //

#include "LamureDrawable.h"
#include "LamurePointCloud.h"
#include <cover/coVRConfig.h>

LamureDrawable::ContextState::ContextState()
{
}

LamureDrawable::ContextState::~ContextState()
{
}
LamureDrawable::LamureDrawable()
{
#ifdef VERBOSE
    cerr << "VolumeDrawable::<init> warn: empty constructor called" << endl;
#endif
    init();
}

LamureDrawable::LamureDrawable(const LamureDrawable& drawable,
    const osg::CopyOp& copyop)
    : Drawable(drawable, copyop)
{
#ifdef VERBOSE
    cerr << "VolumeDrawable::<init> copying" << endl;
#endif
    init();
}

void LamureDrawable::init()
{
    setSupportsDisplayList(false);
}


LamureDrawable::~LamureDrawable()
{
#ifdef VERBOSE
    cerr << "VolumeDrawable::<dtor>: this=" << this << endl;
#endif
    contextState.clear();
}

void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
    osg::ref_ptr<osg::StateSet> currentState = new osg::StateSet;
    renderInfo.getState()->captureCurrentState(*currentState);
    renderInfo.getState()->pushStateSet(currentState.get());

    char* argv[] = { "opencover" };
    scm::shared_ptr<scm::core> scm_core(new scm::core(1, argv));

    int max_upload_budget = 64;
    int video_memory_budget = 2048;
    int main_memory_budget = 4096;


    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(max_upload_budget); //8
    policy->set_render_budget_in_mb(video_memory_budget); //2048
    policy->set_out_of_core_budget_in_mb(main_memory_budget); //4096, 8192
    policy->set_window_width(coVRConfig::instance()->windows[0].sx);
    policy->set_window_height(coVRConfig::instance()->windows[0].sy);


    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();

    std::vector<scm::math::mat4d> parsed_views = std::vector<scm::math::mat4d>();


    LamureDrawable* ld = (LamureDrawable*)this;

    //const unsigned ctx = renderInfo.getState()->getContextID();
    //while (ctx >= contextState.size())
    //{
    //    // this will delete the old renderer contextState.resize(ctx+1);
    //    ContextState* nc = new ContextState;
    //    contextState.push_back(nc);
    //}
    //    ld->MainLoop();
    //    renderInfo.getState()->popStateSet();
}