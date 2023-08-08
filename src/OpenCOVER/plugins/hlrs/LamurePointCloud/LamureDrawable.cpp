/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 //gl
#ifndef __gl_h_
#include <GL/glew.h>
#endif
 
#include "LamurePointCloud.h"
#include "LamureDrawable.h"
#include <cover/coVRConfig.h>
#include <cover/coVRPluginSupport.h>
#include <osg/GL>
#include <osg/GLExtensions>
#include <osg/BufferObject>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Array>
#include <osg/Geode>

#include "cover/OpenCOVER.h"

using namespace opencover;
using namespace covise;

LamureDrawable::LamureDrawable()
{
    pcl = this;
}


void LamureDrawable::config() {
    //osg::Geode* pcl_geode = new osg::Geode;
    //osg::StateSet* statesetBackgroundBin = new osg::StateSet();
    //statesetBackgroundBin->setRenderBinDetails(-2, "RenderBin");
    //statesetBackgroundBin->setNestRenderBins(false);
    //pcl_geode->addDrawable(pcl);
    //pcl->setStateSet(statesetBackgroundBin);
    //cover->getScene()->addChild(pcl_geode);
}

void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const {
    uint8_t ctx_id = renderInfo.getContextID();
    osg::GLExtensions* gl_api = new osg::GLExtensions(ctx_id);
    std::cout << "getUseDisplayList(): " << getUseDisplayList() << std::endl;
    std::cout << "getUseVertexArrayObject(): " << getUseVertexArrayObject() << std::endl;
    std::cout << "renderInfo.getState()->getUseStateAttributeFixedFunction(): " << renderInfo.getState()->getUseStateAttributeFixedFunction() << std::endl;
    std::cout << "getUseVertexBufferObjects(): " << getUseVertexBufferObjects() << std::endl;

    renderInfo.getState()->useVertexArrayObject(true);
    renderInfo.getState()->setUseStateAttributeShaders(true);
    renderInfo.getState()->setUseStateAttributeFixedFunction(false);
    renderInfo.getState()->setUseModelViewAndProjectionUniforms(false);


    float position[6] = {
         0.5f, -0.5f,  // Vertex 1 (X, Y)
        -0.5f, -0.5f,  // Vertex 2 (X, Y)
        -0.5f,  0.5f   // Vertex 3 (X, Y)
    };

    GLuint vbo;
    gl_api->glGenBuffers(1, &vbo);
    gl_api->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl_api->glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), position, GL_STATIC_DRAW);

    gl_api->glEnableVertexAttribArray(0);
    gl_api->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);

    std::string vertexShader =
        "#version 330 core\n"
        "\n"
        "layout(location = 0) in vec4 position;"
        "\n"
        "void main()\n"
        "{\n"
        "   gl_Position = position;\n"
        "}\n";

    std::cout << vertexShader << std::endl;

    std::string fragmentShader =
        "#version 330 core\n"
        "\n"
        "out vec4 color;"
        "\n"
        "void main()\n"
        "{\n"
        "   color = vec4(1.0, 1.0, 1.0, 1.0);\n"
        "}\n";

    std::cout << fragmentShader << std::endl;

    GLuint vshader = CreateShader(vertexShader, fragmentShader, ctx_id);
    gl_api->glUseProgram(vshader);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    


    /*
    float data_vertices[9] =
    {
        -300.0f, 0.0f, -300.0f,
        0.0f, 0.0f, 300.0f,
        300.0f, 0.0f, -300.0f
    };

    std::string vertexShader =
        "#version 110 core\n"
        "\n"
        "layout(location = 0) in vec3 position;\n"
        "\n"
        "void main()\n"
        "{\n"
        "   gl_Position = position;"
        "}\n";
    std::cout << vertexShader << std::endl;

    std::string fragmentShader =
        "#version 110 core\n"
        "\n"
        "layout(location = 0) out vec4 color;"
        "\n"
        "void main()\n"
        "{\n"
        "   color = vec4(1.0, 1.0, 1.0, 1.0);\n"
        "}\n";
    std::cout << fragmentShader << std::endl;

    const char* vertex_shader_source = vertexShader.c_str();
    const char* fragment_shader_source = fragmentShader.c_str();
    GLuint vshader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vshader, 1, &vertex_shader_source, NULL); // vertex_shader_source is a GLchar* containing glsl shader source code
    glCompileShader(vshader);
    GLint vertex_compiled;
    glGetShaderiv(vshader, GL_COMPILE_STATUS, &vertex_compiled);
    if (vertex_compiled != GL_TRUE)
    {
        GLsizei log_length = 0;
        GLchar message[1024];
        glGetShaderInfoLog(vshader, 1024, &log_length, message);
        // Write the error to a log
    }
    GLuint fshader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fshader, 1, &fragment_shader_source, NULL); // fragment_shader_source is a GLchar* containing glsl shader source code
    glCompileShader(fshader);
    GLint fragment_compiled;
    glGetShaderiv(fshader, GL_COMPILE_STATUS, &fragment_compiled);
    if (fragment_compiled != GL_TRUE)
    {
        GLsizei log_length = 0;
        GLchar message[1024];
        glGetShaderInfoLog(fshader, 1024, &log_length, message);
        // Write the error to a log
    }
    GLuint program = glCreateProgram();
    glBindAttribLocation(program, 0, "position");
    glAttachShader(program, vshader);
    glAttachShader(program, fshader);
    glLinkProgram(program);

    GLint program_linked;
    glGetProgramiv(program, GL_LINK_STATUS, &program_linked);
    if (program_linked != GL_TRUE)
    {
        GLsizei log_length = 0;
        GLchar message[1024];
        glGetProgramInfoLog(program, 1024, &log_length, message);
        // Write the error to a log
    }
    GLint position_attrib_index = glGetAttribLocation(program, "position");
    GLint colAttrib = glGetAttribLocation(program, "color");
    GLuint vertex_buffer; // Save this for later rendering
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glEnableVertexAttribArray(position_attrib_index);
    GLintptr vertex_position_offset = 0 * sizeof(float);
    glVertexAttribPointer(position_attrib_index, 3, GL_FLOAT, false, 3 * sizeof(float), (GLvoid*)vertex_position_offset); // vertex_data is a float*, 3 per vertex, representing the position of each vertex
    glDrawElements(GL_POINTS, 3, GL_UNSIGNED_INT, NULL);
    //glDisableVertexAttribArray(position_attrib_index);
    */


    //const char* vertex_shader_source = vertexShader.c_str();
    //const char* fragment_shader_source = fragmentShader.c_str();
    //glShaderSource(vshader, 1, &vertex_shader_source, NULL); // vertex_shader_source is a GLchar* containing glsl shader source code
    //glCompileShader(vshader);
    //GLint vertex_compiled;
    //glGetShaderiv(vshader, GL_COMPILE_STATUS, &vertex_compiled);
    //if (vertex_compiled != GL_TRUE)
    //{
    //    GLsizei log_length = 0;
    //    GLchar message[1024];
    //    glGetShaderInfoLog(vshader, 1024, &log_length, message);
    //    // Write the error to a log
    //}
    //GLuint fshader = glCreateShader(GL_FRAGMENT_SHADER);
    //glShaderSource(fshader, 1, &fragment_shader_source, NULL); // fragment_shader_source is a GLchar* containing glsl shader source code
    //glCompileShader(fshader);

    //GLint fragment_compiled;
    //glGetShaderiv(fshader, GL_COMPILE_STATUS, &fragment_compiled);
    //if (fragment_compiled != GL_TRUE)
    //{
    //    GLsizei log_length = 0;
    //    GLchar message[1024];
    //    glGetShaderInfoLog(fshader, 1024, &log_length, message);
    //    // Write the error to a log
    //}
    //GLuint program = glCreateProgram();
    //GLint posAttrib = glGetAttribLocation(program, "position");
    //glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);
    //glEnableVertexAttribArray(posAttrib);
    //glBindFragDataLocation(program, 0, "color");
    //GLint num_uniforms;
    //glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &num_uniforms);

    //GLchar uniform_name[256];
    //GLsizei length;
    //GLint size;
    //GLenum type;
    //for (int i = 0; i < num_uniforms; i++)
    //{
    //    glGetActiveUniform(program, i, sizeof(uniform_name), &length, &size, &type, uniform_name);
    //    // ... save this uniform data so it can be used during rendering
    //    std::cout << i << "glGetActiveUniform:" << std::endl;
    //    std::cout << i << ": " << uniform_name << ", type: " << type << std::endl;
    //}
    //glDisableVertexAttribArray(posAttrib);


}

/*
void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const {
    std::cout << "LamureDrawable::drawImplementation()" << std::endl;

    glBegin(GL_TRIANGLES);
    {
        glVertex3f(-500.0f, 0.0f, -500.0f);
        glVertex3f(500.0f, 0.0f, 500.0f);
        glVertex3f(500.0f, 0.0f, -500);
    }
    glEnd();

    //GLboolean b;
    //glGetBooleanv(GL_VERTEX_ARRAY, &b);
    //glGetDoublev
    //glGetFloatv
    //glGetIntegerv

    

    float positions[6] = { -300.0f, -300.0f, 0.0f, 300.0f, 300.0f, -300.0f };
    unsigned short myIndices[6] = { 0, 1, 2, 3, 4, 5 };
    int numIndices = sizeof(myIndices) / sizeof(unsigned short);

    //osg::ref_ptr<osg::FloatArray> pos(new osg::FloatArray());
    //osg::DrawElementsUInt* ar = new osg::DrawElementsUInt(GL_POINTS, 2048);
    //osg::DrawElements* ps = new osg::DrawElementsUShort(osg::PrimitiveSet::POINTS, numIndices, myIndices);
    //osg::ElementBufferObject* ebo = new osg::ElementBufferObject();

    //ebo->addDrawElements(ps);
}
*/

LamureDrawable::~LamureDrawable() {
    pcl = NULL;
}

osg::Object* LamureDrawable::cloneType() const {
    return new LamureDrawable();
}

osg::Object* LamureDrawable::clone(const osg::CopyOp&) const {
    return new LamureDrawable();
}




/*
void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const {
    static bool firstTime = true;
    static GLuint texHandle = 0;
    if (firstTime)
    {
        glGenTextures(1, &texHandle);
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texHandle);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
        firstTime = false;
    }
    bool rightVideo = false;
    if (osg::View* view = renderInfo.getView())
    {
        if (osg::State* state = renderInfo.getState())
        {
            if (const osg::DisplaySettings* ds = state->getDisplaySettings())
            {
                switch (ds->getStereoMode())
                {
                case osg::DisplaySettings::HORIZONTAL_INTERLACE:
                case osg::DisplaySettings::VERTICAL_INTERLACE:
                case osg::DisplaySettings::CHECKERBOARD:
                case osg::DisplaySettings::ANAGLYPHIC:
                    //TODO
                    break;
                case osg::DisplaySettings::HORIZONTAL_SPLIT:
                case osg::DisplaySettings::VERTICAL_SPLIT:
                    if (osg::Camera* cam = view->getCamera())
                    {
                        for (int i = 0; i < coVRConfig::instance()->numScreens(); ++i)
                        {
                            if (coVRConfig::instance()->channels[i].camera.get() == cam)
                            {
                                rightVideo = coVRConfig::instance()->channels[i].stereoMode == osg::DisplaySettings::RIGHT_EYE;
                                break;
                            }
                        }
                    }
                    break;
                case osg::DisplaySettings::LEFT_EYE:
                    break;
                case osg::DisplaySettings::RIGHT_EYE:
                    rightVideo = true;
                    break;
                case osg::DisplaySettings::QUAD_BUFFER:
                    if (osg::Camera* cam = view->getCamera())
                        rightVideo = (cam->getDrawBuffer() == GL_BACK_RIGHT || cam->getDrawBuffer() == GL_FRONT_RIGHT);
                    break;
                default:
                    cerr << "MarkerTrackingNode::drawImplementation: unknown fb_stereo mode" << endl;
                    break;
                }
            }
        }
    }

    GLint viewport[4]; // OpenGL viewport information (position and size)
    if (osg::View* view = renderInfo.getView())
    {
        osg::Camera* cam = view->getCamera();
        if (cam)
        {
            static bool firstTime = true;
            static GLuint texHandle2 = 0;
            static osg::Image* image = NULL;
            if (firstTime)
            {
                osgDB::ReaderWriter::Options* options = 0;
                image = osgDB::readImageFile("*.bvh", options);
            }

            if (image)
            {
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glGetIntegerv(GL_VIEWPORT, viewport);
                glDepthMask(false);

                glMatrixMode(GL_MODELVIEW);
                glLoadIdentity();
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluOrtho2D(-1, 1, -1, 1);

                glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texHandle2);

                glEnable(GL_TEXTURE_RECTANGLE_ARB); //
                glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //

                float xPos = 1.0;
                float yPos = 1.0;

                glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA8, image->s(), image->t(), 0, image->getPixelFormat(), GL_UNSIGNED_BYTE, image->data());
                glBegin(GL_QUADS);
                {
                    //glTexCoord2f(0, image->t());
                    //glVertex2f(-xPos, -yPos);
                    //glTexCoord2f(image->s(), image->t());
                    //glVertex2f(xPos, -yPos);
                    //glTexCoord2f(image->s(), 0);
                    //glVertex2f(xPos, yPos);
                    //glTexCoord2f(0, 0);
                    //glVertex2f(-xPos, yPos);
                    glTexCoord2f(0, 0);
                    glVertex2f(-xPos, -yPos);
                    glTexCoord2f(image->s(), 0);
                    glVertex2f(xPos, -yPos);
                    glTexCoord2f(image->s(), image->t());
                    glVertex2f(xPos, yPos);
                    glTexCoord2f(0, image->t());
                    glVertex2f(-xPos, yPos);
                }
                glEnd();

                glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
                glDisable(GL_TEXTURE_RECTANGLE_ARB);

                glDepthMask(true);
                glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();
                glMatrixMode(GL_MODELVIEW);
                glPopMatrix();

                firstTime = false;
            }
        }
    }


    if (LamureDrawable::instance()->display > 0)
    {
        // Save OpenGL state:
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glGetIntegerv(GL_VIEWPORT, viewport);
        glDepthMask(false);

        float xsize;
        float ysize;

        if ((coVRConfig::instance()->viewports[0].viewportXMax - coVRConfig::instance()->viewports[0].viewportXMin) == 0)
        {
            xsize = coVRConfig::instance()->windows[coVRConfig::instance()->viewports[0].window].sx;
            ysize = coVRConfig::instance()->windows[coVRConfig::instance()->viewports[0].window].sy;
        }
        else
        {
            xsize = coVRConfig::instance()->windows[coVRConfig::instance()->viewports[0].window].sx * (coVRConfig::instance()->viewports[0].viewportXMax - coVRConfig::instance()->viewports[0].viewportXMin);
            ysize = coVRConfig::instance()->windows[coVRConfig::instance()->viewports[0].window].sy * (coVRConfig::instance()->viewports[0].viewportYMax - coVRConfig::instance()->viewports[0].viewportYMin);
        }

        if (renderTextures) // textures
        {

            // DISPLAY TEXTURE //
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluOrtho2D(-1, 1, -1, 1);

            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texHandle);

            glEnable(GL_TEXTURE_RECTANGLE_ARB); //
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //

            float xPos = 1.0;
            float yPos = 1.0;

            if (LamureDrawable::instance()->flipH)
            {
                xPos *= -1;
            }

            if ((LamureDrawable::instance()->fb_stereo) && (rightVideo))
            {
                if (LamureDrawable::instance()->hasDataRight)
                {
                    //glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA8, LamureDrawable::instance()->fb_width, LamureDrawable::instance()->fb_height, 0, LamureDrawable::instance()->videoMode, GL_UNSIGNED_BYTE, LamureDrawable::instance()->data);
                    if (LamureDrawable::instance()->mirrorDataRight)
                    {
                        xPos *= -1;
                        yPos *= -1;
                    }
                }
            }
            else
            {
                if (LamureDrawable::instance()->videoMode)
                {
                    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA8, LamureDrawable::instance()->fb_width, LamureDrawable::instance()->fb_height, 0, LamureDrawable::instance()->videoMode, GL_UNSIGNED_BYTE, LamureDrawable::instance()->data);
                    if (LamureDrawable::instance()->mirrorDataLeft)
                    {
                        xPos *= -1;
                        yPos *= -1;
                    }
                }
            }

            glBegin(GL_QUADS);
            {
                glTexCoord2f(0, LamureDrawable::instance()->fb_height);
                glVertex2f(-xPos, -yPos);
                glTexCoord2f(LamureDrawable::instance()->fb_width, LamureDrawable::instance()->fb_height);
                glVertex2f(xPos, -yPos);
                glTexCoord2f(LamureDrawable::instance()->fb_width, 0);
                glVertex2f(xPos, yPos);
                glTexCoord2f(0, 0);
                glVertex2f(-xPos, yPos);
            }
            glEnd();

            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
            glDisable(GL_TEXTURE_RECTANGLE_ARB);
        }
        else //glDrawPixels
        {

            // Draw:
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            //        glViewport(0, 0, 1, 1);
            float yPos = 1.0;
            float xPos = -1.0;
            if (LamureDrawable::instance()->flipH)
            {
                ysize *= -1;
                yPos *= -1;
            }
            glPixelZoom(xsize / LamureDrawable::instance()->fb_width, -ysize / LamureDrawable::instance()->fb_height);
            if ((LamureDrawable::instance()->fb_stereo) && (rightVideo))
            {
                if (LamureDrawable::instance()->hasDataRight)
                {
                    if (LamureDrawable::instance()->mirrorDataRight)
                    {
                        glPixelZoom(-xsize / LamureDrawable::instance()->fb_width, ysize / LamureDrawable::instance()->fb_height);
                        yPos *= -1;
                        xPos *= -1;
                    }
                    glRasterPos2f(xPos, yPos);
                    glDrawPixels(LamureDrawable::instance()->fb_width, LamureDrawable::instance()->fb_height, LamureDrawable::instance()->videoMode, GL_UNSIGNED_BYTE, LamureDrawable::instance()->data);
                    if (LamureDrawable::instance()->mirrorDataRight)
                    {
                        glPixelZoom(xsize / LamureDrawable::instance()->fb_width, -ysize / LamureDrawable::instance()->fb_height);
                    }
                }
            }
            else
            {
                if (LamureDrawable::instance()->videoMode)
                {
                    if (LamureDrawable::instance()->mirrorDataLeft)
                    {
                        glPixelZoom(-xsize / LamureDrawable::instance()->fb_width, ysize / LamureDrawable::instance()->fb_height);
                        yPos *= -1;
                        xPos *= -1;
                    }
                    glRasterPos2f(xPos, yPos);
                    //if(MarkerTracking::instance()->hasData)
                    //cerr << "x " << (long long)MarkerTracking::instance()->hasData << " content: " << (int)MarkerTracking::instance()->hasData[100] << endl;
                    glDrawPixels(LamureDrawable::instance()->fb_width, LamureDrawable::instance()->fb_height, LamureDrawable::instance()->videoMode, GL_UNSIGNED_BYTE, LamureDrawable::instance()->data);
                }
            }
        }

        // Restore state:
        glDepthMask(true);
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }

    coVRPluginList::instance()->preDraw(renderInfo);
}
*/



//osg::Object* LamureDrawable::cloneType() const
//{
//    return new LamureDrawable();
//}
//
//osg::Object* LamureDrawable::clone(const osg::CopyOp&) const
//{
//    return new LamureDrawable();
//}
//
//osg::BoundingBox LamureDrawable::computeBoundingBox() const
//{
//    return osg::BoundingBox();
//}
//
//
//LamureDrawable::~LamureDrawable()
//{
//}