/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 //gl
#ifndef __gl_h_
#include <GL/glew.h>
#endif
 
#include "LamureDrawable.h"
#include <osg/GL>
#include <osg/GLExtensions>
#include <osg/BufferObject>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Array>


LamureDrawable::LamureDrawable()
{
    drawable_lmr = this;
    box.init();
    //setInitialBound(box);
}


void LamureDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{

    /*std::cout << "getUseDisplayList(): " << getUseDisplayList() << std::endl;
    std::cout << "getSupportsDisplayList(): " << getSupportsDisplayList() << std::endl;
    std::cout << "getUseVertexBufferObjects(): " << getUseVertexBufferObjects() << std::endl;*/

    /*glBegin(GL_TRIANGLES);
    {
        glVertex3f(-500.0f, 0.0f, -500.0f);
        glVertex3f(500.0f, 0.0f, 500.0f);
        glVertex3f(500.0f, 0.0f, -500);
    }
    glEnd();*/


    //renderInfo.getState()->setUseStateAttributeShaders(true);
    //renderInfo.getState()->setUseStateAttributeFixedFunction(false);
    //renderInfo.getState()->setUseModelViewAndProjectionUniforms(false);
    //GLboolean b;
    //glGetBooleanv(GL_VERTEX_ARRAY, &b);
    //glGetDoublev
    //glGetFloatv
    //glGetIntegerv


    /*
    float data_vertices[6] = { 
        0.0f,  0.5f, // Vertex 1 (X, Y)
        0.5f, -0.5f, // Vertex 2 (X, Y)
       -0.5f, -0.5f  // Vertex 3 (X, Y) 
    };

    GLuint vbo;
    glGenBuffers(1, &vbo); // Generate 1 buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(data_vertices), data_vertices, GL_STATIC_DRAW);

    std::string vertexShader =
        "#version 130 core\n"
        "\n"
        "layout(location = 0) in vec2 position;"
        "\n"
        "void main()\n"
        "{\n"
        "   gl_Position = vec4(position, 0.0, 1.0);"
        "}\n";

    std::string fragmentShader =
        "#version 130 core\n"
        "\n"
        "out vec4 color;"
        "\n"
        "void main()\n"
        "{\n"
        "   color = vec4(1.0, 1.0, 1.0, 1.0);\n"
        "}\n";

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

    GLint posAttrib = glGetAttribLocation(program, "position");
    glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(posAttrib);
    glBindFragDataLocation(program, 0, "color");

    GLint num_uniforms;
    glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &num_uniforms);
    GLchar uniform_name[256];
    GLsizei length;
    GLint size;
    GLenum type;
    for (int i = 0; i < num_uniforms; i++)
    {
        glGetActiveUniform(program, i, sizeof(uniform_name), &length, &size, &type, uniform_name);
        // ... save this uniform data so it can be used during rendering
        std::cout << i << "glGetActiveUniform:" << std::endl;
        std::cout << i << ": " << uniform_name << ", type: " << type << std::endl;
    }

    glDrawArrays(GL_TRIANGLES, 0, 3);

    glDisableVertexAttribArray(posAttrib);*/


    /*float data_vertices[9] =
    {
        -300.0f, 0.0f, -300.0f, 
        0.0f, 0.0f, 300.0f,
        300.0f, 0.0f, -300.0f
    };

    std::string vertexShader =
        "#version 110 core\n"
        "\n"
        "layout(location = 0) in vec3 position;"
        "\n"
        "void main()\n"
        "{\n"
        "   gl_Position = position;"
        "}\n";

    std::string fragmentShader =
        "#version 330 core\n"
        "\n"
        "layout(location = 0) out vec4 color;"
        "\n"
        "void main()\n"
        "{\n"
        "   color = vec4(1.0, 0.0, 0.0, 1.0);\n"
        "}\n";

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
    glDisableVertexAttribArray(position_attrib_index);

    */



    /*drawable_lmr->setUseDisplayList(false);
    drawable_lmr->setSupportsDisplayList(false);
    drawable_lmr->setUseVertexBufferObjects(true);

    float positions[6] = { -300.0f, -300.0f, 0.0f, 300.0f, 300.0f, -300.0f };
    unsigned short myIndices[6] = { 0, 1, 2, 3, 4, 5 };
    int numIndices = sizeof(myIndices) / sizeof(unsigned short);


    osg::ref_ptr<osg::FloatArray> pos(new osg::FloatArray());
    osg::DrawElementsUInt* ar = new osg::DrawElementsUInt(GL_POINTS, 2048);
    osg::DrawElements* ps = new osg::DrawElementsUShort(osg::PrimitiveSet::POINTS, numIndices, myIndices);
    osg::ElementBufferObject* ebo = new osg::ElementBufferObject();

    ebo->addDrawElements(ps);*/
}



static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader, unsigned int osgid)
{
    osg::GLExtensions* glapi = new osg::GLExtensions(osgid);
    unsigned int program = glapi->glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader, osgid);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader, osgid);

    glapi->glAttachShader(program, vs);
    glapi->glAttachShader(program, fs);
    glapi->glLinkProgram(program);
    glapi->glValidateProgram(program);

    glapi->glDeleteProgram(vs);
    glapi->glDeleteProgram(fs);
    return 1;
}


static unsigned int CompileShader(unsigned int type, const std::string& source, unsigned int osgid)
{
    osg::GLExtensions* glapi = new osg::GLExtensions(osgid);
    unsigned int id = glapi->glCreateShader(type);
    const char* src = source.c_str();
    glapi->glShaderSource(id, 1, &src, nullptr);
    glapi->glCompileShader(id);

    int result;
    glapi->glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == false)
    {
        int length;
        glapi->glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        glapi->glGetShaderInfoLog(id, length, &length, message);
        std::cout << "Failed to compile " <<
            (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << std::endl;
        std::cout << message << std::endl;
        glapi->glDeleteProgram(id);
        return 0;
    };

    return id;
}


osg::Object* LamureDrawable::cloneType() const
{
    return new LamureDrawable();
}

osg::Object* LamureDrawable::clone(const osg::CopyOp&) const
{
    return new LamureDrawable();
}

osg::BoundingBox LamureDrawable::computeBoundingBox() const
{
    return osg::BoundingBox();
}


LamureDrawable::~LamureDrawable()
{
}