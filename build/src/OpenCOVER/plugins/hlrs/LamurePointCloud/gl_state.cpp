#include "gl_state.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <windows.h>

// Hilfsfunktion: Alle existierenden Buffer-IDs bis maxID ermitteln
static std::set<GLuint> enumerateAllBufferIDs(GLuint maxID = 100) {
    std::set<GLuint> buffers;
    for (GLuint id = 1; id <= maxID; ++id) {
        if (glIsBuffer(id))
            buffers.insert(id);
    }
    return buffers;
}

// Hilfsfunktion: Informationen zu einem Buffer abfragen
static GLBufferInfo queryBufferInfo(GLuint id, GLenum target) {
    GLBufferInfo info;
    info.id = id;
    info.target = (target == GL_ARRAY_BUFFER)
        ? "GL_ARRAY_BUFFER"
        : "GL_ELEMENT_ARRAY_BUFFER";

    GLint oldBinding = 0;
    if (target == GL_ARRAY_BUFFER)
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &oldBinding);
    else
        glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &oldBinding);

    glBindBuffer(target, id);
    glGetBufferParameteriv(target, GL_BUFFER_SIZE, &info.size);
    glBindBuffer(target, oldBinding);
    return info;
}

// Hilfsfunktion: Zustand aller Vertex-Attribs erfassen
static std::vector<GLVertexAttribInfo> captureVertexAttribState() {
    std::vector<GLVertexAttribInfo> attribs;
    GLint maxAttribs = 0;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &maxAttribs);

    for (GLint i = 0; i < maxAttribs; ++i) {
        GLVertexAttribInfo a;
        a.index = i;
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, reinterpret_cast<GLint*>(&a.enabled));
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &a.size);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, reinterpret_cast<GLint*>(&a.type));
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED, reinterpret_cast<GLint*>(&a.normalized));
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &a.stride);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &a.bufferBinding);
        glGetVertexAttribPointerv(i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &a.pointer);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_DIVISOR, &a.divisor);
        attribs.push_back(a);
    }
    return attribs;
}

// ----- 1) Capture aller GL-States -----
GLState GLState::capture() {
    GLState s;
    // Basis-Zustände
    glGetIntegerv(GL_CURRENT_PROGRAM, &s.m_currentProgram);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &s.m_vertexArrayBinding);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &s.m_arrayBufferBinding);
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &s.m_elementArrayBufferBinding);

    // Buffer-Größen
    s.m_arrayBufferSize = 0;
    if (s.m_arrayBufferBinding) {
        GLint old; glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &old);
        glBindBuffer(GL_ARRAY_BUFFER, s.m_arrayBufferBinding);
        glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &s.m_arrayBufferSize);
        glBindBuffer(GL_ARRAY_BUFFER, old);
    }
    s.m_elementArrayBufferSize = 0;
    if (s.m_elementArrayBufferBinding) {
        GLint old; glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &old);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s.m_elementArrayBufferBinding);
        glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &s.m_elementArrayBufferSize);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, old);
    }

    s.m_cullFaceEnabled = glIsEnabled(GL_CULL_FACE);
    s.m_depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
    s.m_blendEnabled = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_POLYGON_MODE, s.m_polygonMode);

    // Erweiterte Zustände
    glGetIntegerv(GL_VIEWPORT, s.m_viewport);
    glGetIntegerv(GL_SCISSOR_BOX, s.m_scissorBox);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, s.m_clearColor);
    glGetFloatv(GL_DEPTH_CLEAR_VALUE, &s.m_clearDepth);
    glGetFloatv(GL_BLEND_COLOR, s.m_blendColor);
    glGetIntegerv(GL_BLEND_SRC_RGB, &s.m_blendSrcRGB);
    glGetIntegerv(GL_BLEND_DST_RGB, &s.m_blendDstRGB);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &s.m_blendSrcAlpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &s.m_blendDstAlpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &s.m_blendEquationRGB);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &s.m_blendEquationAlpha);
    glGetFloatv(GL_DEPTH_RANGE, s.m_depthRange);
    glGetFloatv(GL_LINE_WIDTH, &s.m_lineWidth);
    glGetFloatv(GL_POINT_SIZE, &s.m_pointSize);
    glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &s.m_polygonOffsetFactor);
    glGetFloatv(GL_POLYGON_OFFSET_UNITS, &s.m_polygonOffsetUnits);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &s.m_activeTexture);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &s.m_textureBinding2D);
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &s.m_framebufferBinding);
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &s.m_renderbufferBinding);

    // Buffers
    for (GLuint id : enumerateAllBufferIDs()) {
        s.m_bufferInfos.push_back(queryBufferInfo(id, GL_ARRAY_BUFFER));
        s.m_bufferInfos.push_back(queryBufferInfo(id, GL_ELEMENT_ARRAY_BUFFER));
    }

    // Vertex-Attribs
    s.m_vertexAttribInfos = captureVertexAttribState();
    return s;
}

// ----- 2) Restore der gesicherten States -----
void GLState::restore() const {
    // Programm + VAO
    glUseProgram(m_currentProgram);
    glBindVertexArray(m_vertexArrayBinding);

    // Vertex-Attribs
    for (const auto& a : m_vertexAttribInfos) {
        glBindBuffer(GL_ARRAY_BUFFER, a.bufferBinding);
        glVertexAttribPointer(a.index,
            a.size,
            a.type,
            a.normalized,
            a.stride,
            a.pointer);
        glVertexAttribDivisor(a.index, a.divisor);
        if (a.enabled) glEnableVertexAttribArray(a.index);
        else           glDisableVertexAttribArray(a.index);
    }

    // Buffers
    glBindBuffer(GL_ARRAY_BUFFER, m_arrayBufferBinding);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementArrayBufferBinding);

    // FBO + Renderbuffer
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebufferBinding);
    glBindRenderbuffer(GL_RENDERBUFFER, m_renderbufferBinding);

    // Texturen
    glActiveTexture(m_activeTexture);
    glBindTexture(GL_TEXTURE_2D, m_textureBinding2D);

    // Viewport + Scissor
    glViewport(m_viewport[0], m_viewport[1], m_viewport[2], m_viewport[3]);
    glScissor(m_scissorBox[0], m_scissorBox[1], m_scissorBox[2], m_scissorBox[3]);

    // Clear-Werte
    glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
    glClearDepth(m_clearDepth);
}

// ----- 3) Vergleich zweier States -----
void GLState::compare(const GLState& before,
    const GLState& after,
    const std::string& info)
{
    // VAO Binding
    if (before.m_vertexArrayBinding != after.m_vertexArrayBinding)
        std::cout << "GL_VERTEX_ARRAY_BINDING changed: "
        << before.m_vertexArrayBinding
        << " -> "
        << after.m_vertexArrayBinding << std::endl;

    // Vertex-Attrib-Anzahl
    if (before.m_vertexAttribInfos.size() != after.m_vertexAttribInfos.size()) {
        std::cout << "Number of vertex attrib entries changed: "
            << before.m_vertexAttribInfos.size() << " -> "
            << after.m_vertexAttribInfos.size() << std::endl;
    }
    else {
        for (size_t i = 0; i < before.m_vertexAttribInfos.size(); ++i) {
            const auto& b = before.m_vertexAttribInfos[i];
            const auto& a = after.m_vertexAttribInfos[i];
            if (b.enabled != a.enabled ||
                b.size != a.size ||
                b.type != a.type ||
                b.normalized != a.normalized ||
                b.stride != a.stride ||
                b.bufferBinding != a.bufferBinding ||
                b.pointer != a.pointer) {
                std::cout << "Vertex Attrib " << b.index << " changed:" << std::endl;
                std::cout << "  Before: enabled=" << b.enabled
                    << ", size=" << b.size
                    << ", type=" << b.type
                    << ", normalized=" << (b.normalized ? "YES" : "NO")
                    << ", stride=" << b.stride
                    << ", bufferBinding=" << b.bufferBinding
                    << ", pointer=" << b.pointer << std::endl;
                std::cout << "  After : enabled=" << a.enabled
                    << ", size=" << a.size
                    << ", type=" << a.type
                    << ", normalized=" << (a.normalized ? "YES" : "NO")
                    << ", stride=" << a.stride
                    << ", bufferBinding=" << a.bufferBinding
                    << ", pointer=" << a.pointer << std::endl;
            }
        }
    }

    // Buffer-Vergleich
    auto cmpBuf = [](const GLBufferInfo& x, const GLBufferInfo& y) {
        if (x.id != y.id)
            return x.id < y.id;
        return x.target < y.target;
        };
    std::vector<GLBufferInfo> sb = before.m_bufferInfos;
    std::vector<GLBufferInfo> sa = after.m_bufferInfos;
    std::sort(sb.begin(), sb.end(), cmpBuf);
    std::sort(sa.begin(), sa.end(), cmpBuf);

    size_t i = 0, j = 0;
    while (i < sb.size() || j < sa.size()) {
        if (i < sb.size() && j < sa.size()) {
            const auto& b = sb[i];
            const auto& a = sa[j];
            if (b.id == a.id && b.target == a.target) {
                if (b.size != a.size) {
                    std::cout << "Buffer modified: ID=" << b.id
                        << ", Target=" << b.target
                        << ", Size changed: " << b.size
                        << " -> " << a.size << " bytes" << std::endl;
                }
                ++i; ++j;
            }
            else if ((b.id < a.id) || (b.id == a.id && b.target < a.target)) {
                std::cout << "Buffer removed: ID=" << b.id
                    << ", Target=" << b.target
                    << ", Size=" << b.size << " bytes" << std::endl;
                ++i;
            }
            else {
                std::cout << "New buffer created: ID=" << a.id
                    << ", Target=" << a.target
                    << ", Size=" << a.size << " bytes" << std::endl;
                ++j;
            }
        }
        else if (j < sa.size()) {
            const auto& a = sa[j++];
            std::cout << "New buffer created: ID=" << a.id
                << ", Target=" << a.target
                << ", Size=" << a.size << " bytes" << std::endl;
        }
        else {
            const auto& b = sb[i++];
            std::cout << "Buffer removed: ID=" << b.id
                << ", Target=" << b.target
                << ", Size=" << b.size << " bytes" << std::endl;
        }
    }

    // Program
    if (before.m_currentProgram != after.m_currentProgram) {
        std::cout << info << std::endl
            << "GL_CURRENT_PROGRAM changed: "
            << before.m_currentProgram
            << " -> "
            << after.m_currentProgram << std::endl;
    }

    // Array-Buffer Binding & Size
    if (before.m_arrayBufferBinding != after.m_arrayBufferBinding)
        std::cout << "GL_ARRAY_BUFFER_BINDING changed: "
        << before.m_arrayBufferBinding
        << " -> "
        << after.m_arrayBufferBinding << std::endl;
    if (before.m_arrayBufferSize != after.m_arrayBufferSize)
        std::cout << "GL_ARRAY_BUFFER size changed: "
        << before.m_arrayBufferSize
        << " -> "
        << after.m_arrayBufferSize
        << " bytes" << std::endl;

    // Element-Array-Buffer Binding & Size
    if (before.m_elementArrayBufferBinding != after.m_elementArrayBufferBinding)
        std::cout << "GL_ELEMENT_ARRAY_BUFFER_BINDING changed: "
        << before.m_elementArrayBufferBinding
        << " -> "
        << after.m_elementArrayBufferBinding << std::endl;
    if (before.m_elementArrayBufferSize != after.m_elementArrayBufferSize)
        std::cout << "GL_ELEMENT_ARRAY_BUFFER size changed: "
        << before.m_elementArrayBufferSize
        << " -> "
        << after.m_elementArrayBufferSize
        << " bytes" << std::endl;

    // Simple Enable/Disable States
    if (before.m_cullFaceEnabled != after.m_cullFaceEnabled)
        std::cout << "GL_CULL_FACE state changed: "
        << (before.m_cullFaceEnabled ? "enabled" : "disabled")
        << " -> "
        << (after.m_cullFaceEnabled ? "enabled" : "disabled") << std::endl;
    if (before.m_depthTestEnabled != after.m_depthTestEnabled)
        std::cout << "GL_DEPTH_TEST state changed: "
        << (before.m_depthTestEnabled ? "enabled" : "disabled")
        << " -> "
        << (after.m_depthTestEnabled ? "enabled" : "disabled") << std::endl;
    if (before.m_blendEnabled != after.m_blendEnabled)
        std::cout << "GL_BLEND state changed: "
        << (before.m_blendEnabled ? "enabled" : "disabled")
        << " -> "
        << (after.m_blendEnabled ? "enabled" : "disabled") << std::endl;

    // Polygon Mode
    if (before.m_polygonMode[0] != after.m_polygonMode[0] || before.m_polygonMode[1] != after.m_polygonMode[1])
        std::cout << "GL_POLYGON_MODE changed: (" << before.m_polygonMode[0] << ", " << before.m_polygonMode[1] << ") -> (" << after.m_polygonMode[0] << ", " << after.m_polygonMode[1] << ")" << std::endl;

    // Viewport & Scissor
    bool vpChanged = false; for (int k = 0; k < 4; ++k) if (before.m_viewport[k] != after.m_viewport[k]) { vpChanged = true; break; }
    if (vpChanged)
        std::cout << "GL_VIEWPORT changed: (" << before.m_viewport[0] << ", " << before.m_viewport[1] << ", " << before.m_viewport[2] << ", " << before.m_viewport[3] << ") -> (" << after.m_viewport[0] << ", " << after.m_viewport[1] << ", " << after.m_viewport[2] << ", " << after.m_viewport[3] << ")" << std::endl;
    bool scChanged = false; for (int k = 0; k < 4; ++k) if (before.m_scissorBox[k] != after.m_scissorBox[k]) { scChanged = true; break; }
    if (scChanged)
        std::cout << "GL_SCISSOR_BOX changed: (" << before.m_scissorBox[0] << ", " << before.m_scissorBox[1] << ", " << before.m_scissorBox[2] << ", " << before.m_scissorBox[3] << ") -> (" << after.m_scissorBox[0] << ", " << after.m_scissorBox[1] << ", " << after.m_scissorBox[2] << ", " << after.m_scissorBox[3] << ")" << std::endl;

    // Clear Colors & Depth
    bool ccChanged = false; for (int k = 0; k < 4; ++k) if (before.m_clearColor[k] != after.m_clearColor[k]) { ccChanged = true; break; }
    if (ccChanged)
        std::cout << "GL_COLOR_CLEAR_VALUE changed: (" << before.m_clearColor[0] << ", " << before.m_clearColor[1] << ", " << before.m_clearColor[2] << ", " << before.m_clearColor[3] << ") -> (" << after.m_clearColor[0] << ", " << after.m_clearColor[1] << ", " << after.m_clearColor[2] << ", " << after.m_clearColor[3] << ")" << std::endl;
    if (before.m_clearDepth != after.m_clearDepth)
        std::cout << "GL_DEPTH_CLEAR_VALUE changed: " << before.m_clearDepth << " -> " << after.m_clearDepth << std::endl;

    // Blend Color & Equations & Factors
    bool bcChanged = false; for (int k = 0; k < 4; ++k) if (before.m_blendColor[k] != after.m_blendColor[k]) { bcChanged = true; break; }
    if (bcChanged)
        std::cout << "GL_BLEND_COLOR changed: (" << before.m_blendColor[0] << ", " << before.m_blendColor[1] << ", " << before.m_blendColor[2] << ", " << before.m_blendColor[3] << ") -> (" << after.m_blendColor[0] << ", " << after.m_blendColor[1] << ", " << after.m_blendColor[2] << ", " << after.m_blendColor[3] << ")" << std::endl;
    if (before.m_blendSrcRGB != after.m_blendSrcRGB)
        std::cout << "GL_BLEND_SRC_RGB changed: " << before.m_blendSrcRGB << " -> " << after.m_blendSrcRGB << std::endl;
    if (before.m_blendDstRGB != after.m_blendDstRGB)
        std::cout << "GL_BLEND_DST_RGB changed: " << before.m_blendDstRGB << " -> " << after.m_blendDstRGB << std::endl;
    if (before.m_blendSrcAlpha != after.m_blendSrcAlpha)
        std::cout << "GL_BLEND_SRC_ALPHA changed: " << before.m_blendSrcAlpha << " -> " << after.m_blendSrcAlpha << std::endl;
    if (before.m_blendDstAlpha != after.m_blendDstAlpha)
        std::cout << "GL_BLEND_DST_ALPHA changed: " << before.m_blendDstAlpha << " -> " << after.m_blendDstAlpha << std::endl;
    if (before.m_blendEquationRGB != after.m_blendEquationRGB)
        std::cout << "GL_BLEND_EQUATION_RGB changed: " << before.m_blendEquationRGB << " -> " << after.m_blendEquationRGB << std::endl;
    if (before.m_blendEquationAlpha != after.m_blendEquationAlpha)
        std::cout << "GL_BLEND_EQUATION_ALPHA changed: " << before.m_blendEquationAlpha << " -> " << after.m_blendEquationAlpha << std::endl;

    // Rest: Depth Range, Line Width, Point Size...
    if (before.m_depthRange[0] != after.m_depthRange[0] || before.m_depthRange[1] != after.m_depthRange[1])
        std::cout << "GL_DEPTH_RANGE changed: (" << before.m_depthRange[0] << ", " << before.m_depthRange[1] << ") -> (" << after.m_depthRange[0] << ", " << after.m_depthRange[1] << ")" << std::endl;
    if (before.m_lineWidth != after.m_lineWidth)
        std::cout << "GL_LINE_WIDTH changed: " << before.m_lineWidth << " -> " << after.m_lineWidth << std::endl;
    if (before.m_pointSize != after.m_pointSize)
        std::cout << "GL_POINT_SIZE changed: " << before.m_pointSize << " -> " << after.m_pointSize << std::endl;
    if (before.m_polygonOffsetFactor != after.m_polygonOffsetFactor)
        std::cout << "GL_POLYGON_OFFSET_FACTOR changed: " << before.m_polygonOffsetFactor << " -> " << after.m_polygonOffsetFactor << std::endl;
    if (before.m_polygonOffsetUnits != after.m_polygonOffsetUnits)
        std::cout << "GL_POLYGON_OFFSET_UNITS changed: " << before.m_polygonOffsetUnits << " -> " << after.m_polygonOffsetUnits << std::endl;

    // Texture + FBO + RBO Bindings
    if (before.m_activeTexture != after.m_activeTexture)
        std::cout << "GL_ACTIVE_TEXTURE changed: " << before.m_activeTexture << " -> " << after.m_activeTexture << std::endl;
    if (before.m_textureBinding2D != after.m_textureBinding2D)
        std::cout << "GL_TEXTURE_BINDING_2D changed: " << before.m_textureBinding2D << " -> " << after.m_textureBinding2D << std::endl;
    if (before.m_framebufferBinding != after.m_framebufferBinding)
        std::cout << "GL_FRAMEBUFFER_BINDING changed: " << before.m_framebufferBinding << " -> " << after.m_framebufferBinding << std::endl;
    if (before.m_renderbufferBinding != after.m_renderbufferBinding)
        std::cout << "GL_RENDERBUFFER_BINDING changed: " << before.m_renderbufferBinding << " -> " << after.m_renderbufferBinding << std::endl;
}



void GLState::printAllExistingBuffers(GLuint maxID) {
    std::vector<GLenum> targets = { GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER };
    std::cout << "=== Overview of All Buffers ===" << std::endl;
    for (GLuint i = 1; i <= maxID; ++i) {
        if (glIsBuffer(i)) {
            GLState::printCandidateBuffer(i, targets);
        }
    }
}


void GLState::printCurrentContext() {
#ifdef _WIN32
    HGLRC context = wglGetCurrentContext();
    HDC device = wglGetCurrentDC();
    std::cout << "Current OpenGL Context (wglGetCurrentContext): " << context << std::endl;
    std::cout << "Current Device Context (wglGetCurrentDC): " << device << std::endl;
#elif defined(__linux__)
    GLXContext context = glXGetCurrentContext();
    Display* display = glXGetCurrentDisplay();
    std::cout << "Current OpenGL Context (glXGetCurrentContext): " << context << std::endl;
    std::cout << "Current Display (glXGetCurrentDisplay): " << display << std::endl;
#else
    std::cout << "Context query not implemented for this system." << std::endl;
#endif
}


void GLState::printBufferContents(GLenum target, GLuint buffer, size_t sizeInBytes) {
    GLState::printCurrentContext();
    GLint oldBinding = 0;
    if (target == GL_ARRAY_BUFFER) {
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &oldBinding);
    }
    else if (target == GL_ELEMENT_ARRAY_BUFFER) {
        glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &oldBinding);
    }

    std::cout << "Target: " << ((target == GL_ARRAY_BUFFER) ? "GL_ARRAY_BUFFER" : "GL_ELEMENT_ARRAY_BUFFER") << std::endl;
    std::cout << "Buffer ID: " << buffer << std::endl;
    std::cout << "Previously bound buffer: " << oldBinding
        << ((static_cast<GLuint>(oldBinding) == buffer) ? " (same)" : " (different)") << std::endl;

    glBindBuffer(target, buffer);

    void* ptr = glMapBuffer(target, GL_READ_ONLY);
    if (ptr == nullptr) {
        std::cerr << "glMapBuffer() failed." << std::endl;
        glBindBuffer(target, oldBinding);
        return;
    }

    if (target == GL_ARRAY_BUFFER) {
        float* data = static_cast<float*>(ptr);
        size_t numElements = sizeInBytes / sizeof(float);
        std::cout << "Array Buffer Contents (" << numElements << " float elements):" << std::endl;
        for (size_t i = 0; i < numElements; i++) {
            std::cout << "Element " << i << ": " << data[i] << std::endl;
        }
    }
    else if (target == GL_ELEMENT_ARRAY_BUFFER) {
        unsigned short* data = static_cast<unsigned short*>(ptr);
        size_t numElements = sizeInBytes / sizeof(unsigned short);
        std::cout << "Element Array Buffer Contents (" << numElements << " unsigned short elements):" << std::endl;
        for (size_t i = 0; i < numElements; i++) {
            std::cout << "Element " << i << ": " << data[i] << std::endl;
        }
    }
    else {
        std::cerr << "Unknown buffer target." << std::endl;
    }

    glUnmapBuffer(target);
    glBindBuffer(target, oldBinding);
}


void GLState::printVAOAttributes(GLuint vao) {
    GLint previousVAO = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previousVAO);
    printCurrentContext();
    glBindVertexArray(vao);
    GLint maxAttribs = 0;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &maxAttribs);
    std::cout << "Maximum Vertex Attributes: " << maxAttribs << std::endl;

    for (GLint i = 0; i < maxAttribs; ++i) {
        GLint enabled = 0;
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &enabled);
        std::cout << "Attribute " << i << " enabled: " << enabled << std::endl;

        if (enabled) {
            GLint size = 0, type = 0, stride = 0, bufferBinding = 0;
            glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &size);
            glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, &type);
            glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &stride);
            glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &bufferBinding);
            std::cout << "  Size: " << size
                << ", Type: " << type
                << ", Stride: " << stride
                << ", Buffer Binding: " << bufferBinding << std::endl;
        }
    }
    glBindVertexArray(previousVAO);
}


void GLState::printCandidateBuffer(GLuint candidate, const std::vector<GLenum>& targets) {
    std::cout << "Buffer Handle: " << candidate << std::endl;
    for (auto target : targets) {
        std::string targetName;
        switch (target) {
        case GL_ARRAY_BUFFER: targetName = "GL_ARRAY_BUFFER"; break;
        case GL_ELEMENT_ARRAY_BUFFER: targetName = "GL_ELEMENT_ARRAY_BUFFER"; break;
        default: targetName = "Unknown Target"; break;
        }
        GLint oldBinding = 0;
        if (target == GL_ARRAY_BUFFER)
            glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &oldBinding);
        else if (target == GL_ELEMENT_ARRAY_BUFFER)
            glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &oldBinding);
        bool isBound = (static_cast<GLuint>(oldBinding) == candidate);
        std::cout << "  Target: " << targetName
            << ", Currently Bound: " << (isBound ? "YES" : "NO") << std::endl;

        glBindBuffer(target, candidate);
        GLint bufferSize = 0;
        glGetBufferParameteriv(target, GL_BUFFER_SIZE, &bufferSize);
        std::cout << "    Buffer Type: " << targetName << ", Size: " << bufferSize << " bytes" << std::endl;
        if (bufferSize > 0) {
            void* ptr = glMapBuffer(target, GL_READ_ONLY);
            if (ptr) {
                if (target == GL_ARRAY_BUFFER) {
                    float* data = static_cast<float*>(ptr);
                    int numElements = bufferSize / sizeof(float);
                    std::cout << "    Elements (first 4): ";
                    for (int i = 0; i < std::min(numElements, 4); ++i)
                        std::cout << data[i] << " ";
                    std::cout << "\n    Elements (last 4): ";
                    for (int i = std::max(0, numElements - 4); i < numElements; ++i)
                        std::cout << data[i] << " ";
                    std::cout << std::endl;
                }
                else if (target == GL_ELEMENT_ARRAY_BUFFER) {
                    unsigned short* data = static_cast<unsigned short*>(ptr);
                    int numElements = bufferSize / sizeof(unsigned short);
                    std::cout << "    Elements (first 4): ";
                    for (int i = 0; i < std::min(numElements, 4); ++i)
                        std::cout << data[i] << " ";
                    std::cout << "\n    Elements (last 4): ";
                    for (int i = std::max(0, numElements - 4); i < numElements; ++i)
                        std::cout << data[i] << " ";
                    std::cout << std::endl;
                }
                glUnmapBuffer(target);
            }
            else {
                std::cout << "    Buffer mapping failed." << std::endl;
            }
        }
        else {
            std::cout << "    Buffer size is 0." << std::endl;
        }
        glBindBuffer(target, oldBinding);
    }
    std::cout << std::endl;
}
