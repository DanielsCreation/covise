#ifndef GL_STATE_H
#define GL_STATE_H

#ifndef __gl_h_
#include <GL/glew.h>
#endif

#include <vector>
#include <string>

// Einzelner Buffer‑Zustand
struct GLBufferInfo {
    GLuint id;              // Buffer‑Handle
    GLint size;             // Größe in Bytes
    std::string target;     // z.B. "GL_ARRAY_BUFFER"
};

// Einzelner Vertex‑Attrib‑Zustand
struct GLVertexAttribInfo {
    GLint   index;          // Attribut‑Index
    GLboolean enabled;      // aktiviert?
    GLint   size;           // Komponenten‑Anzahl
    GLenum  type;           // z.B. GL_FLOAT
    GLboolean normalized;   // normalisiert?
    GLint   stride;         // Byte‑Offset zwischen Elementen
    GLint   bufferBinding;  // aktuell gebundener Buffer
    void* pointer;        // Vertex‑Pointer
    GLint   divisor;        // Instancing‑Divisor
};

class GLState {
public:
    // 1) Erstelle ein "Snapshot" aller States
    static GLState capture();

    // 2) Stelle genau die States wieder her, die in dieser Instanz gespeichert sind
    void restore() const;

    // 3) Vergleiche zwei Snapshots und schreib Unterschiede auf std::cout
    static void compare(const GLState& before, const GLState& after, const std::string& info);

    static void printAllExistingBuffers(GLuint maxID);

    static void printCurrentContext();

    static void printBufferContents(GLenum target, GLuint buffer, size_t sizeInBytes);

    static void printVAOAttributes(GLuint vao);

    GLint getVertexArrayBinding() const { return m_vertexArrayBinding; }

private:
    static void printCandidateBuffer(GLuint candidate, const std::vector<GLenum>& targets);

    // Basis‑Zustände
    GLint       m_currentProgram;
    GLint       m_vertexArrayBinding;
    GLint       m_arrayBufferBinding;
    GLint       m_elementArrayBufferBinding;
    GLint       m_arrayBufferSize;
    GLint       m_elementArrayBufferSize;
    GLboolean   m_cullFaceEnabled;
    GLboolean   m_depthTestEnabled;
    GLboolean   m_blendEnabled;
    GLint       m_polygonMode[2];

    // Erweiterte Zustände
    GLint       m_viewport[4];
    GLint       m_scissorBox[4];
    GLfloat     m_clearColor[4];
    GLfloat     m_clearDepth;
    GLfloat     m_blendColor[4];
    GLint       m_blendSrcRGB;
    GLint       m_blendDstRGB;
    GLint       m_blendSrcAlpha;
    GLint       m_blendDstAlpha;
    GLint       m_blendEquationRGB;
    GLint       m_blendEquationAlpha;
    GLfloat     m_depthRange[2];
    GLfloat     m_lineWidth;
    GLfloat     m_pointSize;
    GLfloat     m_polygonOffsetFactor;
    GLfloat     m_polygonOffsetUnits;
    GLint       m_activeTexture;
    GLint       m_textureBinding2D;
    GLint       m_framebufferBinding;
    GLint       m_renderbufferBinding;

    // Buffer- und Attrib‑Listen
    std::vector<GLBufferInfo>        m_bufferInfos;
    std::vector<GLVertexAttribInfo>  m_vertexAttribInfos;
};

#endif // GL_STATE_H

