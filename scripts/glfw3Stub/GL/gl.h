// gl.h — minimal legacy-GL subset for the software-GL test shim.
// Only what mech's viewer.cpp uses. Testing-only; not shipped.
#pragma once

extern "C" {

#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_LINES 0x0001
#define GL_LINE_LOOP 0x0002
#define GL_LINE_STRIP 0x0003
#define GL_TRIANGLE_FAN 0x0006
#define GL_QUADS 0x0007
#define GL_QUAD_STRIP 0x0008
#define GL_POLYGON 0x0009
#define GL_BLEND 0x0BE2
#define GL_SRC_ALPHA 0x0302
#define GL_ONE_MINUS_SRC_ALPHA 0x0303
#define GL_MODELVIEW 0x1700
#define GL_PROJECTION 0x1701
#define GL_RGB 0x1907
#define GL_UNSIGNED_BYTE 0x1401
#define GL_FRONT 0x0404
#define GL_PACK_ALIGNMENT 0x0D05

typedef float GLfloat;
typedef double GLdouble;
typedef unsigned char GLubyte;
typedef unsigned int GLenum;
typedef unsigned int GLbitfield;
typedef int GLint;
typedef int GLsizei;

void glClearColor(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
void glClear(GLbitfield mask);
void glColor4f(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha);
void glEnable(GLenum cap);
void glBlendFunc(GLenum sfactor, GLenum dfactor);
void glLineWidth(GLfloat width);
void glMatrixMode(GLenum mode);
void glLoadIdentity(void);
void glOrtho(GLdouble left, GLdouble right, GLdouble bottom, GLdouble top,
             GLdouble nearVal, GLdouble farVal);
void glViewport(GLint x, GLint y, GLsizei width, GLsizei height);
void glBegin(GLenum mode);
void glEnd(void);
void glVertex2d(GLdouble x, GLdouble y);
void glPixelStorei(GLenum pname, GLint param);
void glReadBuffer(GLenum mode);
void glReadPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum format,
                  GLenum type, void* data);

} // extern "C"
