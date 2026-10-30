// glfw_shim_softgl.cpp — minimal GLFW API subset over a tiny software GL.
// Testing-only harness so mech's real viewer.cpp can run with no display,
// no OSMesa, no GPU: the legacy fixed-function subset the viewer uses
// (clear/color/lines/quads/polygons + ortho + read pixels) is rasterized
// into an RGB8 buffer. MECH_SHOT="frame:file.ppm" and MECH_FRAMES work as
// with the OSMesa shim. Not part of the shipped binary.
#include "GLFW/glfw3.h"
#include "GL/gl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <vector>

// ---------------------------------------------------------------------------
// mini software GL
// ---------------------------------------------------------------------------
namespace {

struct Rgba { double r, g, b, a; };

unsigned char* g_pix = nullptr;      // W*H*3
int g_W = 1000, g_H = 720;
Rgba g_clear{0.0, 0.0, 0.0, 1.0};
Rgba g_color{1.0, 1.0, 1.0, 1.0};
double g_lineW = 1.0;
unsigned g_matrixMode = 0x1701;   // GL_PROJECTION by default
bool g_inBegin = false;
unsigned g_mode = 0;
std::vector<double> g_vx, g_vy;      // current vertex batch (world coords)

// projection: only glOrtho is ever used
double g_ol = 0, g_or = 1, g_ob = 0, g_ot = 1;

inline void putPx(int x, int y, const Rgba& c) {
    if (x < 0 || y < 0 || x >= g_W || y >= g_H) return;
    unsigned char* p = g_pix + (size_t)((g_H - 1 - y) * g_W + x) * 3;  // GL flips Y
    p[0] = (unsigned char)(std::max(0.0, std::min(255.0, c.r * 255.0)));
    p[1] = (unsigned char)(std::max(0.0, std::min(255.0, c.g * 255.0)));
    p[2] = (unsigned char)(std::max(0.0, std::min(255.0, c.b * 255.0)));
}

void drawLine(double x0, double y0, double x1, double y1) {
    // world -> pixel
    auto toPx = [&](double x, double y) {
        return std::make_pair((x - g_ol) / (g_or - g_ol) * g_W,
                              (y - g_ob) / (g_ot - g_ob) * g_H);
    };
    auto p0 = toPx(x0, y0);
    auto p1 = toPx(x1, y1);
    double ax = p0.first, ay = p0.second, bx = p1.first, by = p1.second;
    int steps = (int)std::ceil(std::max(std::fabs(bx - ax), std::fabs(by - ay))) + 1;
    const int w = (int)std::ceil(g_lineW);
    for (int i = 0; i <= steps; ++i) {
        const double t = steps ? (double)i / steps : 0.0;
        const int px = (int)std::lround(ax + t * (bx - ax));
        const int py = (int)std::lround(ay + t * (by - ay));
        for (int dy = -(w - 1) / 2; dy <= (w - 1) / 2; ++dy)
            for (int dx = -(w - 1) / 2; dx <= (w - 1) / 2; ++dx)
                putPx(px + dx, py + dy, g_color);
    }
}

void fillTri(double x0, double y0, double x1, double y1, double x2, double y2) {
    auto toPx = [&](double x, double y) {
        return std::make_pair((x - g_ol) / (g_or - g_ol) * g_W,
                              (y - g_ob) / (g_ot - g_ob) * g_H);
    };
    auto a = toPx(x0, y0), b = toPx(x1, y1), c = toPx(x2, y2);
    const double minx = std::min({a.first, b.first, c.first});
    const double maxx = std::max({a.first, b.first, c.first});
    const double miny = std::min({a.second, b.second, c.second});
    const double maxy = std::max({a.second, b.second, c.second});
    const double area = (b.first - a.first) * (c.second - a.second) -
                        (c.first - a.first) * (b.second - a.second);
    if (std::fabs(area) < 1e-12) return;
    for (int py = (int)std::floor(miny); py <= (int)std::ceil(maxy); ++py) {
        for (int px = (int)std::floor(minx); px <= (int)std::ceil(maxx); ++px) {
            const double cx = px + 0.5, cy = py + 0.5;
            const double w0 = ((b.first - a.first) * (cy - a.second) -
                               (cx - a.first) * (b.second - a.second)) / area;
            const double w1 = ((cx - a.first) * (c.second - a.second) -
                               (c.first - a.first) * (cy - a.second)) / area;
            const double w2 = 1.0 - w0 - w1;
            if (w0 >= -1e-9 && w1 >= -1e-9 && w2 >= -1e-9) putPx(px, py, g_color);
        }
    }
}

void flushBatch() {
    const size_t n = g_vx.size();
    switch (g_mode) {
        case GL_LINES:
            for (size_t i = 0; i + 1 < n; i += 2) drawLine(g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1]);
            break;
        case GL_LINE_STRIP:
            for (size_t i = 0; i + 1 < n; ++i) drawLine(g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1]);
            break;
        case GL_LINE_LOOP:
            for (size_t i = 0; i + 1 < n; ++i) drawLine(g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1]);
            if (n > 2) drawLine(g_vx[n - 1], g_vy[n - 1], g_vx[0], g_vy[0]);
            break;
        case GL_QUADS:
            for (size_t i = 0; i + 3 < n; i += 4) {
                fillTri(g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1], g_vx[i + 2], g_vy[i + 2]);
                fillTri(g_vx[i], g_vy[i], g_vx[i + 2], g_vy[i + 2], g_vx[i + 3], g_vy[i + 3]);
            }
            break;
        case GL_QUAD_STRIP:
            for (size_t i = 0; i + 3 < n; i += 2)
                fillTri(g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1], g_vx[i + 2], g_vy[i + 2]),
                fillTri(g_vx[i + 1], g_vy[i + 1], g_vx[i + 3], g_vy[i + 3], g_vx[i + 2], g_vy[i + 2]);
            break;
        case GL_TRIANGLE_FAN:
            for (size_t i = 1; i + 1 < n; ++i)
                fillTri(g_vx[0], g_vy[0], g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1]);
            break;
        case GL_POLYGON:
            for (size_t i = 1; i + 1 < n; ++i)
                fillTri(g_vx[0], g_vy[0], g_vx[i], g_vy[i], g_vx[i + 1], g_vy[i + 1]);
            break;
        default:
            break;
    }
    g_vx.clear();
    g_vy.clear();
}

} // namespace

// --- the GL subset the viewer uses -------------------------------------------

void glClearColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { g_clear = {r, g, b, a}; }
void glClear(GLbitfield mask) {
    if (!(mask & GL_COLOR_BUFFER_BIT)) return;
    const unsigned char c[3] = {
        (unsigned char)(std::max(0.0, std::min(255.0, g_clear.r * 255.0))),
        (unsigned char)(std::max(0.0, std::min(255.0, g_clear.g * 255.0))),
        (unsigned char)(std::max(0.0, std::min(255.0, g_clear.b * 255.0))),
    };
    for (size_t i = 0; i < (size_t)g_W * g_H * 3; i += 3) g_pix[i] = c[0], g_pix[i + 1] = c[1], g_pix[i + 2] = c[2];
}
void glColor4f(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { g_color = {r, g, b, a}; }
void glEnable(GLenum) {}
void glBlendFunc(GLenum, GLenum) {}
void glLineWidth(GLfloat w) { g_lineW = (double)w; }
void glMatrixMode(GLenum mode) { g_matrixMode = mode; }
void glLoadIdentity(void) {
    if (g_matrixMode == GL_PROJECTION) { g_ol = 0; g_or = 1; g_ob = 0; g_ot = 1; }
    // the modelview is always identity here (2-D ortho only)
}
void glOrtho(double l, double r, double b, double t, double, double) {
    if (g_matrixMode != GL_PROJECTION) return;
    g_ol = l; g_or = r; g_ob = b; g_ot = t;
}
void glViewport(GLint x, GLint y, GLsizei w, GLsizei h) {
    // full-window viewport only (what the viewer uses)
    (void)x; (void)y; (void)w; (void)h;
}
void glBegin(GLenum mode) { g_inBegin = true; g_mode = mode; g_vx.clear(); g_vy.clear(); }
void glEnd(void) {
    if (g_inBegin) flushBatch();
    g_inBegin = false;
}
void glVertex2d(double x, double y) { g_vx.push_back(x); g_vy.push_back(y); }
void glPixelStorei(GLenum, GLint) {}
void glReadBuffer(GLenum) {}
void glReadPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, void* data) {
    if (format != GL_RGB || type != GL_UNSIGNED_BYTE) return;
    unsigned char* out = (unsigned char*)data;
    for (int j = 0; j < height; ++j) {
        // GL row 0 is the bottom row
        const int srcY = g_H - 1 - (y + j);
        for (int i = 0; i < width; ++i) {
            const int srcX = x + i;
            unsigned char rgb[3] = {0, 0, 0};
            if (srcX >= 0 && srcX < g_W && srcY >= 0 && srcY < g_H) {
                const unsigned char* p = g_pix + ((size_t)srcY * g_W + srcX) * 3;
                rgb[0] = p[0]; rgb[1] = p[1]; rgb[2] = p[2];
            }
            out[((size_t)j * width + i) * 3 + 0] = rgb[0];
            out[((size_t)j * width + i) * 3 + 1] = rgb[1];
            out[((size_t)j * width + i) * 3 + 2] = rgb[2];
        }
    }
}


// ---------------------------------------------------------------------------
// GLFW subset
// ---------------------------------------------------------------------------
namespace {

bool g_shouldClose = false;
char g_title[256] = "mech";
double g_time = 0.0;
std::vector<unsigned char> g_framebuf;

} // namespace

extern "C" {

int glfwInit(void) {
    delete[] g_pix;
    g_pix = new unsigned char[(size_t)g_W * g_H * 3];
    std::memset(g_pix, 20, (size_t)g_W * g_H * 3);
    g_shouldClose = false;
    g_time = 0.0;
    return 1;   // GLFW_TRUE
}
void glfwTerminate(void) {}
GLFWwindow* glfwCreateWindow(int w, int h, const char* title, const void*, const void*) {
    g_W = w > 0 ? w : g_W;
    g_H = h > 0 ? h : g_H;
    std::snprintf(g_title, sizeof g_title, "%s", title ? title : "mech");
    delete[] g_pix;
    g_pix = new unsigned char[(size_t)g_W * g_H * 3];
    std::memset(g_pix, 20, (size_t)g_W * g_H * 3);
    return (GLFWwindow*)(uintptr_t)1;   // a non-null dummy handle
}
void glfwDestroyWindow(GLFWwindow*) {}
int glfwWindowShouldClose(GLFWwindow*) { return g_shouldClose ? 1 : 0; }
void glfwSetWindowShouldClose(GLFWwindow*, int value) { g_shouldClose = value != 0; }
void glfwMakeContextCurrent(GLFWwindow*) {}
void glfwSwapBuffers(GLFWwindow*) {}
void glfwSwapInterval(int) {}
void glfwPollEvents(void) { g_time += 1.0 / 60.0; }
double glfwGetTime(void) { return g_time; }
void glfwGetFramebufferSize(GLFWwindow*, int* w, int* h) { *w = g_W; *h = g_H; }
void glfwGetCursorPos(GLFWwindow*, double* x, double* y) { *x = g_W * 0.5; *y = g_H * 0.5; }
int glfwGetKey(GLFWwindow*, int) { return 0 /* GLFW_RELEASE */; }
int glfwGetMouseButton(GLFWwindow*, int) { return 0; }
void glfwSetWindowTitle(GLFWwindow*, const char* title) {
    std::snprintf(g_title, sizeof g_title, "%s", title ? title : "mech");
    std::fprintf(stderr, "softgl: title '%s'\n", g_title);
}
GLFWerrorfun glfwSetErrorCallback(GLFWerrorfun cb) { return cb; }

} // extern "C"
