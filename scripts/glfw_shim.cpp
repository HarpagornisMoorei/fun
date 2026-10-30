// glfw_shim.cpp — minimal GLFW API subset implemented over OSMesa (offscreen).
// Testing-only harness so mech's real viewer.cpp can run without a display.
#include <GLFW/glfw3.h>

#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

// OSMesa API (stable, from Mesa's include/GL/osmesa.h)
extern "C" {
typedef struct osmesa_context* OSMesaContext;
OSMesaContext OSMesaCreateContextExt(unsigned format, int depthBits, int stencilBits,
                                      int accumBits, OSMesaContext sharelist);
void OSMesaDestroyContext(OSMesaContext ctx);
int OSMesaMakeCurrent(OSMesaContext ctx, void* buffer, unsigned type, int width, int height);
}

namespace {

OSMesaContext g_ctx = nullptr;
unsigned char* g_buf = nullptr;
int g_w = 1000, g_h = 720;
bool g_shouldClose = false;

} // namespace

// GLFW API subset (glfw3.h declares these without extern "C" in C++)

int glfwInit(void) { return GLFW_TRUE; }

void glfwTerminate(void) {
    if (g_ctx) { OSMesaDestroyContext(g_ctx); g_ctx = nullptr; }
    if (g_buf) { std::free(g_buf); g_buf = nullptr; }
}

GLFWerrorfun glfwSetErrorCallback(GLFWerrorfun cb) { (void)cb; return nullptr; }

GLFWwindow* glfwCreateWindow(int w, int h, const char* title,
                             GLFWmonitor*, GLFWwindow*) {
    g_ctx = OSMesaCreateContextExt(GL_RGBA, 0, 0, 0, nullptr);
    if (!g_ctx) { std::fprintf(stderr, "shim: OSMesaCreateContextExt failed\n"); return nullptr; }
    g_buf = (unsigned char*)std::malloc((size_t)w * h * 4);
    if (!OSMesaMakeCurrent(g_ctx, g_buf, GL_UNSIGNED_BYTE, w, h)) {
        std::fprintf(stderr, "shim: OSMesaMakeCurrent failed\n"); return nullptr;
    }
    g_w = w; g_h = h;
    g_shouldClose = false;
    std::fprintf(stderr, "shim: offscreen window %dx%d '%s'\n", w, h, title);
    return (GLFWwindow*)0x1;
}

void glfwDestroyWindow(GLFWwindow*) {}

void glfwMakeContextCurrent(GLFWwindow*) {}
void glfwSwapInterval(int) {}

int glfwWindowShouldClose(GLFWwindow*) { return g_shouldClose ? GLFW_TRUE : GLFW_FALSE; }
void glfwSetWindowShouldClose(GLFWwindow*, int v) { g_shouldClose = v != 0; }

void glfwGetFramebufferSize(GLFWwindow*, int* w, int* h) { *w = g_w; *h = g_h; }

double glfwGetTime(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

int glfwGetKey(GLFWwindow*, int) { return GLFW_RELEASE; }
void glfwGetCursorPos(GLFWwindow*, double* x, double* y) { *x = 0; *y = 0; }
int glfwGetMouseButton(GLFWwindow*, int) { return GLFW_RELEASE; }
void glfwPollEvents(void) {}
void glfwSwapBuffers(GLFWwindow*) {}
void glfwSetWindowTitle(GLFWwindow*, const char*) {}
