// glfw3.h — minimal GLFW API subset for the software-GL test shim.
// Only what mech's viewer.cpp uses. Testing-only; not shipped.
#pragma once

// the real GLFW header drags in the GL declarations; so does this one
#include "../GL/gl.h"

extern "C" {

#define GLFW_RELEASE 0
#define GLFW_PRESS 1
#define GLFW_TRUE 1
#define GLFW_FALSE 0
#define GLFW_KEY_SPACE 32
#define GLFW_KEY_Q 81
#define GLFW_KEY_R 82
#define GLFW_KEY_F 70
#define GLFW_KEY_ESCAPE 256
#define GLFW_KEY_1 49
#define GLFW_MOUSE_BUTTON_LEFT 0

typedef struct GLFWwindow GLFWwindow;
typedef void (*GLFWerrorfun)(int, const char*);

int glfwInit(void);
void glfwTerminate(void);
GLFWwindow* glfwCreateWindow(int width, int height, const char* title,
                             const void* monitor, const void* share);
void glfwDestroyWindow(GLFWwindow* window);
int glfwWindowShouldClose(GLFWwindow* window);
void glfwSetWindowShouldClose(GLFWwindow* window, int value);
void glfwMakeContextCurrent(GLFWwindow* window);
void glfwSwapBuffers(GLFWwindow* window);
void glfwSwapInterval(int interval);
void glfwPollEvents(void);
double glfwGetTime(void);
void glfwGetFramebufferSize(GLFWwindow* window, int* width, int* height);
void glfwGetCursorPos(GLFWwindow* window, double* xpos, double* ypos);
int glfwGetKey(GLFWwindow* window, int key);
int glfwGetMouseButton(GLFWwindow* window, int button);
void glfwSetWindowTitle(GLFWwindow* window, const char* title);
GLFWerrorfun glfwSetErrorCallback(GLFWerrorfun errorfun);

} // extern "C"
