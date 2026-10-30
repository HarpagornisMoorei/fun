# mech — M1 walking skeleton of a mechanism language
#
# The viewer needs GLFW. Without it, everything except the window still
# builds and runs (check / run --headless / test).

CXX      ?= g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Isrc

CORE_SRC := src/lexer.cpp src/parser.cpp src/model.cpp src/kinematics.cpp src/dynamics.cpp
CORE_HDR := src/geom.hpp src/ast.hpp src/lexer.hpp src/parser.hpp src/linalg.hpp \
            src/model.hpp src/kinematics.hpp src/dynamics.hpp src/viewer.hpp tests/tests.hpp

HAVE_GLFW := $(shell pkg-config --exists glfw3 && echo 1 || echo 0)

ifeq ($(HAVE_GLFW),1)
VIEWER_FLAGS := $(shell pkg-config --cflags glfw3) -DHAVE_VIEWER=1
VIEWER_LIBS  := $(shell pkg-config --libs glfw3) -lGL
else
VIEWER_FLAGS :=
VIEWER_LIBS  :=
$(info NOTE: GLFW not found -> building headless-only binary.)
$(info      Install libglfw3-dev and re-run make for the OpenGL window.)
endif

.PHONY: all test clean

all: mech

mech: src/main.cpp src/viewer.cpp tests/tests.cpp $(CORE_SRC) $(CORE_HDR)
	$(CXX) $(CXXFLAGS) $(VIEWER_FLAGS) src/main.cpp src/viewer.cpp \
	    tests/tests.cpp $(CORE_SRC) -o $@ $(VIEWER_LIBS)

test: mech
	./mech test

clean:
	rm -f mech
