# scripts

`summarize_run.py` — summarizes a CSV from `mech run --headless --csv` (standard library
only): min/max/mean/final of any column, how often each contact lifted off, and the
energy ledger's worst residual. See `docs/EXPORT.md`.

## offscreen (headless-display) testing harnesses

These are NOT part of the shipped `mech` binary; they let the real
`src/viewer.cpp` run on a machine with no display, so screenshots can be
taken in CI or over ssh.

## glfw_shim.cpp (OSMesa)

A minimal GLFW API subset implemented over OSMesa (software GL). Build a
screenshot-capable binary with:

    g++ -std=c++17 -O2 -Isrc -DHAVE_VIEWER=1 \
        src/main.cpp src/viewer.cpp tests/tests.cpp \
        src/lexer.cpp src/parser.cpp src/model.cpp src/profile.cpp src/cam.cpp src/record.cpp src/render.cpp src/loader.cpp src/sweep.cpp \
        src/kinematics.cpp src/dynamics.cpp \
        scripts/glfw_shim.cpp -o mech-shot -lOSMesa -lGL

## glfw_shim_softgl.cpp (no GL at all)

The same GLFW subset over a tiny built-in software rasterizer of the
legacy-GL calls the viewer makes (lines, quads, polygons, ortho). Needs no
GL library — the stub headers in glfw3Stub/ provide GLFW/glfw3.h and
GL/gl.h:

    g++ -std=c++17 -O2 -Isrc -Iscripts/glfw3Stub -DHAVE_VIEWER=1 \
        src/main.cpp src/viewer.cpp tests/tests.cpp \
        src/lexer.cpp src/parser.cpp src/model.cpp src/profile.cpp src/cam.cpp src/record.cpp src/render.cpp src/loader.cpp src/sweep.cpp \
        src/kinematics.cpp src/dynamics.cpp \
        scripts/glfw_shim_softgl.cpp -o mech-shot

## running offscreen

    MECH_FRAMES=330 MECH_SHOT="320:shot.ppm" ./mech-shot run examples/clock.mech

dumps a P6 PPM of frame 320 and quits at frame 330.
`analyze_shot.py shot.ppm` summarizes what got drawn.
