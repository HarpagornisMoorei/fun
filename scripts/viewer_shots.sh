#!/bin/sh
# Screenshots of the OpenGL viewer, taken without a display: the real src/viewer.cpp runs on the
# software-GL shim (scripts/glfw_shim_softgl.cpp), one frame is dumped, and a PNG is made of it.
#
#   sh scripts/viewer_shots.sh                 # builds the shot binary, then writes docs/img/viewer/*.png
#   SHOT=./mech-shot sh scripts/viewer_shots.sh   # use a binary you already built
#
# What this proves and what it does not: it runs the viewer's own drawing code (the scene of src/scene.cpp
# turned into GL calls) through a small rasterizer that implements the legacy-GL calls the viewer makes,
# including source-alpha blending. It shows geometry, colours and translucency. It does not prove anything about
# a particular GPU or driver (line widths above one pixel, antialiasing, the window system).
set -e
cd "$(dirname "$0")/.."
OUT=docs/img/viewer
SHOT=${SHOT:-./mech-shot}
if [ ! -x "$SHOT" ]; then
  echo "building $SHOT (a few minutes: the whole project, with the software GL)"
  g++ -std=c++17 -O1 -Isrc -Iscripts/glfw3Stub -DHAVE_VIEWER=1 \
      src/main.cpp src/viewer.cpp tests/tests.cpp \
      src/lexer.cpp src/parser.cpp src/model.cpp src/profile.cpp src/cam.cpp src/record.cpp src/render.cpp \
      src/scene.cpp src/loader.cpp src/sweep.cpp src/report.cpp src/kinematics.cpp src/dynamics.cpp src/diagnose.cpp \
      scripts/glfw_shim_softgl.cpp -o "$SHOT"
fi
mkdir -p "$OUT"
# name:frame — the frame is where the picture is taken (60 frames are about a second of motion)
for spec in boom:130 quickreturn:130 scotchyoke:60 slidingbead:40 camstar:120 planetary:120 ratchet:120 clock:200 rackpinion:90 beltdrive:60 rollingwheel:40 geneva:170; do
  name=${spec%%:*}; frame=${spec##*:}
  MECH_FRAMES=$((frame + 10)) MECH_SHOT="$frame:$OUT/$name.ppm" timeout 300 "$SHOT" run "examples/$name.mech" >/dev/null 2>&1
  python3 scripts/ppm2png.py "$OUT/$name.ppm" "$OUT/$name.png"
  rm -f "$OUT/$name.ppm"
  echo "wrote $OUT/$name.png"
done
