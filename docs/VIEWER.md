# The OpenGL viewer (`mech run`)

`mech run file.mech` opens a window and animates the mechanism. It needs a machine with a display and GLFW
(`libglfw3-dev` on Debian/Ubuntu); without GLFW the binary builds a stub that tells you to run headless or
use `mech render`.

Controls: **space** pause/resume, **R** reset, **F** refit the view, **wheel** zoom, **drag** pan, **1–6** speed,
**Q**/**Esc** quit. `--trace body.anchor` draws the path of an anchor (a coupler curve), `--speed x` and `--paused`
set the start.

## It draws the same picture as `mech render`

Since M15 the viewer has no drawing logic of its own. `src/scene.cpp` turns a pose into a list of drawing items
(lines, circles, polygons, polylines, in world coordinates), and two things draw that list:

* `mech render` writes it as SVG (`docs/RENDER.md`);
* the viewer draws it with OpenGL's legacy immediate-mode calls (`src/viewer.cpp`: bars as rectangles with round
  ends, thick rings as annuli, translucent fills as ear-clipped triangles).

So everything in the SVG renderer is in the window too: gears with a tick per tooth and ring gears as bands (the
larger gear of an internal mesh only), cam and ratchet profiles (concave ones too), rods, bodies with several anchors drawn
as a chain, springs, rollers and contact points, hands and dials, a hatched ground bar when the frame's anchors are
collinear and a small mark under each otherwise, and, since M14, **slots and slides whose rail rides a moving body**
(drawn from the rail's own anchor past the guide's reach).

The one difference is time. The SVG knows the whole motion in advance and draws a rail over all of it; a live view does
not, so the viewer keeps the *running* extent of every slide's travel (reset by **R**) and a rail grows as the motion
unfolds.

## How it is checked, and what that does not prove

The test suite covers the scene: every number in a rendered SVG is read back and compared with the solver, and the scene is
checked to be well formed for every example. That covers the geometry the viewer draws, because it is the same list.

What the suite cannot cover is the OpenGL end. For that, `scripts/viewer_shots.sh` runs the real `src/viewer.cpp` offscreen on a
small software rasterizer (`scripts/glfw_shim_softgl.cpp`) that implements the legacy-GL calls the viewer makes, dumps a
frame and converts it to PNG (`docs/img/viewer/`). Looking at those shots is how the viewer's missing M14 drawing, its
diagonal ground bar and its planet-as-ring were found and fixed.

That is evidence about the drawing code, not about your GPU:

* The rasterizer implements source-alpha blending, because the viewer asks for it (an earlier version ignored alpha, which made
  every translucent fill look solid; it was the tool that was wrong, not the viewer).
* It does not antialias, and lines wider than one pixel are drawn as squares of pixels. A real driver may render wide lines and
  antialiasing differently; the viewer's choices (rectangles for thick bars, annuli for thick rings) avoid relying on wide-line
  support.
* Nothing here tests the window system, input handling or frame timing.

If something looks wrong on your machine, a screenshot of the real window is the most useful report.

## Limits

* No labels, dimensions or force arrows (as in the SVG).
* The live view cannot show the future: a rail covers only the travel seen so far, and the camera fits the first pose
  (press **F** to refit).
