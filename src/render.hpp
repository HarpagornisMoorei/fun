// mech — render.hpp: SVG drawings of a mechanism (M13)
//
//   mech render examples/camstar.mech --at 1.2 -o camstar.svg            one pose
//   mech render examples/fourbar.mech --frames 48 --span 10 -o fourbar.svg   an animation that loops
//
// The drawing is what the viewer shows (ground, gears, profiles, rods, springs, contacts,
// pins, hands, dials, a trace) without needing a window or OpenGL. Everything is emitted in
// *world* coordinates — metres, y up — inside one root transform, so the numbers in the file are
// the solver's numbers and a test can read them back; sizes that are pixels on screen (line widths,
// joint hubs) are converted with the drawing's scale.
#pragma once
#include "model.hpp"
#include <string>
#include <vector>

namespace mech {

struct RenderOptions {
    double at = 0.0;           // time of the still, or of the first frame (s)
    int frames = 1;            // > 1: an animated SVG that loops
    double span = 0.0;         // simulated seconds the animation covers (frames are spread over it),
                               // and, in a still, how far the trace runs from `at`
    double speed = 1.0;        // animation playback: simulated seconds per second
    double width = 800.0;      // drawing width in pixels
    std::string trace;         // "body.anchor": its path over [at, at + span]
    bool dark = false;         // the viewer's dark theme instead of a white page
};

struct RenderResult {
    bool ok = true;
    std::string svg;
    std::string error;
    int frames = 0;
};

// Poses of the model (q vectors) at the given ascending times: kinematic models are re-solved in
// steps of at most 1/60 s from the assembly (so the branch is followed), dynamic ones integrated.
// Empty result and `error` set if the model does not assemble.
std::vector<std::vector<double>> samplePoses(const Model& m, const std::vector<double>& times,
                                             std::string& error);

RenderResult renderSvg(const Model& m, const RenderOptions& opt);

} // namespace mech
