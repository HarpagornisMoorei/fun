// mech — viewer.hpp: animated top-view OpenGL window
#pragma once
#include "model.hpp"
#include <string>

namespace mech {
namespace viewer {

// traceRef: "" or "body.anchor" — draws the traced point's path (e.g. the
// coupler curve of a four-bar). Returns process exit code.
int run(const Model& m, double speedMul, bool startPaused, const std::string& traceRef);

} // namespace viewer
} // namespace mech
