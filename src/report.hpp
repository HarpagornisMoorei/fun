// mech — report.hpp: what a mechanism has to carry (M20)
//
// `mech report` runs a dynamic mechanism and reduces every load it computes to a few numbers an engineer
// sizes parts with: for each rate drive the peak and RMS torque (the motor's peak and continuous rating), the
// peak speed and the peak and mean power; for each joint the peak and RMS force (the bearing's load, the
// slide's side load and moment); and the same for springs, contacts, racks and belts. Everything is the solver's
// own constraint multipliers, so the numbers are exact for the model: rigid bodies, ideal joints, no friction
// in the joints (the loads are the loads the constraints must supply).
#pragma once
#include "model.hpp"
#include <string>
#include <vector>

namespace mech {

struct ReportRow {
    std::string group;     // "drive", "joint", "slide", "spring", "contact", "rack", "belt"
    std::string name;
    std::string quantity;  // "torque", "force", "moment", "speed", "power", ...
    std::string unit;
    double peak = 0.0;     // largest |value|
    double peakAt = 0.0;   // time of it
    double rms = 0.0;
    double mean = 0.0;     // signed mean (for power this is the average power: negative gives energy back)
};

struct Report {
    bool ok = true;
    std::string error;
    double seconds = 0.0;
    std::vector<ReportRow> rows;
};

// Run `m` for `seconds` and reduce its loads. Fails (ok false, error set) for a kinematic mechanism, which has no
// masses to carry, or one that does not assemble.
Report buildReport(const Model& m, double seconds);

} // namespace mech
