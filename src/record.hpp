// mech — record.hpp: time-series export of a simulation (M7)
//
// A Recorder turns a state (t, q, and for dynamic files qd) into one row of
// named columns, so a run can be exported as CSV and plotted or checked
// elsewhere. It knows nothing about the command line: the tool and the tests
// both drive it directly.
//
// Columns (SI throughout; the unit is part of the name, e.g. `cam.theta[rad]`):
//   t[s]
//   <body>.theta[rad] .x[m] .y[m]           every moving body's pose
//   <body>.<anchor>.x[m] .y[m]              each probed anchor, in the world
//   maxRes                                  the largest constraint residual
//   <body>.<anchor>@<profile>.gap[m]        each touch: signed gap less roller
//                          .foot.x[m] .foot.y[m] .normal[rad]
//   dynamic files, in addition:
//   <body>.omega[rad/s] .vx[m/s] .vy[m/s]   velocities
//   <body>.<anchor>@<profile>.N[N] .engaged contact force; 1 while on the curve
//   drive.<body>[N*m]                       reaction torque of each rate drive
//   KE[J] PE[J] E[J]
//   ledger.weight[J] .impulse[J] .drive[J] .impact[J] .friction[J]
//   ledger.residual[J]    (E - E0) - (weight + impulse + drive - impact - friction)
#pragma once
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "model.hpp"
#include <string>
#include <utility>
#include <vector>

namespace mech {

struct Probe {
    int body = -1, anchor = -1;
};

class Recorder {
public:
    // `dyn` is null for a kinematic run.
    Recorder(const Model& m, const DynamicsSystem* dyn, std::vector<Probe> probes);

    // Mechanical energy at the start of the run; the ledger residual is measured from it.
    void setBaseline(double E0) { E0_ = E0; }

    const std::vector<std::string>& columns() const { return cols_; }

    // One row. qd is ignored (may be empty) for a kinematic run.
    std::vector<double> sample(double t, const std::vector<double>& q,
                               const std::vector<double>& qd, double maxRes) const;

    std::string headerLine() const;
    static std::string csvLine(const std::vector<double>& row);

private:
    const Model& m_;
    const DynamicsSystem* dyn_;
    std::vector<Probe> probes_;
    std::vector<std::string> cols_;
    double E0_ = 0.0;
};

} // namespace mech
