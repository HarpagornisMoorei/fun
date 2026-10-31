// mech — sweep.hpp: design exploration over parameters (M8)
//
// `mech sweep` runs one file at many parameter values and tabulates what happened:
//
//   mech sweep jumpcam.mech --set rpm=400:900:50 --time 0.3 --report liftoffs --report max:follower.y
//
// A design that stops being valid partway through a sweep (a cam that starts to
// undercut, a linkage that cannot assemble) reports its error in its own row; the
// sweep carries on, because *where* a design breaks is usually the answer.
#pragma once
#include "dynamics.hpp"
#include "model.hpp"
#include "parser.hpp"
#include "record.hpp"
#include <string>
#include <vector>

namespace mech {

// One simulated run, recorded. Column names follow record.hpp.
struct SimSeries {
    std::vector<std::string> columns;
    std::vector<std::vector<double>> rows;
    HybridStats stats;
    bool dynamic = false;
    bool ok = true;
    std::string error;     // why it could not run (assembly, geometry)
};

// Assemble and run a model: a kinematic file for `seconds` at `dt` per frame, a dynamic
// one at its world step. Records every `every`-th step (the t = 0 row always).
SimSeries simulate(const Model& m, double seconds, double dt, const std::vector<Probe>& probes,
                   long every = 1);

// What to measure from a series. `spec` is one of
//   max:<col> min:<col> mean:<col> final:<col>   a recorded column (the [unit] may be omitted)
//   maxres                                         the largest constraint residual
//   liftoffs impacts                               contact events (dynamic files)
//   ledger                                         the energy ledger's worst |residual| (dynamic)
struct Metric {
    enum class Kind { Max, Min, Mean, Final, MaxRes, Liftoffs, Impacts, Ledger };
    Kind kind = Kind::Max;
    int col = -1;
    std::string label;
};
bool parseMetric(const std::string& spec, const std::vector<std::string>& columns, bool dynamic,
                 Metric& out, std::string& err);
double evalMetric(const Metric& m, const SimSeries& s);

// A parameter axis: "name=v", "name=v1,v2,...", or "name=a:b:step" (b inclusive).
struct SweepAxis {
    std::string name;
    std::vector<double> values;
};
bool parseAxis(const std::string& spec, SweepAxis& out, std::string& err);

struct SweepRow {
    std::vector<double> params;     // one per axis
    std::vector<double> metrics;    // one per metric
    std::string error;              // non-empty: this design could not be built or run
};
struct SweepResult {
    std::vector<std::string> axisNames, metricLabels;
    std::vector<SweepRow> rows;
    bool dynamic = false;
};

struct SweepOptions {
    double seconds = 2.0;
    double dt = 1.0 / 60.0;
    std::vector<std::string> reports;   // empty: a sensible default for the file
    std::vector<std::string> probes;
    std::string path;                   // where the source came from: anchors relative `import`s
};

// Run the sweep over the full grid of axes (first axis slowest). Throws std::runtime_error
// for problems that make the whole sweep meaningless (the unmodified file does not build,
// an unknown metric, a grid over 5000 runs).
SweepResult runSweep(const std::string& source, const std::vector<SweepAxis>& axes,
                     const SweepOptions& opt);

} // namespace mech
