// mech — diagnose.hpp: is the pose a regular one? (M22)
//
// A mechanism can assemble ("converged, max residual 1e-16") and still sit somewhere it should not: on a dead point, on a
// folded branch where two rods lie on top of each other, or at the flat pose where a linkage branches. The solver cannot
// tell, because every one of those poses satisfies the constraints. The Jacobian can. At a regular pose the rows of J (one per
// active constraint) are independent, and the nearer a pose is to a singular one, the more nearly some row is a combination of
// the others.
//
// The measure is a *separation angle*. Take the rows in a fixed order; the angle between row i and the span of the rows before
// it is how far that constraint is from being implied by the others. It is 90 degrees for rows that have nothing to do with each
// other, and 0 for a dependent one. An angle does not depend on how long a row is, so the units a constraint is written in do not
// matter; the unknowns are measured as displacements: a rotation of a body that reaches l metres from its origin moves its far end
// by l times the angle, so its column is weighted by 1/l (see unknownWeights). The numbers then mean the same thing for a clock and
// for a crane: the same mechanism drawn ten times bigger has the same separation angles.
//
//   dependent     separation below kDependentDeg: the row is a combination of the rows before it (a rank deficiency)
//   near-singular separation below kNearDeg: the pose is within that angle of a singular one
//
// Nothing here changes what the solvers compute: the monitors read the Jacobian at poses the solvers already found.
#pragma once
#include "kinematics.hpp"
#include "model.hpp"

#include <string>
#include <utility>
#include <vector>

namespace mech {

// sin^2 of the separation angle: a row is dependent below this ...
constexpr double kDependentRatio = 1e-9;
// ... and a pose is near-singular when some row is closer than this (asin(sqrt(1e-4)) = 0.57 degrees)
constexpr double kNearRatio = 1e-4;
// A constraint residual above this means the solver did not meet the constraints (it converges to 1e-10).
constexpr double kLostResidual = 1e-6;

inline double ratioToDegrees(double ratio) {
    const double r = ratio < 0.0 ? 0.0 : (ratio > 1.0 ? 1.0 : ratio);
    return std::asin(std::sqrt(r)) * 180.0 / 3.14159265358979323846;
}

struct JacobianHealth {
    int rows = 0;                  // active constraint rows
    int unknowns = 0;              // columns that some row touches (the others cannot be told apart from rest)
    int rank = 0;                  // rows - dependent.size()
    std::vector<int> dependent;    // rows (in ConstraintSystem order) found to be combinations of the rows before them
    double minRatio = 1.0;         // smallest sin^2 of the separation angle among the independent rows
    int minRow = -1;               // the row that has it (-1 when there are no rows)
    std::vector<double> ratios;    // sin^2 of the separation of every row, in the order of J (0 for a dependent row)
    double minDegrees() const { return ratioToDegrees(minRatio); }
};

// How each unknown is weighed so that all are displacements: 1 for a position, 1/l for the angle of a body that reaches l from its
// origin. l is the farthest anchor, gear or profile the body carries, or the farthest a row of `J` (when given) actually reaches at
// this pose: a bead that has slid 27 m along a tube turns that tube's angle into 27 m of motion, whatever the tube was drawn with.
// A body that reaches nowhere is taken to reach as far as the largest one. A small part is not given a larger reach to make it
// count for more: a 1 mm gear in a 1.5 m mechanism is as regular as the same train drawn big.
std::vector<double> unknownWeights(const Model& m, const std::vector<SparseRow>* J = nullptr);

// The health of a Jacobian given as sparse rows (ConstraintSystem::jacobianSparse). `nq` is the number of columns and `weight`, when
// given, one factor per column (unknownWeights; without it the columns are taken as they are). Sparse all through: the cost grows
// with the size of the mechanism the way a solve does, not with its cube.
JacobianHealth jacobianHealth(const std::vector<SparseRow>& J, int nq, const std::vector<double>* weight = nullptr);

// What each constraint row is, in ConstraintSystem order ("pin coupler.B -> rocker.B (y)", "drive crank", ...): the names
// a message needs. One label per row of `sys.rowCount()`.
std::vector<std::string> constraintLabels(const Model& m);

// Pairs of moving bodies lying in exactly the same place (same origin, same angle): two copies of one chain on top of each
// other. `tol` is relative to the size of the mechanism.
std::vector<std::pair<int, int>> coincidentBodies(const Model& m, const std::vector<double>& q, double tol = 1e-7);

// ---- the assembled pose -------------------------------------------------------------------------------------------

struct PoseDiagnosis {
    JacobianHealth health;
    std::vector<std::pair<int, int>> coincident;   // bodies in the same place; looked for only when something is dependent
    int explained = 0;      // dependent rows the count alone forces: more rows than unknowns
    int unexplained = 0;    // dependent rows beyond that
};

PoseDiagnosis diagnosePose(const Model& m, const ConstraintSystem& sys, const std::vector<double>& q);

// Warn: something is wrong and there is evidence (bodies on top of each other). Note: the Jacobian says the pose is special, which
// can be a mistake (a dead point) or the design (parallel cranks); the rank alone cannot say which.
enum class Severity { Ok, Note, Warn };

struct Finding {
    Severity severity;
    std::string headline;               // one line, no tag and no trailing newline
    std::vector<std::string> details;   // further lines: what it means and what to do
};

// What to tell the author about an assembled pose. Empty when there is nothing to say.
std::vector<Finding> describePose(const Model& m, const PoseDiagnosis& d);

// ---- along a run ------------------------------------------------------------------------------------------------------

// A stretch of a run during which the pose was near-singular, or the constraints had become dependent.
struct Excursion {
    double t0 = 0.0, t1 = 0.0;        // first and last sampled time inside it
    double worstT = 0.0;              // when the constraints were closest to dependent
    double worstRatio = 1.0;          // sin^2 of the separation then (0 when rows had become dependent)
    int dependent = 0;                // most dependent rows beyond those the run started with
    int rows = 0, rank = 0;           // at the worst sample
    std::string involving;            // one constraint that was closest to dependent, by name
    double maxResidual = -1.0;        // largest constraint residual the run reported inside it (-1: not reported)
    long samples = 0;
    double worstDegrees() const { return ratioToDegrees(worstRatio); }
};

// Watches a run for singular configurations: call observe() with each pose the run reaches (every step of a kinematic run or an
// integration). It reads the Jacobian through the run's own ConstraintSystem and evaluates nothing else, so it cannot disturb the
// solvers' state: a run with a monitor produces the same numbers as one without.
//
// What it can promise is what it samples: a singular configuration that the run steps over without a sample landing within
// kNearRatio of it goes unseen. closestDegrees() says how near the samples did come.
class RunMonitor {
public:
    explicit RunMonitor(const ConstraintSystem& sys) : sys_(sys) {}

    // The first call fixes what is normal: a mechanism that is over-constrained on purpose (consistent redundancy) has dependent
    // rows all along, and only *more* of them is news. The baseline is taken again whenever the number of rows changes (a mode
    // hold, a contact engaging).
    // `residual` is the largest constraint residual the run found at this pose, when it has computed it (the monitor never
    // evaluates residuals itself: that would touch the state the solvers track from step to step). A large one inside an excursion
    // says the mechanism has run out of range, not merely met a branch point.
    void observe(const std::vector<double>& q, double t, double residual = -1.0);
    // Close an excursion still open at the end of the run.
    void finish();

    const std::vector<Excursion>& excursions() const { return done_; }
    long samples() const { return samples_; }
    double closestRatio() const { return closest_; }
    double closestDegrees() const { return ratioToDegrees(closest_); }
    double closestAt() const { return closestT_; }

    // The lines to print at the end of a run (none when no excursion happened). At most `maxShown` excursions are described.
    std::vector<std::string> report(int maxShown = 5) const;

    // For a run someone is watching (the window): the lines for the excursions that have begun since the last call, each told once,
    // as it begins; and a further line when an excursion turns out to be the end of the mechanism's range (the constraints can
    // no longer be met), which is only known a few steps in.
    std::vector<std::string> startedSince();
    // Forget the run and start again (the window's reset key).
    void reset();

private:
    static std::string describe(const Excursion& e);

    const ConstraintSystem& sys_;
    size_t announced_ = 0;
    bool lostTold_ = false;
    long samples_ = 0;
    bool haveBase_ = false;
    int baseRows_ = 0, baseDependent_ = 0;
    double closest_ = 1.0, closestT_ = 0.0;
    bool open_ = false;
    Excursion cur_;
    std::vector<Excursion> done_;
};

} // namespace mech
