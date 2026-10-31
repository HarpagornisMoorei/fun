// mech — kinematics.hpp: constraint-based kinematic solver
#pragma once
#include "model.hpp"
#include <vector>

namespace mech {

// The whole mechanism is one algebraic system:
//   pin rows:    world(A,a) - world(B,b)        = 0   (2 rows per pin)
//   drive rows:  theta_body - (theta0 + w*t)    = 0   (1 row per drive)
// Unknowns q = (theta, x, y) per non-ground body.
// Time t appears only inside drive targets, so motion is whatever the
// constraint manifold allows — drives are not assignments.
// M14: where a slide's guide point and rail line are. The rail is a ground constant, or rides a body.
struct SlideGeom {
    Vec2 pA, pB;       // guide point and rail anchor (world)
    Vec2 u;            // rail direction (world, unit)
    Vec2 rA, rB;       // R * local anchors: offsets from the bodies' origins (rB zero for a ground rail)
    Vec2 dirG;         // guide direction (world), only meaningful when the direction is locked
    int oA = -1, oB = -1;   // qOffset of the guide body and of the rail body (-1: ground)
};

SlideGeom slideGeom(const Model& m, const SlideJoint& s, const std::vector<double>& q);

class ConstraintSystem {
public:
    explicit ConstraintSystem(const Model& m) : m(m), nq(m.nq()) {}

    int rowCount() const;
    // M14: rows before the first drive row (pins, then one or two rows per slide), and the slide rows alone
    int slideRowCount() const;
    int driveRowStart() const { return 2 * (int)m.pins.size() + slideRowCount(); }
    // M14: the first of the linear-rate-drive rows, which come last of all
    int linearRowStart() const;

    void residuals(const std::vector<double>& q, double t,
                   std::vector<double>& c) const;

    // Row-major [rows x nq] analytic Jacobian.
    void jacobian(const std::vector<double>& q, std::vector<double>& J) const;

    // M6: the Jacobian row of one touch (d gap / d q), added into `row`
    // (nq entries, caller zeroes). Used by jacobian() and by impact handling.
    void touchRow(const TouchJoint& tj, const std::vector<double>& q, double* row) const;

    // Levenberg-Marquardt with deterministic multi-start. Warm-start
    // friendly: pass the previous solution. Returns true when max|c| < 1e-10
    // (or < 1e-8 after exhaustion).
    bool solve(std::vector<double>& q, double t, int& itersOut,
               double* maxResidualOut = nullptr) const;

    // Single LM run from the given guess.
    bool solveAttempt(std::vector<double>& q, double t, int& itersOut,
                      double* maxResidualOut = nullptr) const;

    double maxResidual(const std::vector<double>& q, double t) const;

    const Model& m;
    int nq;
};

} // namespace mech
