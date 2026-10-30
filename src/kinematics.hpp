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
class ConstraintSystem {
public:
    explicit ConstraintSystem(const Model& m) : m(m), nq(m.nq()) {}

    int rowCount() const;

    void residuals(const std::vector<double>& q, double t,
                   std::vector<double>& c) const;

    // Row-major [rows x nq] analytic Jacobian.
    void jacobian(const std::vector<double>& q, std::vector<double>& J) const;

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
