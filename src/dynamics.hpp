// mech — dynamics.hpp: constrained rigid-body dynamics (M3) + hybrid events (M5)
//
// Kinematic mechanisms (rate drives only) are solved: the constraint system
// plus drive rows determine the configuration at every t. Dynamic mechanisms
// (a `world` block, springs or torque/weight drives) are *integrated*: masses,
// gravity, spring forces and torques accelerate the mechanism, constraints
// hold via Lagrange multipliers, and rate drives remain exact prescribed
// coordinates whose reaction torques come out of the same solve.
//
// M5: when the mechanism carries `mode` elements, the integrator becomes a
// hybrid one. Guards are evaluated at step boundaries; a rising crossing is
// localized by bisection (re-integrating sub-steps), the transition's resets
// are applied at the crossing instant, the new state's holds are captured,
// and integration resumes to the end of the step. Weight drives pay out cord
// and energy; the whole run is bookkept in a ledger (drawn / impulse /
// impact loss) that closes against the change in mechanical energy.
#pragma once
#include "kinematics.hpp"
#include <vector>

namespace mech {

// M5 — what the hybrid run did, for reporting and the conservation test.
struct HybridStats {
    long fires = 0;               // transitions fired (all modes together)
    double weightDrawn = 0.0;     // J paid out by weight drives
    double impulseEnergy = 0.0;   // J delivered by impulse resets (net)
    double impactLoss = 0.0;      // J discarded by event relock projections
                                  // and by contact impacts (M6c)
    long liftoffs = 0;            // M6c: contacts that separated
    long impacts = 0;             // M6c: contacts that touched down again
    double frictionLoss = 0.0;    // M6f: J dissipated by contact friction (integral of F_t |slip|)
    double driveWork = 0.0;       // M6c: J delivered by rate drives (reaction x rate) and
                                  // torque drives (torque x angular velocity)
};

class DynamicsSystem {
public:
    DynamicsSystem(const Model& model, const ConstraintSystem& sys);

    // One fixed step from time t to t + world.step. q and qd are updated in
    // place; mode transitions that fire inside the step are applied at their
    // localized crossing times. Deterministic: same inputs, same outputs.
    void step(std::vector<double>& q, std::vector<double>& qd, double t) const;

    // Constrained accelerations at a state: solves
    //   M qdd = Qeff + J^T lambda,  J qdd = -bias
    // via the Schur complement. Exposed for testing (the finite-difference
    // constraint-acceleration check validates every bias term).
    void accelerations(const std::vector<double>& q, const std::vector<double>& qd,
                       std::vector<double>& qdd) const;

    // Mass-metric velocity projection onto the active constraint rows
    // (J qd = nu, nu = drive rates): the same projection step() uses after
    // every step and at events. Public so tests can build consistent states.
    // If `impulseWork` is given it receives the work (J) the projection's
    // impulses do on prescribed (rate-drive) coordinates: the cam is
    // "infinitely heavy", so an impact on a driven body is paid for by its drive.
    void projectVelocities(const std::vector<double>& q, std::vector<double>& qd,
                           double* impulseWork = nullptr) const;

    // Energy bookkeeping (J). kinetic uses the exact mass matrix;
    // potential covers gravity and spring stretch. The weight drives' energy
    // is external — it appears in the hybrid ledger, not here.
    double kinetic(const std::vector<double>& q, const std::vector<double>& qd) const;
    double potential(const std::vector<double>& q) const;

    // Reaction multiplier of each rate-drive row from the last step
    // (N*m; the torque the prescribed coordinate transmits into the body).
    // Disabled mode holds report 0.
    const std::vector<double>& driveReactions() const { return lastReaction_; }

    // M14: the force (N, along the rail) each linear rate drive transmits into its guide; 0 for a force drive.
    const std::vector<double>& linearReactions() const { return lastLinReaction_; }

    // M6c: the normal force (N) of each touch from the last acceleration
    // solve; 0 for a contact that is not engaged.
    const std::vector<double>& contactForces() const { return lastContact_; }

    // M5: the hybrid ledger (energy bookkeeping of discrete events).
    const HybridStats& hybridStats() const { return stats_; }
    void resetHybrid() const { stats_ = HybridStats{}; }

    const Model& m;
    const ConstraintSystem& sys;

private:
    // per-body scratch for the mass matrix at the current theta
    struct MassBlock {
        double a, b, c, d;   // [[a,b,c],[b,d,0],[c,0,d]] symmetric
    };
    void massBlocks(const std::vector<double>& q, std::vector<MassBlock>& out) const;
    void applyW(const std::vector<MassBlock>& W, int bodyIdx,
                double v0, double v1, double v2, double* out) const;

    // --- M5 hybrid machinery -------------------------------------------------
    // One smooth RK4 step of length h (no event handling inside).
    void integrate(std::vector<double>& q, std::vector<double>& qd,
                   double t, double h) const;
    // Step with event handling; also runs the weight-drive ledger.
    void hybridStep(std::vector<double>& q, std::vector<double>& qd,
                    double t, double h) const;
    // Guard value g(body.field) - threshold; a transition fires when g
    // crosses zero upward (for '<' guards the value is negated).
    double guardValue(const Mode& mo, const ModeTransition& tr,
                      const std::vector<double>& q, const std::vector<double>& qd) const;
    // Switch to state idx: capture advance origins and hold targets.
    void enterState(const Mode& mo, int idx, const std::vector<double>& q) const;
    // Apply a transition at the localized crossing state.
    void applyTransition(const Mode& mo, const ModeTransition& tr,
                         std::vector<double>& q, std::vector<double>& qd,
                         double tNow) const;
    // Energy reset: fixed joules, or 'maintain' the stated half-swing about
    // the body's natural hang angle.
    void applyImpulse(const ModeReset& rs, const std::vector<double>& q,
                      std::vector<double>& qd) const;
    // Weight drives: pay cord and draw energy for |dtheta| of the hosts.
    void accountWeights(const std::vector<double>& qStart,
                        const std::vector<double>& qEnd) const;

    // --- M6c unilateral contact ---------------------------------------------
    // Contact force of touch ti at a state (solves the KKT system once).
    double contactForce(int ti, const std::vector<double>& q,
                        const std::vector<double>& qd) const;
    // Level-triggered: disengage any engaged contact that would have to pull.
    void settleContacts(std::vector<double>& q, std::vector<double>& qd, double tNow) const;
    // Touch-down: re-anchor, then Newton impact with restitution e.
    void applyImpact(int ti, std::vector<double>& q, std::vector<double>& qd,
                     double tNow) const;

    mutable std::vector<double> lastReaction_;
    mutable std::vector<double> lastLinReaction_;
    mutable std::vector<double> lastContact_;
    mutable double lastDriveWork_ = 0.0;   // J of the last integrate() call
    mutable double lastFrictionLoss_ = 0.0; // J dissipated by friction in the last integrate()
    mutable double lastFrictionPower_ = 0.0; // W dissipated at the last acceleration solve
    mutable HybridStats stats_;
};

} // namespace mech
