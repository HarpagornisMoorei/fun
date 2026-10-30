// mech — dynamics.cpp: constrained rigid-body dynamics (M3)
//
// The mechanism obeys the Lagrangian equations of motion
//
//   M(q) qdd = Q(q, qd, t) + gyro(q, qd) + J(q)^T lambda     (1)
//   c(q, t) = 0                                              (2)
//
// with q = (theta, x, y) per non-ground body and the constraint Jacobian J
// shared with the kinematic solver. Each body's mass matrix block, for a
// center of mass at body-frame offset com (Rc = R(theta) com, p = perp(Rc)):
//
//   [[ I + m|com|^2 ,  m p_x ,  m p_y ],
//    [  m p_x       ,  m     ,  0     ],
//    [  m p_y       ,  0     ,  m     ]]
//
// which is symmetric positive definite (det = m^2 I_com). The gyroscopic
// term is (0, m w^2 Rc_x, m w^2 Rc_y); gravity's generalized force is
// (-m g Rc_x, 0, -m g) plus, for every spring, the anchor forces/torques of
// a Hooke (and optional viscous) element; torque drives add straight to the
// theta column — they are pure energy injection, no constraint row.
//
// Differentiating (2) twice gives the acceleration-level system
//
//   J qdd = -bias,      bias = Jdot qdot      (analytic per row type)
//
// Remarkably, for every row type in the kernel (pins, slides, drive rows,
// mesh roll rows with moving centers, tangency rows) the acceleration-level
// Jacobian is exactly the position-level Jacobian — the qdd-dependence
// enters through the same linear map as the qd-dependence — so the KKT
// system reuses the kinematic J wholesale and only the bias vector differs.
// Eliminating qdd via the Schur complement (W = M^-1, blockwise):
//
//   (J W J^T) lambda = -(J W (Q + gyro) + bias)
//   qdd = W ((Q + gyro) + J^T lambda)
//
// Integration is semi-implicit (symplectic) Euler at the fixed world step,
// followed by a least-norm velocity projection (J qd = nu, nu = drive rates)
// and the LM position solver as post-stabilization. Rate drives therefore
// stay exactly on schedule while their reaction torques are the
// corresponding lambda entries. Everything is deterministic.

#include "dynamics.hpp"
#include "linalg.hpp"

#include <algorithm>
#include <cmath>

namespace mech {

namespace {

// Solve A x = b for symmetric A with growing Tikhonov damping until the
// Cholesky succeeds. Returns false only if even brutal damping collapses.
bool dampedSolve(std::vector<double> A, const std::vector<double>& b, int n,
                 std::vector<double>& x) {
    double eps = 1e-10;
    double maxDiag = 1e-12;
    for (int i = 0; i < n; ++i)
        maxDiag = std::max(maxDiag, A[(size_t)i * n + i]);
    eps *= 1.0 + maxDiag;
    for (int attempt = 0; attempt < 6; ++attempt) {
        std::vector<double> Ad = A;
        for (int i = 0; i < n; ++i) Ad[(size_t)i * n + i] += eps;
        if (choleskySolve(Ad, b, n, x)) return true;
        eps *= 100.0;
    }
    x.assign(n, 0.0);
    return false;
}

inline double dot3(const double* a, const double* b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

} // namespace

DynamicsSystem::DynamicsSystem(const Model& model, const ConstraintSystem& sys)
    : m(model), sys(sys) {}

void DynamicsSystem::massBlocks(const std::vector<double>& q,
                                std::vector<MassBlock>& out) const {
    out.assign(m.bodies.size(), MassBlock{1.0, 0.0, 0.0, 1.0});
    for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
        const MassProps& mp = m.mass[bi];
        const double th = q[m.bodies[bi].qOffset + 0];
        const Vec2 Rc = rotate(mp.com, th);
        const Vec2 p = perp(Rc);
        MassBlock B;
        B.a = mp.inertia + mp.mass * dot(mp.com, mp.com);
        B.b = mp.mass * p.x;
        B.c = mp.mass * p.y;
        B.d = mp.mass;
        out[bi] = B;
    }
}

void DynamicsSystem::applyW(const std::vector<MassBlock>& W, int bodyIdx,
                            double v0, double v1, double v2, double* out) const {
    const MassBlock& M = W[bodyIdx];
    const double det = M.d * (M.a * M.d - M.b * M.b - M.c * M.c);
    if (!(det > 1e-18)) { out[0] = v0; out[1] = v1; out[2] = v2; return; }
    const double id = 1.0 / det;
    out[0] = id * ( M.d * M.d * v0 - M.b * M.d * v1 - M.c * M.d * v2);
    out[1] = id * (-M.b * M.d * v0 + (M.a * M.d - M.c * M.c) * v1 + M.b * M.c * v2);
    out[2] = id * (-M.c * M.d * v0 + M.b * M.c * v1 + (M.a * M.d - M.b * M.b) * v2);
}

void DynamicsSystem::accelerations(const std::vector<double>& q,
                                   const std::vector<double>& qd,
                                   std::vector<double>& qdd) const {
    const int nq = m.nq();
    const int rows = sys.rowCount();
    qdd.assign(nq, 0.0);
    if (nq == 0) return;

    // --- per-body scratch ----------------------------------------------------
    auto bodyW   = [&](int bi) { return m.bodies[bi].qOffset >= 0 ? qd[m.bodies[bi].qOffset + 0] : 0.0; };
    auto bodyVel = [&](int bi) -> Vec2 {
        return m.bodies[bi].qOffset >= 0
            ? Vec2{qd[m.bodies[bi].qOffset + 1], qd[m.bodies[bi].qOffset + 2]}
            : Vec2{0.0, 0.0};
    };
    // rotated anchor offset from the body origin (ground: zero motion)
    auto anchorOff = [&](int bi, int ai) {
        return m.bodies[bi].qOffset >= 0
            ? rotate(m.bodies[bi].anchors[ai].local, q[m.bodies[bi].qOffset + 0])
            : m.bodies[bi].anchors[ai].local;
    };
    auto anchorVel = [&](int bi, int ai) -> Vec2 {
        if (m.bodies[bi].qOffset < 0) return {0.0, 0.0};
        const Vec2 r = anchorOff(bi, ai);
        const Vec2 v = bodyVel(bi);
        return {v.x - bodyW(bi) * r.y, v.y + bodyW(bi) * r.x};
    };

    // --- forces Q (generalized) + gyro ---------------------------------------
    std::vector<double> Q(nq, 0.0);
    const double g = m.world.gravity;
    if (g != 0.0) {
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const MassProps& mp = m.mass[bi];
            const int o = m.bodies[bi].qOffset;
            const Vec2 Rc = rotate(mp.com, q[o + 0]);
            Q[o + 0] += -mp.mass * g * Rc.x;   // gravity torque about the origin
            Q[o + 2] += -mp.mass * g;          // gravity force along -y
        }
    }
    for (const TorqueDrive& td : m.torques) {
        Q[m.bodies[td.body].qOffset + 0] += td.torque;
    }
    // M5: weight drives apply their torque (in the cord's unwind sense)
    // until the cord runs out
    for (const WeightDrive& w : m.weights) {
        if (w.exhausted) continue;
        Q[m.bodies[w.body].qOffset + 0] += (double)w.sign * w.torque;
    }
    for (const SpringJoint& sp : m.springs) {
        const Vec2 pa = m.worldAnchor(sp.bodyA, sp.anchA, q);
        const Vec2 pb = m.worldAnchor(sp.bodyB, sp.anchB, q);
        const Vec2 va = anchorVel(sp.bodyA, sp.anchA);
        const Vec2 vb = anchorVel(sp.bodyB, sp.anchB);
        const Vec2 d = pb - pa;
        const double L = vlen(d);
        if (L < 1e-9) continue;
        const Vec2 u{d.x / L, d.y / L};
        const double ext = L - sp.rest;             // > 0: stretched
        const double rate = dot(vb - va, u);        // separation speed
        // element tension (> 0 pulls the two anchors together)
        const double tension = sp.k * ext + sp.damp * rate;
        const Vec2 F{tension * u.x, tension * u.y}; // force on body A (toward B)
        const Vec2 rA = anchorOff(sp.bodyA, sp.anchA);
        const Vec2 rB = anchorOff(sp.bodyB, sp.anchB);
        if (m.bodies[sp.bodyA].qOffset >= 0) {
            const int o = m.bodies[sp.bodyA].qOffset;
            Q[o + 0] += crossz(rA, F);
            Q[o + 1] += F.x;
            Q[o + 2] += F.y;
        }
        if (m.bodies[sp.bodyB].qOffset >= 0) {
            const int o = m.bodies[sp.bodyB].qOffset;
            Q[o + 0] += crossz(rB, -F);
            Q[o + 1] += -F.x;
            Q[o + 2] += -F.y;
        }
    }
    // gyroscopic term: (0, m w^2 Rc_x, m w^2 Rc_y)
    std::vector<double> Qeff = Q;
    for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
        const MassProps& mp = m.mass[bi];
        const int o = m.bodies[bi].qOffset;
        const Vec2 Rc = rotate(mp.com, q[o + 0]);
        const double w2 = qd[o + 0] * qd[o + 0];
        Qeff[o + 1] += mp.mass * w2 * Rc.x;
        Qeff[o + 2] += mp.mass * w2 * Rc.y;
    }

    // --- constraint rows: J, residuals (for slide dirlock), bias --------------
    std::vector<double> c;
    sys.residuals(q, 0.0, c);   // slide dirlock values; also keeps psi tracking warm
    std::vector<double> J;
    sys.jacobian(q, J);
    std::vector<double> bias(rows, 0.0);

    int r = 0;
    for (const PinJoint& p : m.pins) {
        for (int side = 0; side < 2; ++side) {
            const int bi = side == 0 ? p.bodyA : p.bodyB;
            const int ai = side == 0 ? p.anchA : p.anchB;
            if (m.bodies[bi].qOffset < 0) continue;   // ground never accelerates
            const double w2 = qd[m.bodies[bi].qOffset + 0] * qd[m.bodies[bi].qOffset + 0];
            const Vec2 Ra = anchorOff(bi, ai);
            const double sgn = side == 0 ? -1.0 : 1.0;   // A: -w^2 R, B: +w^2 R
            bias[r]     += sgn * w2 * Ra.x;
            bias[r + 1] += sgn * w2 * Ra.y;
        }
        r += 2;
    }
    for (const SlideJoint& s : m.slides) {
        const int gi = s.guideBody;
        const double w2 = qd[m.bodies[gi].qOffset + 0] * qd[m.bodies[gi].qOffset + 0];
        const Vec2 Rg = anchorOff(gi, s.guideAnchor);
        bias[r] = -crossz(s.railDir, w2 * Rg);   // offset row
        bias[r + 1] = -w2 * c[r + 1];            // dirlock row: bias = -w^2 c2
        r += 2;
    }
    const int driveRow0 = r;
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;   // M5: disabled mode holds add no row
        ++r;                        // drive rows: bias 0
    }
    for (const MeshJoint& mm : m.meshes) {
        const Gear& ga = m.gears[mm.gearA];
        const Gear& gb = m.gears[mm.gearB];
        const Vec2 ca = m.worldAnchor(ga.body, ga.hostAnchor, q);
        const Vec2 cb = m.worldAnchor(gb.body, gb.hostAnchor, q);
        const Vec2 d = cb - ca;
        const double d2 = dot(d, d);
        const double dist = std::sqrt(d2);
        const double s = mm.sgn >= 0 ? 1.0 : -1.0;
        const double roll = ga.radius + s * gb.radius;

        // center velocities for psi-dot and d-dot
        const Vec2 va = anchorVel(ga.body, ga.hostAnchor);
        const Vec2 vb = anchorVel(gb.body, gb.hostAnchor);
        const Vec2 dv = vb - va;
        const double psiDot = dist > 1e-9 ? crossz(d, dv) / d2 : 0.0;
        const double dDot = dist > 1e-9 ? dot(d, dv) / dist : 0.0;

        // centripetal parts of the center accelerations
        Vec2 cwA{0.0, 0.0}, cwB{0.0, 0.0};
        if (m.bodies[ga.body].qOffset >= 0) {
            const double w2 = qd[m.bodies[ga.body].qOffset + 0] * qd[m.bodies[ga.body].qOffset + 0];
            cwA = w2 * anchorOff(ga.body, ga.hostAnchor);
        }
        if (m.bodies[gb.body].qOffset >= 0) {
            const double w2 = qd[m.bodies[gb.body].qOffset + 0] * qd[m.bodies[gb.body].qOffset + 0];
            cwB = w2 * anchorOff(gb.body, gb.hostAnchor);
        }

        if (dist > 1e-9) {
            const double K = roll / d2;
            bias[r] = K * crossz(d, cwB - cwA) + 2.0 * roll * psiDot * dDot / dist;
        }
        ++r;

        if (mm.tangent) {
            bias[r] = 2.0 * dot(dv, dv) + 2.0 * dot(d, cwA - cwB);
            ++r;
        }
    }

    // --- KKT via Schur complement --------------------------------------------
    std::vector<MassBlock> Mblocks;
    massBlocks(q, Mblocks);
    std::vector<std::vector<double>> BJ(rows, std::vector<double>(nq, 0.0));
    for (int j = 0; j < rows; ++j) {
        const double* Jj = &J[(size_t)j * nq];
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const int o = m.bodies[bi].qOffset;
            if (Jj[o] == 0.0 && Jj[o + 1] == 0.0 && Jj[o + 2] == 0.0) continue;
            double wv[3];
            applyW(Mblocks, (int)bi, Jj[o], Jj[o + 1], Jj[o + 2], wv);
            BJ[j][o] = wv[0];
            BJ[j][o + 1] = wv[1];
            BJ[j][o + 2] = wv[2];
        }
    }
    // z = W Qeff
    std::vector<double> z(nq, 0.0);
    for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
        const int o = m.bodies[bi].qOffset;
        double wv[3];
        applyW(Mblocks, (int)bi, Qeff[o], Qeff[o + 1], Qeff[o + 2], wv);
        z[o] = wv[0];
        z[o + 1] = wv[1];
        z[o + 2] = wv[2];
    }
    // A = J W J^T, rhs = -(J z + bias)
    std::vector<double> A((size_t)rows * rows, 0.0);
    std::vector<double> rhs(rows, 0.0);
    for (int j = 0; j < rows; ++j) {
        const double* Jj = &J[(size_t)j * nq];
        for (int k = j; k < rows; ++k) {
            double s = 0.0;
            for (int cix = 0; cix < nq; ++cix) s += Jj[cix] * BJ[k][cix];
            A[(size_t)j * rows + k] = s;
            A[(size_t)k * rows + j] = s;
        }
        double s = 0.0;
        for (int cix = 0; cix < nq; ++cix) s += Jj[cix] * z[cix];
        rhs[j] = -(s + bias[j]);
    }

    std::vector<double> lam;
    if (rows > 0) dampedSolve(A, rhs, rows, lam);

    // qdd = z + W J^T lam
    qdd = z;
    for (int j = 0; j < rows; ++j) {
        if (lam[j] == 0.0) continue;
        for (int cix = 0; cix < nq; ++cix) qdd[cix] += BJ[j][cix] * lam[j];
    }

    // reaction torques of the rate drives (N*m about each driven body;
    // a disabled mode hold reports 0)
    lastReaction_.assign(m.drives.size(), 0.0);
    {
        int rr = driveRow0;
        for (size_t di = 0; di < m.drives.size(); ++di) {
            if (!m.drives[di].enabled) continue;
            lastReaction_[di] = rr < (int)lam.size() ? lam[rr] : 0.0;
            ++rr;
        }
    }
}

// ---- integration -------------------------------------------------------------
//
// hybridStep() is the public step: it integrates [t, t+h] and applies any
// mode transitions that fire inside it. integrate() is one smooth RK4 step
// (the M3 kernel, unchanged); events sit entirely around it.

void DynamicsSystem::step(std::vector<double>& q, std::vector<double>& qd,
                          double t) const {
    hybridStep(q, qd, t, m.world.step);
}

void DynamicsSystem::integrate(std::vector<double>& q, std::vector<double>& qd,
                               double t, double h) const {
    const int nq = m.nq();

    // Classical RK4 on the pair (q, qd) with q' = qd, qd' = f(q, qd), where
    // f is the constrained acceleration from the KKT solve. The acceleration
    // is exact given a state (validated by the finite-difference constraint
    // test in the suite), so all remaining error is quadrature. Plain
    // semi-implicit Euler drifts at first order on this non-separable system
    // (the mass matrix depends on q); RK4 keeps chaotic mechanisms' energy
    // flat to well under a percent at the default 2 ms step.
    std::vector<double> a1, a2, a3, a4;
    accelerations(q, qd, a1);

    std::vector<double> qS(nq), qdS(nq);
    for (int i = 0; i < nq; ++i) {
        qS[i] = q[i] + 0.5 * h * qd[i];
        qdS[i] = qd[i] + 0.5 * h * a1[i];
    }
    accelerations(qS, qdS, a2);

    for (int i = 0; i < nq; ++i) {
        qS[i] = q[i] + 0.5 * h * qd[i] + 0.25 * h * h * a1[i];
        qdS[i] = qd[i] + 0.5 * h * a2[i];
    }
    accelerations(qS, qdS, a3);

    for (int i = 0; i < nq; ++i) {
        qS[i] = q[i] + h * qd[i] + 0.5 * h * h * a2[i];
        qdS[i] = qd[i] + h * a3[i];
    }
    accelerations(qS, qdS, a4);

    for (int i = 0; i < nq; ++i)
        qd[i] += h / 6.0 * (a1[i] + 2.0 * a2[i] + 2.0 * a3[i] + a4[i]);
    // position with the fresh velocity (semi-implicit form of the same RK4
    // polynomial): q + h*qd_new - h^2/6 (a2 + a3 + a4)
    for (int i = 0; i < nq; ++i)
        q[i] += h * qd[i] - h * h / 6.0 * (a2[i] + a3[i] + a4[i]);

    // --- post-stabilization: position, then velocity --------------------------
    int iters = 0;
    sys.solve(q, t + h, iters);
    projectVelocities(q, qd);
}

void DynamicsSystem::projectVelocities(const std::vector<double>& q,
                                        std::vector<double>& qd) const {
    const int nq = m.nq();
    const int rows = sys.rowCount();
    if (rows <= 0) return;

    std::vector<double> J2;
    sys.jacobian(q, J2);
    // nu: drive rows prescribe their rate; every other row wants J qd = 0
    const int driveRow0 = 2 * (int)m.pins.size() + 2 * (int)m.slides.size();
    std::vector<double> nu(rows, 0.0);
    {
        int r = driveRow0;
        for (const Drive& d : m.drives) {
            if (!d.enabled) continue;
            nu[r++] = d.omegaRad;
        }
    }

    // Velocity projection in the mass metric: the correction acts through the
    // same map as constraint forces (M^-1 J^T), so it does no work on the
    // constraint manifold — the Euclidean least-norm projection would inject
    // energy whenever joint rows couple several bodies' coordinates.
    std::vector<MassBlock> Mblocks;
    massBlocks(q, Mblocks);   // at the stabilized configuration
    std::vector<std::vector<double>> BJ2(rows, std::vector<double>(nq, 0.0));
    for (int j = 0; j < rows; ++j) {
        const double* Jj = &J2[(size_t)j * nq];
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const int o = m.bodies[bi].qOffset;
            if (Jj[o] == 0.0 && Jj[o + 1] == 0.0 && Jj[o + 2] == 0.0) continue;
            double wv[3];
            applyW(Mblocks, (int)bi, Jj[o], Jj[o + 1], Jj[o + 2], wv);
            BJ2[j][o] = wv[0];
            BJ2[j][o + 1] = wv[1];
            BJ2[j][o + 2] = wv[2];
        }
    }
    std::vector<double> A2((size_t)rows * rows, 0.0);
    std::vector<double> r2(rows, 0.0);
    for (int j = 0; j < rows; ++j) {
        const double* Jj = &J2[(size_t)j * nq];
        for (int k = j; k < rows; ++k) {
            double s = 0.0;
            for (int cix = 0; cix < nq; ++cix) s += Jj[cix] * BJ2[k][cix];
            A2[(size_t)j * rows + k] = s;
            A2[(size_t)k * rows + j] = s;
        }
        double s = 0.0;
        for (int cix = 0; cix < nq; ++cix) s += Jj[cix] * qd[cix];
        r2[j] = s - nu[j];
    }
    std::vector<double> mu;
    if (dampedSolve(A2, r2, rows, mu)) {
        for (int j = 0; j < rows; ++j) {
            if (mu[j] == 0.0) continue;
            for (int cix = 0; cix < nq; ++cix) qd[cix] -= BJ2[j][cix] * mu[j];
        }
    }
}

// ---- M5 hybrid machinery ------------------------------------------------------

double DynamicsSystem::guardValue(const Mode& mo, const ModeTransition& tr,
                                  const std::vector<double>& q,
                                  const std::vector<double>& qd) const {
    const Body& b = m.bodies[(size_t)tr.body];
    const int o = b.qOffset;
    double v = 0.0;
    switch (tr.field) {
        case ModeTransition::Field::Theta:
            v = q[(size_t)o + 0];
            break;
        case ModeTransition::Field::Omega:
            v = qd[(size_t)o + 0];
            break;
        case ModeTransition::Field::Advance: {
            // angle travelled since the current state was entered
            int slot = -1;
            for (size_t gi = 0; gi < mo.guardBodies.size(); ++gi)
                if (mo.guardBodies[gi] == tr.body) { slot = (int)gi; break; }
            v = slot >= 0 ? q[(size_t)o + 0] - mo.entryTheta[(size_t)slot] : 0.0;
            break;
        }
    }
    return tr.greater ? v - tr.threshold : tr.threshold - v;
}

void DynamicsSystem::enterState(const Mode& mo, int idx,
                                const std::vector<double>& q) const {
    mo.current = idx;
    for (size_t gi = 0; gi < mo.guardBodies.size(); ++gi) {
        const int o = m.bodies[(size_t)mo.guardBodies[gi]].qOffset;
        mo.entryTheta[gi] = o >= 0 ? q[(size_t)o + 0] : 0.0;
    }
    for (size_t hi = 0; hi < mo.holdBodies.size(); ++hi) {
        const Drive& d = m.drives[(size_t)mo.holdDrive[hi]];
        const int o = m.bodies[(size_t)mo.holdBodies[hi]].qOffset;
        d.enabled = mo.stateHolds(idx, mo.holdBodies[hi]);
        if (d.enabled) d.theta0Rad = q[(size_t)o + 0];
    }
}

void DynamicsSystem::applyImpulse(const ModeReset& rs, const std::vector<double>& q,
                                  std::vector<double>& qd) const {
    const MassProps& mp = m.mass[(size_t)rs.body];
    const int o = m.bodies[(size_t)rs.body].qOffset;
    // effective rotational inertia about the body's frame origin
    const double I0 = mp.inertia + mp.mass * dot(mp.com, mp.com);
    if (!(I0 > 1e-18)) return;
    const double w = qd[(size_t)o + 0];
    double dE = rs.energy;
    if (rs.maintain) {
        // Exactly the energy for the body to reach the target half-swing
        // about its natural hang angle (com straight below the origin):
        //   E(theta, w) = 1/2 I0 w^2 + m g (y + (R(theta) com).y)
        //   theta_hang  = -pi/2 - atan2(com.x, com.y)
        // A pendulum pinned at its origin swings symmetrically about the
        // hang angle, so this is the horologist's "maintaining power":
        // top up when the swing is short, brake when it is long.
        if (w == 0.0) return;   // no motion: no direction to push toward
        const double phi = std::atan2(mp.com.y, mp.com.x);
        const double thHang = -0.5 * kPi - phi;
        const double dir = w > 0.0 ? 1.0 : -1.0;
        const double thTarget = thHang + dir * rs.swingRad;
        const Vec2 comNow = rotate(mp.com, q[(size_t)o + 0]);
        const Vec2 comTgt = rotate(mp.com, thTarget);
        const double Enow = 0.5 * I0 * w * w +
                            mp.mass * m.world.gravity * (q[(size_t)o + 2] + comNow.y);
        const double Etgt = mp.mass * m.world.gravity * (q[(size_t)o + 2] + comTgt.y);
        dE = Etgt - Enow;
    }
    if (dE == 0.0) return;
    if (w == 0.0) return;   // cannot direct energy into a resting body
    const double w2 = w * w + 2.0 * dE / I0;
    if (w2 <= 0.0) {
        qd[(size_t)o + 0] = 0.0;   // braking exactly to rest
    } else {
        qd[(size_t)o + 0] = (w >= 0.0 ? 1.0 : -1.0) * std::sqrt(w2);
    }
}

void DynamicsSystem::applyTransition(const Mode& mo, const ModeTransition& tr,
                                     std::vector<double>& q, std::vector<double>& qd,
                                     double tNow) const {
    // resets at the crossing instant
    const double T0 = kinetic(q, qd);
    for (const ModeReset& rs : tr.resets) applyImpulse(rs, q, qd);
    const double T1 = kinetic(q, qd);
    stats_.impulseEnergy += T1 - T0;

    // discrete state switch: holds and advance origins are captured from
    // the state at the crossing
    ++mo.fires;
    ++stats_.fires;
    enterState(mo, tr.to, q);

    // re-anchor the configuration on the (possibly changed) row set
    int iters = 0;
    sys.solve(q, tNow, iters);

    // discard whatever velocity the new rows forbid — the relock impact.
    // (With nu = 0 the mass-metric projection can only remove energy.)
    const double T2 = kinetic(q, qd);
    projectVelocities(q, qd);
    const double T3 = kinetic(q, qd);
    stats_.impactLoss += T2 - T3;
}

void DynamicsSystem::accountWeights(const std::vector<double>& qStart,
                                     const std::vector<double>& qEnd) const {
    for (const WeightDrive& w : m.weights) {
        if (w.exhausted) continue;
        const int o = m.bodies[(size_t)w.body].qOffset;
        const double dth = qEnd[(size_t)o + 0] - qStart[(size_t)o + 0];
        // ratchet: only travel in the unwind sense pays out cord
        const double along = (double)w.sign * dth;
        const double forward = along > 0.0 ? along : 0.0;
        if (forward <= 0.0) continue;
        double avail = 1e30;
        if (w.drop >= 0.0) avail = (w.drop - w.paid) / w.barrelR;
        if (avail <= 0.0) { w.exhausted = true; continue; }
        const double used = std::min(forward, avail);
        w.paid += w.barrelR * used;
        stats_.weightDrawn += w.torque * used;
        if (used < forward) w.exhausted = true;
    }
}

void DynamicsSystem::hybridStep(std::vector<double>& q, std::vector<double>& qd,
                                double t, double h) const {
    if (m.modes.empty()) {
        const std::vector<double> q0 = q;
        integrate(q, qd, t, h);
        if (!m.weights.empty()) accountWeights(q0, q);
        return;
    }

    // Event loop: integrate the remainder, detect the first guard whose
    // value rose across zero (rising edge: false at the start of the
    // sub-interval, true at its end), localize the crossing by bisection
    // (re-integrating sub-steps, resolution h/1024), apply the transition,
    // and continue with what is left of the step. Simultaneous rises within
    // one step are processed in declaration order; a pathological mode that
    // fires more than 8 times in one step finishes the step unwatched.
    const std::vector<double> qStart = q;   // for the weight ledger
    double tau = 0.0;
    int events = 0;
    while (tau < h - 1e-12) {
        const double rem = h - tau;
        const double t0 = t + tau;
        const std::vector<double> q0 = q, qd0 = qd;
        integrate(q, qd, t0, rem);   // trial integration of the remainder

        const Mode* fmo = nullptr;
        const ModeTransition* ftr = nullptr;
        for (const Mode& mo : m.modes) {
            for (const ModeTransition& tr : mo.transitions) {
                if (tr.from != mo.current) continue;
                if (guardValue(mo, tr, q0, qd0) > 0.0) continue;  // already true
                if (guardValue(mo, tr, q, qd) <= 0.0) continue;   // not true now
                fmo = &mo;
                ftr = &tr;
                break;
            }
            if (ftr) break;
        }
        if (!ftr) break;   // no event: q, qd hold the whole remainder

        // bisect the crossing: g(t0) <= 0 < g(t0 + hi)
        double lo = 0.0, hi = rem;
        std::vector<double> qHi = q, qdHi = qd;
        for (int it = 0; it < 10; ++it) {
            const double mid = 0.5 * (lo + hi);
            std::vector<double> qm = q0, qdm = qd0;
            integrate(qm, qdm, t0, mid);
            if (guardValue(*fmo, *ftr, qm, qdm) > 0.0) {
                hi = mid;
                qHi = qm;
                qdHi = qdm;
            } else {
                lo = mid;
            }
        }
        if (hi < rem) {
            q = qHi;   // land on the crossing state (guard true, h/1024 sharp)
            qd = qdHi;
        }
        applyTransition(*fmo, *ftr, q, qd, t0 + hi);
        tau += hi;

        if (++events >= 8) {
            // chatter guard: finish the step without further events
            integrate(q, qd, t + tau, h - tau);
            tau = h;
            break;
        }
    }
    if (!m.weights.empty()) accountWeights(qStart, q);
}

double DynamicsSystem::kinetic(const std::vector<double>& q,
                               const std::vector<double>& qd) const {
    double T = 0.0;
    for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
        const MassProps& mp = m.mass[bi];
        const int o = m.bodies[bi].qOffset;
        const double w = qd[o + 0];
        const Vec2 Rc = rotate(mp.com, q[o + 0]);
        const Vec2 vcom{qd[o + 1] - w * Rc.y, qd[o + 2] + w * Rc.x};
        T += 0.5 * mp.mass * dot(vcom, vcom) + 0.5 * mp.inertia * w * w;
    }
    return T;
}

double DynamicsSystem::potential(const std::vector<double>& q) const {
    double V = 0.0;
    if (m.world.gravity != 0.0) {
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const MassProps& mp = m.mass[bi];
            const int o = m.bodies[bi].qOffset;
            const Vec2 Rc = rotate(mp.com, q[o + 0]);
            V += mp.mass * m.world.gravity * (q[o + 2] + Rc.y);
        }
    }
    for (const SpringJoint& sp : m.springs) {
        const Vec2 pa = m.worldAnchor(sp.bodyA, sp.anchA, q);
        const Vec2 pb = m.worldAnchor(sp.bodyB, sp.anchB, q);
        const double ext = vlen(pb - pa) - sp.rest;
        V += 0.5 * sp.k * ext * ext;
    }
    return V;
}

} // namespace mech
