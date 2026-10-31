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

// Regularized Coulomb friction: F_t = mu N tanh(s / eps), s the slip speed. Below
// ~eps the contact "creeps" instead of sticking (the price of a smooth force that an
// explicit integrator can take); 2 mm/s is far below any motion worth modelling. The
// width is raised per contact (see accelerations) so mu N / eps never exceeds what the
// step can integrate stably.
const double kFrictionEps = 2e-3;

// Solve A x = b for symmetric A with growing Tikhonov damping until the
// Cholesky succeeds. Returns false only if even brutal damping collapses.
//
// The damping is relative to each row's own diagonal. (It used to be one absolute number scaled by the
// largest diagonal, which for masses spanning many decades — a 10 kg boom and a 1e-5 kg linkage — swamped
// the rows that come from the heavy bodies and skewed the multipliers by percent.)
bool dampedSolve(std::vector<double> A, const std::vector<double>& b, int n,
                 std::vector<double>& x) {
    double eps = 1e-10;
    for (int attempt = 0; attempt < 6; ++attempt) {
        std::vector<double> Ad = A;
        for (int i = 0; i < n; ++i) {
            const double d = Ad[(size_t)i * n + i];
            Ad[(size_t)i * n + i] += eps * std::max(d, 1e-12) + 1e-15;
        }
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
    // M14: a linear force pushes the guide along the rail and the rail's body back
    for (const LinearDrive& ld : m.linDrives) {
        if (!ld.isForce) continue;
        const SlideGeom g = slideGeom(m, m.slides[(size_t)ld.slide], q);
        const Vec2 F = ld.force * g.u;
        Q[g.oA + 0] += crossz(g.rA, F);
        Q[g.oA + 1] += F.x;
        Q[g.oA + 2] += F.y;
        if (g.oB >= 0) {
            Q[g.oB + 0] -= crossz(g.rB, F);
            Q[g.oB + 1] -= F.x;
            Q[g.oB + 2] -= F.y;
        }
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
    // Slide rows. With u the rail direction (turning with the rail's body, rate wB), v = pA - pB and
    // vdot the relative velocity of the two anchor points, the offset row f1 = crossz(u, v) has
    //   f1'' = J qdd + bias,  bias = crossz(-wB^2 u, v) + 2 crossz(wB perp(u), vdot)
    //                                + crossz(u, -wA^2 rA + wB^2 rB)
    // and the direction-lock row f2 = crossz(a, u) (a the guide's direction) has bias = -(wA - wB)^2 f2.
    // For a ground rail (wB = 0, rB = 0) these are the old -crossz(u, wA^2 rA) and -wA^2 f2.
    for (const SlideJoint& s : m.slides) {
        const SlideGeom g = slideGeom(m, s, q);
        const double wA = bodyW(s.guideBody), wB = s.railBody ? bodyW(s.railBody) : 0.0;
        const Vec2 vA = anchorVel(s.guideBody, s.guideAnchor);
        const Vec2 vB = s.railBody ? anchorVel(s.railBody, s.railAnchor) : Vec2{0.0, 0.0};
        const Vec2 v = g.pA - g.pB, vdot = vA - vB;
        bias[r] = crossz((-wB * wB) * g.u, v) + 2.0 * crossz(wB * perp(g.u), vdot) +
                  crossz(g.u, (-wA * wA) * g.rA + (wB * wB) * g.rB);
        ++r;
        if (s.lockDir) {
            bias[r] = -(wA - wB) * (wA - wB) * c[r];
            ++r;
        }
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

    // M6d: non-circular mesh rows. R = thB - thB0 - F(psi - thA), F' = rho, so
    //   R'  = wB + rho wA
    //   R'' = aB + rho aA - rho'(phi_A) wA^2          (phi_A' = -wA)
    // and J qdd + bias = 0 with J = (rho, 1) and bias = -rho' wA^2.
    for (const PolarMesh& pm : m.polarMeshes) {
        const Profile& pa = m.profiles[(size_t)pm.profA];
        const double wA = bodyW(pa.body);
        double rho, drho;
        pm.phase(*pa.curve, pm.psi - q[(size_t)m.bodies[(size_t)pa.body].qOffset], rho, drho);
        bias[r++] = -drho * wA * wA;
    }

    // M6c: touch rows. The row is the signed gap g(q) of a point against a
    // curve, so with w the point's velocity relative to the curve's material
    // point at the foot, t the unit tangent, kappa the curve's curvature and
    // u the foot's sliding speed along the curve (see docs/LANG.md 5.2):
    //
    //   g''  = J qdd + bias
    //   bias = n . (-wB^2 rB + wA^2 rA) + wA u + Omega (t . w)
    //   u    = (t . w - g wA) / (1 + g kappa),     Omega = wA + kappa u
    //
    // (A = the profile's body, B = the touching point's body; rA is the arm to
    // the foot, rB the arm to the anchor.) For a roller g is its radius, so
    // 1 + g kappa is the offset curve's stretch over the surface.
    const int touchRow0 = r;
    for (const TouchJoint& tj : m.touches) {
        if (!tj.engaged) continue;
        const ContactState cs = m.touchState(tj, q);
        const Profile& pr = m.profiles[(size_t)tj.profile];
        const double wB = bodyW(tj.body), wA = bodyW(pr.body);
        const Vec2 rB = anchorOff(tj.body, tj.anchor);
        const Vec2 rA = cs.footArm;
        const Vec2 w = bodyVel(tj.body) - bodyVel(pr.body) + wB * perp(rB) - wA * perp(rA);
        const double tw = dot(cs.tangent, w);
        const double u = (tw - cs.gap * wA) / (1.0 + cs.gap * cs.kappa);
        const double Om = wA + cs.kappa * u;
        bias[r++] = dot(cs.normal, (-wB * wB) * rB + (wA * wA) * rA) + wA * u + Om * tw;
    }
    // M14: linear rate drives. s = u . v, so s'' = J qdd + bias with
    //   bias = dot(-wB^2 u, v) + 2 wB dot(perp(u), vdot) + dot(u, -wA^2 rA + wB^2 rB)
    const int linRow0 = r;
    for (const LinearDrive& ld : m.linDrives) {
        if (ld.isForce) continue;
        const SlideJoint& s = m.slides[(size_t)ld.slide];
        const SlideGeom g = slideGeom(m, s, q);
        const double wA = bodyW(s.guideBody), wB = s.railBody ? bodyW(s.railBody) : 0.0;
        const Vec2 vA = anchorVel(s.guideBody, s.guideAnchor);
        const Vec2 vB = s.railBody ? anchorVel(s.railBody, s.railAnchor) : Vec2{0.0, 0.0};
        const Vec2 v = g.pA - g.pB, vdot = vA - vB;
        bias[r++] = dot((-wB * wB) * g.u, v) + 2.0 * wB * dot(perp(g.u), vdot) +
                    dot(g.u, (-wA * wA) * g.rA + (wB * wB) * g.rB);
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
    // A = J W J^T (it does not depend on the forces)
    std::vector<double> A((size_t)rows * rows, 0.0);
    for (int j = 0; j < rows; ++j) {
        const double* Jj = &J[(size_t)j * nq];
        for (int k = j; k < rows; ++k) {
            double s = 0.0;
            for (int cix = 0; cix < nq; ++cix) s += Jj[cix] * BJ[k][cix];
            A[(size_t)j * rows + k] = s;
            A[(size_t)k * rows + j] = s;
        }
    }

    // Multipliers and accelerations under a given generalized force Qe:
    //   z = W Qe,  A lam = -(J z + bias),  qdd = z + W J^T lam
    std::vector<double> z(nq, 0.0), lam;
    auto solveWith = [&](const std::vector<double>& Qe) {
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const int o = m.bodies[bi].qOffset;
            double wv[3];
            applyW(Mblocks, (int)bi, Qe[o], Qe[o + 1], Qe[o + 2], wv);
            z[o] = wv[0];
            z[o + 1] = wv[1];
            z[o + 2] = wv[2];
        }
        std::vector<double> rhs(rows, 0.0);
        for (int j = 0; j < rows; ++j) {
            const double* Jj = &J[(size_t)j * nq];
            double s = 0.0;
            for (int cix = 0; cix < nq; ++cix) s += Jj[cix] * z[cix];
            rhs[j] = -(s + bias[j]);
        }
        lam.assign((size_t)rows, 0.0);
        if (rows > 0) dampedSolve(A, rhs, rows, lam);
        qdd = z;
        for (int j = 0; j < rows; ++j) {
            if (lam[j] == 0.0) continue;
            for (int cix = 0; cix < nq; ++cix) qdd[cix] += BJ[j][cix] * lam[j];
        }
    };
    solveWith(Qeff);

    // M6f: Coulomb friction at engaged contacts. The tangential force mu N tanh(s/eps)
    // opposes the slip speed s = t . w, and N is the multiplier of the very solve it
    // loads, so iterate: solve, add friction from that N, solve again (converges unless
    // the geometry is jammed; three passes). The force acts through the slip Jacobian
    //   s = Jt qd,   Jt_B = (t . perp(rB), t),   Jt_A = -(t . perp(rA), t),
    // as Q = -F_t Jt^T. Its width eps is raised so the friction "spring" mu N / eps
    // stays below 1 / (h * mobility), mobility being the constrained inverse inertia
    // along Jt: otherwise a stiff contact on a light body is unstable under RK4.
    lastFrictionPower_ = 0.0;
    {
        std::vector<int> trow(m.touches.size(), -1);
        std::vector<size_t> fric;
        int rr = touchRow0;
        for (size_t ti = 0; ti < m.touches.size(); ++ti) {
            if (!m.touches[ti].engaged) continue;
            trow[ti] = rr++;
            if (m.touches[ti].friction > 0.0) fric.push_back(ti);
        }
        for (int pass = 0; pass < 3 && !fric.empty(); ++pass) {
            std::vector<double> Qtot = Qeff;
            double power = 0.0;
            for (size_t ti : fric) {
                const TouchJoint& tj = m.touches[ti];
                const Profile& pr = m.profiles[(size_t)tj.profile];
                const double N = std::max(0.0, lam[(size_t)trow[ti]]);
                const ContactState cs = m.touchState(tj, q);
                const Vec2 t = cs.tangent;
                std::vector<double> Jt((size_t)nq, 0.0);
                const Body& bp = m.bodies[(size_t)tj.body];
                const Body& bh = m.bodies[(size_t)pr.body];
                if (bp.qOffset >= 0) {
                    Jt[(size_t)bp.qOffset + 0] += dot(t, perp(anchorOff(tj.body, tj.anchor)));
                    Jt[(size_t)bp.qOffset + 1] += t.x;
                    Jt[(size_t)bp.qOffset + 2] += t.y;
                }
                if (bh.qOffset >= 0) {
                    Jt[(size_t)bh.qOffset + 0] -= dot(t, perp(cs.footArm));
                    Jt[(size_t)bh.qOffset + 1] -= t.x;
                    Jt[(size_t)bh.qOffset + 2] -= t.y;
                }
                double s = 0.0;
                for (int i = 0; i < nq; ++i) s += Jt[(size_t)i] * qd[(size_t)i];

                // constrained mobility along Jt: Jt W Jt^T - (Jt W J^T) A^-1 (J W Jt^T)
                std::vector<double> g((size_t)nq, 0.0);
                for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
                    const int o = m.bodies[bi].qOffset;
                    double wv[3];
                    applyW(Mblocks, (int)bi, Jt[(size_t)o], Jt[(size_t)o + 1], Jt[(size_t)o + 2], wv);
                    g[(size_t)o] = wv[0];
                    g[(size_t)o + 1] = wv[1];
                    g[(size_t)o + 2] = wv[2];
                }
                double mob = 0.0;
                for (int i = 0; i < nq; ++i) mob += Jt[(size_t)i] * g[(size_t)i];
                if (rows > 0) {
                    std::vector<double> a((size_t)rows, 0.0), y;
                    for (int j = 0; j < rows; ++j)
                        for (int i = 0; i < nq; ++i) a[(size_t)j] += J[(size_t)j * nq + i] * g[(size_t)i];
                    if (dampedSolve(A, a, rows, y)) {
                        double red = 0.0;
                        for (int j = 0; j < rows; ++j) red += a[(size_t)j] * y[(size_t)j];
                        if (mob - red > 1e-12) mob -= red;
                    }
                }
                const double eps = std::max(kFrictionEps, tj.friction * N * m.world.step * mob);
                const double Ft = tj.friction * N * std::tanh(s / eps);
                for (int i = 0; i < nq; ++i) Qtot[(size_t)i] -= Ft * Jt[(size_t)i];
                power += Ft * s;
            }
            solveWith(Qtot);
            lastFrictionPower_ = power;
        }
    }

    // M6c: contact normal forces (0 where the contact is lifted off)
    lastContact_.assign(m.touches.size(), 0.0);
    {
        int rr = touchRow0;
        for (size_t ti = 0; ti < m.touches.size(); ++ti) {
            if (!m.touches[ti].engaged) continue;
            lastContact_[ti] = rr < (int)lam.size() ? lam[rr] : 0.0;
            ++rr;
        }
    }

    // M14: the force each linear rate drive transmits (the multiplier of its row)
    lastLinReaction_.assign(m.linDrives.size(), 0.0);
    {
        int rr = linRow0;
        for (size_t li = 0; li < m.linDrives.size(); ++li) {
            if (m.linDrives[li].isForce) continue;
            lastLinReaction_[li] = rr < (int)lam.size() ? lam[rr] : 0.0;
            ++rr;
        }
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
    // M6c: the drives' power at each stage, for the energy ledger: Simpson over
    // the four stages. A rate drive delivers (reaction torque x prescribed rate);
    // a torque drive delivers (torque x the body's angular velocity there).
    auto power = [&](const std::vector<double>& qq, const std::vector<double>& v) {
        double P = 0.0;
        for (size_t di = 0; di < m.drives.size(); ++di)
            if (m.drives[di].enabled) P += lastReaction_[di] * m.drives[di].omegaRad;
        for (const TorqueDrive& td : m.torques)
            P += td.torque * v[(size_t)m.bodies[(size_t)td.body].qOffset];
        // M14: a linear rate drive delivers (force x prescribed rate); a force drive (force x the
        // guide's speed relative to the rail, along it)
        for (size_t li = 0; li < m.linDrives.size(); ++li) {
            const LinearDrive& ld = m.linDrives[li];
            if (!ld.isForce) { P += lastLinReaction_[li] * ld.rate; continue; }
            const SlideGeom g = slideGeom(m, m.slides[(size_t)ld.slide], qq);
            Vec2 vA{v[(size_t)g.oA + 1] - v[(size_t)g.oA] * g.rA.y, v[(size_t)g.oA + 2] + v[(size_t)g.oA] * g.rA.x};
            Vec2 vB{0.0, 0.0};
            if (g.oB >= 0) vB = Vec2{v[(size_t)g.oB + 1] - v[(size_t)g.oB] * g.rB.y, v[(size_t)g.oB + 2] + v[(size_t)g.oB] * g.rB.x};
            P += ld.force * dot(g.u, vA - vB);
        }
        return P;
    };
    std::vector<double> a1, a2, a3, a4;
    accelerations(q, qd, a1);
    const double P1 = power(q, qd), F1 = lastFrictionPower_;

    std::vector<double> qS(nq), qdS(nq);
    for (int i = 0; i < nq; ++i) {
        qS[i] = q[i] + 0.5 * h * qd[i];
        qdS[i] = qd[i] + 0.5 * h * a1[i];
    }
    accelerations(qS, qdS, a2);
    const double P2 = power(qS, qdS), F2 = lastFrictionPower_;

    for (int i = 0; i < nq; ++i) {
        qS[i] = q[i] + 0.5 * h * qd[i] + 0.25 * h * h * a1[i];
        qdS[i] = qd[i] + 0.5 * h * a2[i];
    }
    accelerations(qS, qdS, a3);
    const double P3 = power(qS, qdS), F3 = lastFrictionPower_;

    for (int i = 0; i < nq; ++i) {
        qS[i] = q[i] + h * qd[i] + 0.5 * h * h * a2[i];
        qdS[i] = qd[i] + h * a3[i];
    }
    accelerations(qS, qdS, a4);
    lastDriveWork_ = h / 6.0 * (P1 + 2.0 * P2 + 2.0 * P3 + power(qS, qdS));
    lastFrictionLoss_ = h / 6.0 * (F1 + 2.0 * F2 + 2.0 * F3 + lastFrictionPower_);

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
                                        std::vector<double>& qd,
                                        double* impulseWork) const {
    const int nq = m.nq();
    const int rows = sys.rowCount();
    if (impulseWork) *impulseWork = 0.0;
    if (rows <= 0) return;

    std::vector<double> J2;
    sys.jacobian(q, J2);
    // nu: drive rows prescribe their rate; every other row wants J qd = 0
    const int driveRow0 = sys.driveRowStart();
    std::vector<double> nu(rows, 0.0);
    {
        int r = driveRow0;
        for (const Drive& d : m.drives) {
            if (!d.enabled) continue;
            nu[r++] = d.omegaRad;
        }
        int lr = sys.linearRowStart();                   // M14: linear rate drives prescribe the guide's speed
        for (const LinearDrive& ld : m.linDrives) {
            if (ld.isForce) continue;
            nu[(size_t)lr++] = ld.rate;
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
            // the impulse on row j is -mu_j; on a prescribed coordinate it
            // does work (-mu_j) * nu_j
            if (impulseWork) *impulseWork -= mu[j] * nu[j];
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

// ---- M6c unilateral contact -----------------------------------------------------
//
// A touch in a dynamic mechanism is a two-state element. While `engaged` its
// row is an ordinary equality constraint (gap = roller) and its multiplier is
// the normal force, which must stay >= 0. It lifts off when that force would
// have to go negative (a pull); it lands again when the gap closes, and the
// landing is a Newton impact. Both are zero-crossings handled by the same
// bisection loop as the mode guards.

namespace {
const double kForceTol = 1e-9;    // N: below this a contact is "just touching"
const double kSettleGap = 1e-9;   // m: deeper than this at a step start is a touch-down
// m/s: slower landings are plastic. A bounce restarts inside the kSettleGap band, so
// the guard can only see a landing after sqrt(2 a kSettleGap) of free speed (about
// 3e-4 m/s at 56 m/s^2, 1.4e-3 at 1000): without a cutoff above that floor a
// resting contact would bounce forever. 2 mm/s clears it with margin and is far
// below any landing worth modelling a rebound for.
const double kStickSpeed = 2e-3;
}

double DynamicsSystem::contactForce(int ti, const std::vector<double>& q,
                                    const std::vector<double>& qd) const {
    std::vector<double> qdd;
    accelerations(q, qd, qdd);
    return lastContact_[(size_t)ti];
}

void DynamicsSystem::settleContacts(std::vector<double>& q, std::vector<double>& qd,
                                    double tNow) const {
    // Level-triggered resolution at the start of a (sub)step: a point already
    // inside its curve lands now; an engaged contact that would have to pull
    // lets go now. A few passes are enough — each pass flips at most one.
    for (int pass = 0; pass < 8; ++pass) {
        bool changed = false;
        for (size_t ti = 0; ti < m.touches.size(); ++ti) {
            const TouchJoint& tj = m.touches[ti];
            if (tj.engaged) continue;
            if (m.touchState(tj, q).gap - tj.roller < -kSettleGap) {
                applyImpact((int)ti, q, qd, tNow);
                changed = true;
                break;
            }
        }
        if (changed) continue;

        bool anyEngaged = false;
        for (const TouchJoint& tj : m.touches) if (tj.engaged) anyEngaged = true;
        if (!anyEngaged) break;
        std::vector<double> qdd;
        accelerations(q, qd, qdd);
        int worst = -1;
        double worstF = -kForceTol;
        for (size_t ti = 0; ti < m.touches.size(); ++ti) {
            if (!m.touches[ti].engaged) continue;
            if (lastContact_[ti] < worstF) { worstF = lastContact_[ti]; worst = (int)ti; }
        }
        if (worst < 0) break;
        m.touches[(size_t)worst].engaged = false;   // lift off: no velocity change
        ++stats_.liftoffs;
    }
}

void DynamicsSystem::applyImpact(int ti, std::vector<double>& q, std::vector<double>& qd,
                                 double tNow) const {
    const TouchJoint& tj = m.touches[(size_t)ti];
    ++stats_.impacts;

    // Row on, position re-anchored onto the surface (the crossing was located
    // to h/1024, so this is a sliver).
    tj.engaged = true;
    int iters = 0;
    sys.solve(q, tNow, iters);

    // Approach speed along the contact normal, from the pre-impact velocity.
    std::vector<double> row((size_t)m.nq(), 0.0);
    sys.touchRow(tj, q, row.data());
    double gdot = 0.0;
    for (int i = 0; i < m.nq(); ++i) gdot += row[(size_t)i] * qd[(size_t)i];
    if (gdot >= 0.0) return;   // not actually approaching: just stay engaged

    // Newton's impact law in the mass metric. qs is the velocity with the
    // contact row held at rest (every other constraint still satisfied), so
    //   qd+ = (1 + e) qs - e qd-
    // has contact speed -e * gdot- and the same impulse direction. A slow
    // landing is plastic: no bouncing the contact to death.
    const double e = (-gdot < kStickSpeed) ? 0.0 : tj.restitution;
    const double T0 = kinetic(q, qd);
    std::vector<double> qs = qd;
    double Wstick = 0.0;
    projectVelocities(q, qs, &Wstick);
    for (int i = 0; i < m.nq(); ++i)
        qd[(size_t)i] = (1.0 + e) * qs[(size_t)i] - e * qd[(size_t)i];
    const double dT = kinetic(q, qd) - T0;

    // Energy: dT = (impulse work done by prescribed coordinates) - dissipated.
    const double W = (1.0 + e) * Wstick;
    stats_.driveWork += W;
    stats_.impactLoss += W - dT;

    if (e > 0.0) tj.engaged = false;   // bouncing away; it can land again
}

void DynamicsSystem::hybridStep(std::vector<double>& q, std::vector<double>& qd,
                                double t, double h) const {
    const bool contacts = !m.touches.empty();
    if (m.modes.empty() && !contacts) {
        const std::vector<double> q0 = q;
        integrate(q, qd, t, h);
        stats_.driveWork += lastDriveWork_;
        stats_.frictionLoss += lastFrictionLoss_;
        if (!m.weights.empty()) accountWeights(q0, q);
        return;
    }

    // Event loop: integrate the remainder, detect every guard whose value
    // rose across zero (rising edge: false at the start of the sub-interval,
    // true at its end), localize each crossing by bisection (re-integrating
    // sub-steps, resolution h/1024), process the *earliest*, and continue with
    // what is left of the step. Guards are mode transitions, contact lift-off
    // (the normal force going negative) and contact touch-down (the gap
    // closing). Simultaneous crossings go in declaration order; a mechanism
    // that fires more than 8 events in one step (32 with contacts, where a
    // restitution bounce sequence is legitimate) finishes it unwatched; the
    // next step's settle pass catches whatever that skipped.
    const std::vector<double> qStart = q;   // for the weight ledger
    double tau = 0.0, work = 0.0, fric = 0.0;   // committed drive work and friction loss this step
    int events = 0;

    struct Cand {
        int kind;                  // 0 mode transition, 1 lift-off, 2 touch-down
        const Mode* mo;
        const ModeTransition* tr;
        int ti;
    };
    // value > 0 means "the event has happened"
    auto value = [&](const Cand& c, const std::vector<double>& qq,
                     const std::vector<double>& dd) -> double {
        switch (c.kind) {
            case 0: return guardValue(*c.mo, *c.tr, qq, dd);
            case 1: return -contactForce(c.ti, qq, dd) - kForceTol;
            default: {
                // armed while the point is within kSettleGap of the surface (just
                // bounced, or resting), so a landing is never "already true"
                const TouchJoint& tj = m.touches[(size_t)c.ti];
                return -(m.touchState(tj, qq).gap - tj.roller) - kSettleGap;
            }
        }
    };

    if (contacts) settleContacts(q, qd, t);

    while (tau < h - 1e-12) {
        const double rem = h - tau;
        const double t0 = t + tau;
        const std::vector<double> q0 = q, qd0 = qd;
        integrate(q, qd, t0, rem);   // trial integration of the remainder
        const double wTrial = lastDriveWork_, fTrial = lastFrictionLoss_;

        std::vector<Cand> fired;
        for (const Mode& mo : m.modes) {
            for (const ModeTransition& tr : mo.transitions) {
                if (tr.from != mo.current) continue;
                if (guardValue(mo, tr, q0, qd0) > 0.0) continue;  // already true
                if (guardValue(mo, tr, q, qd) <= 0.0) continue;   // not true now
                fired.push_back({0, &mo, &tr, -1});
            }
        }
        for (size_t ti = 0; ti < m.touches.size(); ++ti) {
            const Cand c{m.touches[ti].engaged ? 1 : 2, nullptr, nullptr, (int)ti};
            if (value(c, q0, qd0) > 0.0) continue;    // already true
            if (value(c, q, qd) <= 0.0) continue;     // not true now
            fired.push_back(c);
        }
        if (fired.empty()) {      // no event: q, qd hold the whole remainder
            work += wTrial;
            fric += fTrial;
            break;
        }

        // bisect each candidate's crossing: value(t0) <= 0 < value(t0 + hi);
        // the earliest wins (ties keep declaration order)
        double bestHi = 1e300, bestWork = 0.0, bestFric = 0.0;
        std::vector<double> bestQ, bestQd;
        size_t bestIdx = 0;
        for (size_t fi = 0; fi < fired.size(); ++fi) {
            double lo = 0.0, hi = rem, wHi = wTrial, fHi = fTrial;
            std::vector<double> qHi = q, qdHi = qd;
            for (int it = 0; it < 10; ++it) {
                const double mid = 0.5 * (lo + hi);
                std::vector<double> qm = q0, qdm = qd0;
                integrate(qm, qdm, t0, mid);
                const double wm = lastDriveWork_, fm = lastFrictionLoss_;
                if (value(fired[fi], qm, qdm) > 0.0) {
                    hi = mid;
                    qHi = qm;
                    qdHi = qdm;
                    wHi = wm;
                    fHi = fm;
                } else {
                    lo = mid;
                }
            }
            if (hi < bestHi) {
                bestHi = hi;
                bestWork = wHi;
                bestFric = fHi;
                bestQ = qHi;
                bestQd = qdHi;
                bestIdx = fi;
            }
        }
        const Cand& ev = fired[bestIdx];
        if (bestHi < rem) {
            q = bestQ;     // land on the crossing state (guard true, h/1024 sharp)
            qd = bestQd;
        }
        work += bestWork;
        fric += bestFric;

        switch (ev.kind) {
            case 0:
                applyTransition(*ev.mo, *ev.tr, q, qd, t0 + bestHi);
                break;
            case 1:
                m.touches[(size_t)ev.ti].engaged = false;   // lift off, velocity unchanged
                ++stats_.liftoffs;
                break;
            default:
                applyImpact(ev.ti, q, qd, t0 + bestHi);
                break;
        }
        tau += bestHi;

        if (++events >= (contacts ? 32 : 8)) {
            // chatter guard: finish the step without further events
            integrate(q, qd, t + tau, h - tau);
            work += lastDriveWork_;
            fric += lastFrictionLoss_;
            tau = h;
            break;
        }
    }
    stats_.driveWork += work;
    stats_.frictionLoss += fric;
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
