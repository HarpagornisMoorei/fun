// mech — kinematics.cpp: residuals, analytic Jacobian, LM solver
//
// Row layout:
//   pin rows (2 each):    world(A,a) - world(B,b)              = 0
//   slide rows (2 each):  crossz(railDir, guide - railPos)     = 0   (offset)
//                         crossz(R(th)guideDir, railDir)       = 0   (dir lock)
//   drive rows (1 each):  theta_body - (theta0 + w*t)          = 0
//   mesh rows (1 each):   rA*thA + s*rB*thB - (rA + s*rB)*psi - rollConst = 0
//   tangent rows (1 per tangent mesh):
//                         |cB - cA|^2 - (rA + s*rB)^2          = 0
//
// Derivations (see docs/LANG.md):
//   world(b,a)  = R(theta_b) * local_a + t_b
//   d/dtheta [R(theta) v] = perp(R(theta) v)
// so for a pin row r = world(A,a) - world(B,b):
//   dr/dtheta_A = +perp(R(theta_A) local_a)   dr/d(x_A,y_A) = (1,0),(0,1)
//   dr/dtheta_B = -perp(R(theta_B) local_b)   dr/d(x_B,y_B) = -(1,0),-(0,1)
// Ground contributions are constant and drop out of J. Slide rails are
// ground-fixed, so their rows touch only the guide body's columns.
//
// Mesh row geometry: psi = angle of the center line cA -> cB. With fixed
// centers (ordinary gear trains) psi is constant and the row is the plain M2
// roll relation. With moving centers (epicyclic trains) the psi term carries
// the carrier's rotation into the rolling law, which is exactly the classic
// planet-gear kinematics. psi is tracked with branch continuity: it may wind
// without bound as the carrier spins; the tracked value advances by the
// smallest signed increment at each evaluation, so no atan2 branch cut can
// ever inject a 2*pi jump into the residual. The Jacobian needs only the
// center line vector d = cB - cA:
//   dpsi/d(delta) = crossz(d, .) / |d|^2
// which yields, with K = (rA + s*rB)/|d|^2 and w = perp(R host local):
//   drow/dthA = rA + K*crossz(d, wA)     drow/dthB = s*rB - K*crossz(d, wB)
//   drow/d(xA,yA) = (-K*dy, +K*dx)       drow/d(xB,yB) = (+K*dy, -K*dx)
// The tangent row is |d|^2 - (rA + s*rB)^2 with Jacobian 2*d on the center
// columns and 2*dot(d, w) on the angle columns.

#include "kinematics.hpp"
#include "linalg.hpp"

#include <algorithm>
#include <cmath>

namespace mech {

namespace {

inline void addJ(std::vector<double>& J, int cols, int row, int col, double v) {
    J[(size_t)row * cols + col] += v;
}

inline double maxAbs(const std::vector<double>& v) {
    double mx = 0.0;
    for (double x : v) mx = std::max(mx, std::fabs(x));
    return mx;
}

} // namespace

int ConstraintSystem::rowCount() const {
    int rows = 2 * (int)m.pins.size() + 2 * (int)m.slides.size();
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;   // M5: a disabled mode hold contributes no row
        ++rows;
    }
    for (const MeshJoint& mm : m.meshes) {
        rows += mm.tangent ? 2 : 1;
    }
    return rows;
}

void ConstraintSystem::residuals(const std::vector<double>& q, double t,
                                 std::vector<double>& c) const {
    c.assign(rowCount(), 0.0);
    int r = 0;
    for (const PinJoint& p : m.pins) {
        const Vec2 pa = m.worldAnchor(p.bodyA, p.anchA, q);
        const Vec2 pb = m.worldAnchor(p.bodyB, p.anchB, q);
        c[r]     = pa.x - pb.x;
        c[r + 1] = pa.y - pb.y;
        r += 2;
    }
    for (const SlideJoint& s : m.slides) {
        const Body& g = m.bodies[s.guideBody];
        const Anchor& ga = g.anchors[s.guideAnchor];
        const Vec2 rl = rotate(ga.local, q[g.qOffset + 0]);
        const Vec2 p = rl + Vec2{q[g.qOffset + 1], q[g.qOffset + 2]};
        const Vec2 dirG = rotate(ga.dirLocal, q[g.qOffset + 0]);
        c[r]     = crossz(s.railDir, p - s.railPos);   // offset from rail line
        c[r + 1] = crossz(dirG, s.railDir);            // guide parallel to rail
        r += 2;
    }
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;
        const Body& b = m.bodies[d.body];
        const double th = b.qOffset >= 0 ? q[b.qOffset + 0] : 0.0;
        c[r++] = th - (d.theta0Rad + d.omegaRad * t);
    }
    for (const MeshJoint& mm : m.meshes) {
        const Gear& ga = m.gears[mm.gearA];
        const Gear& gb = m.gears[mm.gearB];
        const Body& ba = m.bodies[ga.body];
        const Body& bb = m.bodies[gb.body];
        const Vec2 ca = m.worldAnchor(ga.body, ga.hostAnchor, q);
        const Vec2 cb = m.worldAnchor(gb.body, gb.hostAnchor, q);
        const Vec2 d = cb - ca;
        const double dist = vlen(d);
        const double s = mm.sgn >= 0 ? 1.0 : -1.0;

        // branch-continuity tracking of the center-line angle
        if (dist > 1e-12) {
            const double psiRaw = std::atan2(d.y, d.x);
            if (!mm.psiInit) { mm.psiTrack = psiRaw; mm.psiInit = true; }
            else mm.psiTrack += angDiff(psiRaw, mm.psiTrack);
        }

        const double thA = ba.qOffset >= 0 ? q[ba.qOffset + 0] : 0.0;
        const double thB = bb.qOffset >= 0 ? q[bb.qOffset + 0] : 0.0;
        c[r++] = ga.radius * thA + s * gb.radius * thB
                 - (ga.radius + s * gb.radius) * mm.psiTrack - mm.rollConst;

        if (mm.tangent) {
            const double want = ga.radius + s * gb.radius;
            c[r++] = dot(d, d) - want * want;
        }
    }
}

void ConstraintSystem::jacobian(const std::vector<double>& q,
                                std::vector<double>& J) const {
    const int cols = nq;
    const int rows = rowCount();
    J.assign((size_t)rows * cols, 0.0);

    int r = 0;
    for (const PinJoint& p : m.pins) {
        for (int side = 0; side < 2; ++side) {
            const int bi = side == 0 ? p.bodyA : p.bodyB;
            const int ai = side == 0 ? p.anchA : p.anchB;
            const Body& b = m.bodies[bi];
            if (b.qOffset >= 0) {
                const double sgn = side == 0 ? 1.0 : -1.0;
                const Vec2 rl = rotate(b.anchors[ai].local, q[b.qOffset + 0]);
                addJ(J, cols, r,     b.qOffset + 0, sgn * (-rl.y));
                addJ(J, cols, r + 1, b.qOffset + 0, sgn * ( rl.x));
                addJ(J, cols, r,     b.qOffset + 1, sgn * 1.0);
                addJ(J, cols, r + 1, b.qOffset + 2, sgn * 1.0);
            }
        }
        r += 2;
    }
    for (const SlideJoint& s : m.slides) {
        const Body& g = m.bodies[s.guideBody];
        const Anchor& ga = g.anchors[s.guideAnchor];
        const Vec2 rl = rotate(ga.local, q[g.qOffset + 0]);
        const Vec2 dirG = rotate(ga.dirLocal, q[g.qOffset + 0]);
        // offset row: d crossz(railDir, p - railPos)
        addJ(J, cols, r, g.qOffset + 0, crossz(s.railDir, perp(rl)));
        addJ(J, cols, r, g.qOffset + 1, -s.railDir.y);
        addJ(J, cols, r, g.qOffset + 2,  s.railDir.x);
        // dir-lock row: d crossz(R(th)guideDir, railDir) / dth
        addJ(J, cols, r + 1, g.qOffset + 0, crossz(perp(dirG), s.railDir));
        r += 2;
    }
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;
        const Body& b = m.bodies[d.body];
        if (b.qOffset >= 0) addJ(J, cols, r, b.qOffset + 0, 1.0);
        ++r;
    }
    for (const MeshJoint& mm : m.meshes) {
        const Gear& ga = m.gears[mm.gearA];
        const Gear& gb = m.gears[mm.gearB];
        const Body& ba = m.bodies[ga.body];
        const Body& bb = m.bodies[gb.body];
        const Vec2 ca = m.worldAnchor(ga.body, ga.hostAnchor, q);
        const Vec2 cb = m.worldAnchor(gb.body, gb.hostAnchor, q);
        const Vec2 d = cb - ca;
        const double dist2 = dot(d, d);
        const double dist = std::sqrt(dist2);
        const double s = mm.sgn >= 0 ? 1.0 : -1.0;
        const double roll = ga.radius + s * gb.radius;
        const double K = dist > 1e-9 ? roll / dist2 : 0.0;

        // R(theta)*local for the gear centers (center - body origin)
        Vec2 wA{0.0, 0.0}, wB{0.0, 0.0};
        if (ba.qOffset >= 0) wA = rotate(ba.anchors[ga.hostAnchor].local, q[ba.qOffset + 0]);
        if (bb.qOffset >= 0) wB = rotate(bb.anchors[gb.hostAnchor].local, q[bb.qOffset + 0]);

        if (ba.qOffset >= 0) {
            addJ(J, cols, r, ba.qOffset + 0, ga.radius + K * crossz(d, wA));
            addJ(J, cols, r, ba.qOffset + 1, -K * d.y);
            addJ(J, cols, r, ba.qOffset + 2,  K * d.x);
        }
        if (bb.qOffset >= 0) {
            addJ(J, cols, r, bb.qOffset + 0, s * gb.radius - K * crossz(d, wB));
            addJ(J, cols, r, bb.qOffset + 1,  K * d.y);
            addJ(J, cols, r, bb.qOffset + 2, -K * d.x);
        }
        ++r;

        if (mm.tangent) {
            if (ba.qOffset >= 0) {
                addJ(J, cols, r, ba.qOffset + 0, -2.0 * dot(d, wA));
                addJ(J, cols, r, ba.qOffset + 1, -2.0 * d.x);
                addJ(J, cols, r, ba.qOffset + 2, -2.0 * d.y);
            }
            if (bb.qOffset >= 0) {
                addJ(J, cols, r, bb.qOffset + 0,  2.0 * dot(d, wB));
                addJ(J, cols, r, bb.qOffset + 1,  2.0 * d.x);
                addJ(J, cols, r, bb.qOffset + 2,  2.0 * d.y);
            }
            ++r;
        }
    }
}

bool ConstraintSystem::solve(std::vector<double>& q, double t, int& itersOut,
                             double* maxResidualOut) const {
    // Deterministic multi-start: the incoming guess first, then two tilted
    // restarts. Cheap insurance against singular seeds (e.g. dead points);
    // costs nothing in the warm-started steady state.
    const std::vector<double> q0 = q;
    static const double kTilt[2] = {0.37, -0.53};
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (attempt > 0) {
            q = q0;
            for (size_t i = 0; i < q.size(); ++i)
                q[i] += kTilt[attempt - 1] * (0.5 + 0.25 * ((i % 3) - 1));
        }
        if (solveAttempt(q, t, itersOut, maxResidualOut)) return true;
    }
    return false;   // q holds the closest attempt
}

bool ConstraintSystem::solveAttempt(std::vector<double>& q, double t, int& itersOut,
                                     double* maxResidualOut) const {
    std::vector<double> c;
    residuals(q, t, c);
    double cost = 0.0;
    for (double v : c) cost += v * v;

    std::vector<double> best = q;
    double bestCost = cost;

    double lam = 1e-3;
    const double kStepClamp = 0.35;   // radians / world units per iteration
    std::vector<double> J, A, g, delta;

    for (int iter = 0; iter < 100; ++iter) {
        const double mx = maxAbs(c);
        if (mx < 1e-10) {
            itersOut = iter;
            if (maxResidualOut) *maxResidualOut = mx;
            return true;
        }

        jacobian(q, J);
        const int rows = (int)c.size();

        // normal equations: A = J^T J + lam^2 I, g = J^T c
        A.assign((size_t)nq * nq, 0.0);
        g.assign(nq, 0.0);
        for (int i = 0; i < rows; ++i) {
            const double* Ji = &J[(size_t)i * nq];
            for (int j = 0; j < nq; ++j) {
                const double Jij = Ji[j];
                if (Jij == 0.0) continue;
                g[j] += Jij * c[i];
                for (int k = j; k < nq; ++k)
                    A[(size_t)j * nq + k] += Jij * Ji[k];
            }
        }
        for (int i = 0; i < nq; ++i) A[(size_t)i * nq + i] += lam * lam;
        // mirror upper triangle to lower
        for (int i = 0; i < nq; ++i)
            for (int k = 0; k < i; ++k)
                A[(size_t)i * nq + k] = A[(size_t)k * nq + i];

        // Newton/LM step: solve (J^T J + lam^2 I) delta = -J^T c
        std::vector<double> rhs(nq);
        for (int j = 0; j < nq; ++j) rhs[j] = -g[j];
        if (!choleskySolve(A, rhs, nq, delta)) {
            lam *= 10.0;
            if (lam > 1e14) break;
            continue;
        }

        // damped, clamped trial step
        std::vector<double> q2 = q;
        for (int i = 0; i < nq; ++i) {
            double d = std::max(-kStepClamp, std::min(kStepClamp, delta[i]));
            q2[i] += d;
        }

        std::vector<double> c2;
        residuals(q2, t, c2);
        double cost2 = 0.0;
        for (double v : c2) cost2 += v * v;

        if (cost2 < cost) {
            q.swap(q2);
            c.swap(c2);
            cost = cost2;
            if (cost < bestCost) { best = q; bestCost = cost; }
            lam = std::max(lam / 3.0, 1e-9);
        } else {
            lam *= 8.0;
            if (lam > 1e14) break;
        }
    }

    const double mx = maxAbs(c);
    itersOut = -1;
    if (maxResidualOut) *maxResidualOut = mx;
    if (mx < 1e-8) return true;
    if (bestCost < cost) q = best;   // keep the best posture seen
    return false;
}

double ConstraintSystem::maxResidual(const std::vector<double>& q, double t) const {
    std::vector<double> c;
    residuals(q, t, c);
    return maxAbs(c);
}

} // namespace mech
