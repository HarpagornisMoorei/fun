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
//   polar mesh rows (1 each):  thB - thB0 - F(psi - thA)  = 0   (M6d; F = integral of r_A/(d - r_A))
//   touch rows (1 each):  signedGap(point, profile) - roller   = 0   (M6a)
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
//
// Touch row (M6a). g is the signed distance from the touching point p (body B)
// to the profile curve c(s) (body A), s the foot of the perpendicular. Because
// g is the distance to the *closest* point, it is stationary in s at the foot
// (d is parallel to the normal n, and n . c'(s) = 0), so the Jacobian needs no
// ds term: it is the pin-row pattern projected on the world normal n,
//   dg/d(xB,yB) = +n        dg/dthB = +n . perp(R(thB) pLocal)
//   dg/d(xA,yA) = -n        dg/dthA = -n . perp(R(thA) (originA + c(s)))
// where originA is the host anchor (the profile's origin). The foot is found
// from scratch at every evaluation (ProfileCurve::nearest), so the row is a
// pure function of q — no tracked state to go stale between LM trial steps.

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

SlideGeom slideGeom(const Model& m, const SlideJoint& s, const std::vector<double>& q) {
    SlideGeom g;
    const Body& gb = m.bodies[s.guideBody];
    const Anchor& ga = gb.anchors[s.guideAnchor];
    g.oA = gb.qOffset;
    g.rA = rotate(ga.local, q[gb.qOffset + 0]);
    g.pA = g.rA + Vec2{q[gb.qOffset + 1], q[gb.qOffset + 2]};
    if (s.lockDir) g.dirG = rotate(ga.dirLocal, q[gb.qOffset + 0]);
    if (s.railBody == 0) {
        g.pB = s.railPos;
        g.u = s.railDir;
    } else {
        const Body& rb = m.bodies[s.railBody];
        const Anchor& ra = rb.anchors[s.railAnchor];
        g.oB = rb.qOffset;
        g.rB = rotate(ra.local, q[rb.qOffset + 0]);
        g.pB = g.rB + Vec2{q[rb.qOffset + 1], q[rb.qOffset + 2]};
        g.u = rotate(ra.dirLocal, q[rb.qOffset + 0]);
    }
    return g;
}

int ConstraintSystem::slideRowCount() const {
    int n = 0;
    for (const SlideJoint& s : m.slides) n += s.lockDir ? 2 : 1;
    return n;
}

int ConstraintSystem::linearRowStart() const {
    return rowCount() - (int)std::count_if(m.linDrives.begin(), m.linDrives.end(),
                                           [](const LinearDrive& d) { return !d.isForce; });
}

int ConstraintSystem::rowCount() const {
    int rows = 2 * (int)m.pins.size() + slideRowCount();
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;   // M5: a disabled mode hold contributes no row
        ++rows;
    }
    for (const MeshJoint& mm : m.meshes) {
        rows += mm.tangent ? 2 : 1;
    }
    rows += (int)m.polarMeshes.size();
    for (const TouchJoint& tj : m.touches)
        if (tj.engaged) ++rows;        // M6c: a lifted-off contact adds no row
    for (const RackMesh& rm : m.racks) rows += rm.tangent ? 2 : 1;   // M16: rolling (+ tangency) per pinion on a rack
    for (const Belt& b : m.belts) rows += (int)b.gears.size() - 1;
    for (const LinearDrive& d : m.linDrives)
        if (!d.isForce) ++rows;        // M14: a linear rate drive is one row (a force is not a constraint)
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
        const SlideGeom g = slideGeom(m, s, q);
        c[r++] = crossz(g.u, g.pA - g.pB);                  // offset from the rail line
        if (s.lockDir) c[r++] = crossz(g.dirG, g.u);        // guide parallel to rail (not for a slot)
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
    for (const PolarMesh& pm : m.polarMeshes) {
        const Profile& pa = m.profiles[(size_t)pm.profA];
        const Profile& pb = m.profiles[(size_t)pm.profB];
        const double thA = q[(size_t)m.bodies[(size_t)pa.body].qOffset];
        const double thB = q[(size_t)m.bodies[(size_t)pb.body].qOffset];
        double rho, drho;
        const double F = pm.phase(*pa.curve, pm.psi - thA, rho, drho);
        c[r++] = thB - pm.thB0 - F;
    }
    for (const TouchJoint& tj : m.touches) {
        if (!tj.engaged) continue;
        c[r++] = m.touchState(tj, q).gap - tj.roller;
    }
    for (const RackMesh& rm : m.racks) {                       // M16
        const SlideGeom g = slideGeom(m, rm.geom, q);
        const Gear& gr = m.gears[(size_t)rm.gear];
        const Vec2 v = g.pA - g.pB;
        const double thP = q[(size_t)g.oA], thR = g.oB >= 0 ? q[(size_t)g.oB] : 0.0;
        if (rm.tangent) c[r++] = crossz(g.u, v) - rm.side * gr.radius;
        c[r++] = dot(g.u, v) + rm.side * gr.radius * (thP - thR) - rm.K;
    }
    for (const Belt& b : m.belts) {
        for (size_t i = 0; i + 1 < b.gears.size(); ++i) {
            const Gear& ga = m.gears[(size_t)b.gears[i]];
            const Gear& gb = m.gears[(size_t)b.gears[i + 1]];
            c[r++] = b.sign[i] * ga.radius * q[(size_t)m.bodies[(size_t)ga.body].qOffset] -
                     b.sign[i + 1] * gb.radius * q[(size_t)m.bodies[(size_t)gb.body].qOffset] - b.K[i];
        }
    }
    for (const LinearDrive& ld : m.linDrives) {                // M14: travel along the rail follows s0 + rate t
        if (ld.isForce) continue;
        const SlideGeom g = slideGeom(m, m.slides[(size_t)ld.slide], q);
        c[r++] = dot(g.u, g.pA - g.pB) - (ld.s0 + ld.rate * t);
    }
}

namespace {

// where assembleJacobian puts what it computes: into a dense row-major matrix ...
struct DenseSink {
    std::vector<double>& J;
    int cols;
    void add(int r, int c, double v) { J[(size_t)r * cols + c] += v; }
    void touch(const ConstraintSystem& cs, const TouchJoint& tj, const std::vector<double>& q, int r) {
        cs.touchRow(tj, q, &J[(size_t)r * cols]);
    }
};

// ... or into sparse rows: the entries of a row are collected as they come and then summed per column in the order
// they came, which is what `+=` into a zeroed dense cell does
struct SparseSink {
    std::vector<std::vector<std::pair<int, double>>> raw;
    std::vector<double> scratch;      // a dense row for the touch rows, cleared after each use
    void add(int r, int c, double v) { raw[(size_t)r].emplace_back(c, v); }
    void touch(const ConstraintSystem& cs, const TouchJoint& tj, const std::vector<double>& q, int r) {
        const Body& bp = cs.m.bodies[(size_t)tj.body];
        const Body& bh = cs.m.bodies[(size_t)cs.m.profiles[(size_t)tj.profile].body];
        if (scratch.size() < (size_t)cs.nq) scratch.assign((size_t)cs.nq, 0.0);
        cs.touchRow(tj, q, scratch.data());
        for (const Body* b : {&bp, &bh}) {
            if (b->qOffset < 0) continue;
            for (int k = 0; k < 3; ++k) {
                double& v = scratch[(size_t)b->qOffset + (size_t)k];
                if (v != 0.0) raw[(size_t)r].emplace_back(b->qOffset + k, v);
                v = 0.0;
            }
            if (&bp == &bh) break;
        }
    }
};

} // namespace

template <class Sink>
void ConstraintSystem::assembleJacobian(const std::vector<double>& q, Sink& S) const {
    int r = 0;
    for (const PinJoint& p : m.pins) {
        for (int side = 0; side < 2; ++side) {
            const int bi = side == 0 ? p.bodyA : p.bodyB;
            const int ai = side == 0 ? p.anchA : p.anchB;
            const Body& b = m.bodies[bi];
            if (b.qOffset >= 0) {
                const double sgn = side == 0 ? 1.0 : -1.0;
                const Vec2 rl = rotate(b.anchors[ai].local, q[b.qOffset + 0]);
                S.add(r,     b.qOffset + 0, sgn * (-rl.y));
                S.add(r + 1, b.qOffset + 0, sgn * ( rl.x));
                S.add(r,     b.qOffset + 1, sgn * 1.0);
                S.add(r + 1, b.qOffset + 2, sgn * 1.0);
            }
        }
        r += 2;
    }
    for (const SlideJoint& s : m.slides) {
        const SlideGeom g = slideGeom(m, s, q);
        const Vec2 v = g.pA - g.pB;
        // offset row: d crossz(u, pA - pB)
        S.add(r, g.oA + 0, crossz(g.u, perp(g.rA)));
        S.add(r, g.oA + 1, -g.u.y);
        S.add(r, g.oA + 2,  g.u.x);
        if (g.oB >= 0) {                                    // M14: the rail's body moves too
            S.add(r, g.oB + 0, crossz(perp(g.u), v) - crossz(g.u, perp(g.rB)));
            S.add(r, g.oB + 1,  g.u.y);
            S.add(r, g.oB + 2, -g.u.x);
        }
        ++r;
        if (s.lockDir) {
            // dir-lock row: d crossz(R(thA) guideDir, u)
            S.add(r, g.oA + 0, crossz(perp(g.dirG), g.u));
            if (g.oB >= 0) S.add(r, g.oB + 0, crossz(g.dirG, perp(g.u)));
            ++r;
        }
    }
    for (const Drive& d : m.drives) {
        if (!d.enabled) continue;
        const Body& b = m.bodies[d.body];
        if (b.qOffset >= 0) S.add(r, b.qOffset + 0, 1.0);
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
            S.add(r, ba.qOffset + 0, ga.radius + K * crossz(d, wA));
            S.add(r, ba.qOffset + 1, -K * d.y);
            S.add(r, ba.qOffset + 2,  K * d.x);
        }
        if (bb.qOffset >= 0) {
            S.add(r, bb.qOffset + 0, s * gb.radius - K * crossz(d, wB));
            S.add(r, bb.qOffset + 1,  K * d.y);
            S.add(r, bb.qOffset + 2, -K * d.x);
        }
        ++r;

        if (mm.tangent) {
            if (ba.qOffset >= 0) {
                S.add(r, ba.qOffset + 0, -2.0 * dot(d, wA));
                S.add(r, ba.qOffset + 1, -2.0 * d.x);
                S.add(r, ba.qOffset + 2, -2.0 * d.y);
            }
            if (bb.qOffset >= 0) {
                S.add(r, bb.qOffset + 0,  2.0 * dot(d, wB));
                S.add(r, bb.qOffset + 1,  2.0 * d.x);
                S.add(r, bb.qOffset + 2,  2.0 * d.y);
            }
            ++r;
        }
    }
    for (const PolarMesh& pm : m.polarMeshes) {
        const Profile& pa = m.profiles[(size_t)pm.profA];
        const Profile& pb = m.profiles[(size_t)pm.profB];
        const int oA = m.bodies[(size_t)pa.body].qOffset;
        const int oB = m.bodies[(size_t)pb.body].qOffset;
        double rho, drho;
        pm.phase(*pa.curve, pm.psi - q[(size_t)oA], rho, drho);
        S.add(r, oA, rho);        // d/dthA of -F(psi - thA)
        S.add(r, oB, 1.0);
        ++r;
    }
    for (const TouchJoint& tj : m.touches) {
        if (!tj.engaged) continue;
        S.touch(*this, tj, q, r);
        ++r;
    }
    for (const RackMesh& rm : m.racks) {                       // M16
        const SlideGeom g = slideGeom(m, rm.geom, q);
        const Gear& gr = m.gears[(size_t)rm.gear];
        const Vec2 v = g.pA - g.pB;
        if (rm.tangent) {                                   // tangency: the slide offset row
            S.add(r, g.oA + 0, crossz(g.u, perp(g.rA)));
            S.add(r, g.oA + 1, -g.u.y);
            S.add(r, g.oA + 2,  g.u.x);
            if (g.oB >= 0) {
                S.add(r, g.oB + 0, crossz(perp(g.u), v) - crossz(g.u, perp(g.rB)));
                S.add(r, g.oB + 1,  g.u.y);
                S.add(r, g.oB + 2, -g.u.x);
            }
            ++r;
        }
        // rolling: the linear row u.v plus side*r*(thP - thR)
        const double sr = rm.side * gr.radius;
        S.add(r, g.oA + 0, dot(g.u, perp(g.rA)) + sr);
        S.add(r, g.oA + 1, g.u.x);
        S.add(r, g.oA + 2, g.u.y);
        if (g.oB >= 0) {
            S.add(r, g.oB + 0, dot(perp(g.u), v) - dot(g.u, perp(g.rB)) - sr);
            S.add(r, g.oB + 1, -g.u.x);
            S.add(r, g.oB + 2, -g.u.y);
        }
        ++r;
    }
    for (const Belt& b : m.belts) {
        for (size_t i = 0; i + 1 < b.gears.size(); ++i) {
            const Gear& ga = m.gears[(size_t)b.gears[i]];
            const Gear& gb = m.gears[(size_t)b.gears[i + 1]];
            S.add(r, m.bodies[(size_t)ga.body].qOffset, b.sign[i] * ga.radius);
            S.add(r, m.bodies[(size_t)gb.body].qOffset, -b.sign[i + 1] * gb.radius);
            ++r;
        }
    }
    for (const LinearDrive& ld : m.linDrives) {                // M14: d (u . (pA - pB))
        if (ld.isForce) continue;
        const SlideGeom g = slideGeom(m, m.slides[(size_t)ld.slide], q);
        const Vec2 v = g.pA - g.pB;
        S.add(r, g.oA + 0, dot(g.u, perp(g.rA)));
        S.add(r, g.oA + 1, g.u.x);
        S.add(r, g.oA + 2, g.u.y);
        if (g.oB >= 0) {
            S.add(r, g.oB + 0, dot(perp(g.u), v) - dot(g.u, perp(g.rB)));
            S.add(r, g.oB + 1, -g.u.x);
            S.add(r, g.oB + 2, -g.u.y);
        }
        ++r;
    }
}

void ConstraintSystem::jacobian(const std::vector<double>& q, std::vector<double>& J) const {
    const int rows = rowCount();
    J.assign((size_t)rows * nq, 0.0);
    DenseSink S{J, nq};
    assembleJacobian(q, S);
}

void ConstraintSystem::jacobianSparse(const std::vector<double>& q, std::vector<SparseRow>& J) const {
    SparseSink S;
    S.raw.assign((size_t)rowCount(), {});
    assembleJacobian(q, S);
    J.assign(S.raw.size(), SparseRow());
    for (size_t r = 0; r < S.raw.size(); ++r) {
        auto& e = S.raw[r];
        std::stable_sort(e.begin(), e.end(), [](const std::pair<int, double>& x, const std::pair<int, double>& y) { return x.first < y.first; });
        for (size_t i = 0; i < e.size();) {
            const int c = e[i].first;
            double v = 0.0;
            for (; i < e.size() && e[i].first == c; ++i) v += e[i].second;      // the dense matrix summed them into 0.0 in this order
            if (v != 0.0) { J[r].col.push_back(c); J[r].val.push_back(v); }
        }
    }
}

void ConstraintSystem::touchRow(const TouchJoint& tj, const std::vector<double>& q,
                                double* row) const {
    const ContactState cs = m.touchState(tj, q);
    const Profile& pr = m.profiles[(size_t)tj.profile];
    const Body& bp = m.bodies[(size_t)tj.body];
    const Body& bh = m.bodies[(size_t)pr.body];
    const Vec2 n = cs.normal;
    if (bp.qOffset >= 0) {
        const Vec2 rl = rotate(bp.anchors[(size_t)tj.anchor].local, q[bp.qOffset + 0]);
        row[bp.qOffset + 0] += dot(n, perp(rl));
        row[bp.qOffset + 1] += n.x;
        row[bp.qOffset + 2] += n.y;
    }
    if (bh.qOffset >= 0) {
        row[bh.qOffset + 0] += -dot(n, perp(cs.footArm));
        row[bh.qOffset + 1] += -n.x;
        row[bh.qOffset + 2] += -n.y;
    }
}

bool ConstraintSystem::solve(std::vector<double>& q, double t, int& itersOut,
                             double* maxResidualOut) const {
    // Deterministic multi-start: the incoming guess first, then two tilted
    // restarts. Cheap insurance against singular seeds (e.g. dead points);
    // costs nothing in the warm-started steady state.
    const std::vector<double> q0 = q;
    static const double kTilt[2] = {0.37, -0.53};
    std::vector<double> best;
    double bestRes = 1e300;
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (attempt > 0) {
            q = q0;
            // signed arithmetic: (i % 3) is unsigned, and 0 - 1 wraps to 2^64 - 1
            for (size_t i = 0; i < q.size(); ++i)
                q[i] += kTilt[attempt - 1] * (0.5 + 0.25 * ((int)(i % 3) - 1));
        }
        double res = 0.0;
        if (solveAttempt(q, t, itersOut, &res)) {
            if (maxResidualOut) *maxResidualOut = res;
            return true;
        }
        if (res < bestRes) { bestRes = res; best = q; }
    }
    q = best;                                  // the closest attempt, as promised
    if (maxResidualOut) *maxResidualOut = bestRes;
    return false;
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
    std::vector<SparseRow> J;
    std::vector<double> g, delta;
    Skyline A, L;

    for (int iter = 0; iter < 100; ++iter) {
        const double mx = maxAbs(c);
        if (mx < 1e-10) {
            itersOut = iter;
            if (maxResidualOut) *maxResidualOut = mx;
            return true;
        }

        jacobianSparse(q, J);
        const int rows = (int)c.size();

        // normal equations: A = J^T J + lam^2 I, g = J^T c. A is banded in the order of the unknowns (a row couples only the
        // bodies it joins), so it is kept as a skyline: row k from the first column a row it shares with k reaches back to.
        // Two unknowns are neighbours when a row joins them. They are numbered in the order the bodies were declared; when that
        // makes the envelope large (a mechanism declared out of order) they are renumbered by reverse Cuthill-McKee.
        struct OrderCache { unsigned long long hash = 0; bool have = false; std::vector<int> perm; };
        thread_local OrderCache cache;
        const unsigned long long h = patternHash(J);
        if (!cache.have || cache.hash != h) {
            std::vector<std::vector<int>> adj((size_t)nq);
            for (int i = 0; i < rows; ++i)
                for (int a : J[(size_t)i].col)
                    for (int b : J[(size_t)i].col) if (a != b) adj[(size_t)a].push_back(b);
            cache.perm = envelopeOrdering(adj);
            cache.hash = h;
            cache.have = true;
        }
        const std::vector<int>& perm = cache.perm;
        std::vector<int> iperm;
        if (!perm.empty()) {
            iperm.assign((size_t)nq, 0);
            for (int n = 0; n < nq; ++n) iperm[(size_t)perm[(size_t)n]] = n;
        }
        auto idx = [&](int i) { return perm.empty() ? i : iperm[(size_t)i]; };
        std::vector<int> first((size_t)nq);
        for (int k = 0; k < nq; ++k) first[(size_t)idx(k)] = idx(k);
        for (int i = 0; i < rows; ++i)
            for (int a : J[(size_t)i].col)
                for (int b : J[(size_t)i].col) first[(size_t)idx(a)] = std::min(first[(size_t)idx(a)], idx(b));
        A.init(first);
        g.assign(nq, 0.0);
        for (int i = 0; i < rows; ++i) {
            const SparseRow& Ji = J[(size_t)i];
            for (size_t x = 0; x < Ji.col.size(); ++x) {
                const int j = Ji.col[x];
                const double Jij = Ji.val[x];
                g[(size_t)j] += Jij * c[(size_t)i];
                for (size_t y = x; y < Ji.col.size(); ++y) {
                    const int ra = idx(Ji.col[y]), rb = idx(j);
                    A.at(std::max(ra, rb), std::min(ra, rb)) += Jij * Ji.val[y];
                }
            }
        }
        for (int i = 0; i < nq; ++i) A.at(i, i) += lam * lam;      // (a diagonal entry is the same in either numbering)

        // Newton/LM step: solve (J^T J + lam^2 I) delta = -J^T c
        std::vector<double> rhs(nq);
        for (int j = 0; j < nq; ++j) rhs[(size_t)idx(j)] = -g[(size_t)j];
        if (!skylineFactor(A, L)) {
            lam *= 10.0;
            if (lam > 1e14) break;
            continue;
        }
        std::vector<double> sol;
        skylineSolve(L, rhs, sol);
        delta.assign((size_t)nq, 0.0);
        for (int j = 0; j < nq; ++j) delta[(size_t)j] = sol[(size_t)idx(j)];

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
