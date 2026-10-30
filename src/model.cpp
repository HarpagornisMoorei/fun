// mech — model.cpp: AST -> runtime model, with static checks and desugaring
#include "model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>

namespace mech {

namespace {
void rhoOf(const ProfileCurve& A, double d, double phi, double& rho, double& drho);
void buildPhaseTable(PolarMesh& pm, const ProfileCurve& A);
bool conjugateRoot(const ProfileCurve& known, double phiKnown, const ProfileCurve& other,
                   double d, double& phiOther, double& defect);
}

double PolarMesh::phase(const ProfileCurve& A, double phi, double& rho, double& drho) const {
    const double full = 2.0 * kPi;
    const double turns = std::floor(phi / full);
    const double u = phi - full * turns;                 // in [0, 2 pi)
    const int N = (int)Fnodes.size() - 1;
    const double h = full / (double)N;
    int i = (int)(u / h);
    if (i > N - 1) i = N - 1;
    const double t = (u - (double)i * h) / h;
    const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    // quintic Hermite basis: values, first and second derivatives at both ends
    const double H0 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5;
    const double H1 = t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5;
    const double H2 = 0.5 * t2 - 1.5 * t3 + 1.5 * t4 - 0.5 * t5;
    const double H3 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
    const double H4 = -4.0 * t3 + 7.0 * t4 - 3.0 * t5;
    const double H5 = 0.5 * t3 - t4 + 0.5 * t5;
    const size_t k = (size_t)i;
    const double F = Fnodes[k] * H0 + h * rhoNodes[k] * H1 + h * h * drhoNodes[k] * H2 +
                     Fnodes[k + 1] * H3 + h * rhoNodes[k + 1] * H4 + h * h * drhoNodes[k + 1] * H5;
    rhoOf(A, d, phi, rho, drho);
    return F + turns * Ftot;
}

int Model::bodyIndex(const std::string& name) const {
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].name == name) return (int)i;
    return -1;
}

int Model::anchorIndex(int bodyIdx, const std::string& anchorName) const {
    const Body& b = bodies[bodyIdx];
    for (size_t i = 0; i < b.anchors.size(); ++i)
        if (b.anchors[i].name == anchorName) return (int)i;
    return -1;
}

int Model::gearIndex(const std::string& name) const {
    for (size_t i = 0; i < gears.size(); ++i)
        if (gears[i].name == name) return (int)i;
    return -1;
}

int Model::profileIndex(const std::string& name) const {
    for (size_t i = 0; i < profiles.size(); ++i)
        if (profiles[i].name == name) return (int)i;
    return -1;
}

ContactState Model::touchState(const TouchJoint& t, const std::vector<double>& q) const {
    const Profile& pr = profiles[(size_t)t.profile];
    const Body& host = bodies[(size_t)pr.body];
    const Vec2 pw = worldAnchor(t.body, t.anchor, q);
    const Vec2 origin = host.anchors[(size_t)pr.hostAnchor].local;
    double th = 0.0;
    Vec2 tr{0.0, 0.0};
    if (host.qOffset >= 0) {
        th = q[(size_t)host.qOffset + 0];
        tr = {q[(size_t)host.qOffset + 1], q[(size_t)host.qOffset + 2]};
    }
    // the point in the profile's own frame (anchor-centered, body-aligned)
    const Vec2 u = rotate(pw - tr, -th) - origin;
    const ProfileCurve::Foot f = pr.curve->nearest(u);
    ContactState cs;
    cs.gap = f.signedDist;
    cs.s = f.s;
    cs.footLocal = f.point;
    cs.footArm = rotate(origin + f.point, th);
    cs.foot = cs.footArm + tr;
    cs.normal = rotate(f.normal, th);
    cs.tangent = rotate(normalize(f.tangent), th);
    cs.kappa = f.curvature;
    return cs;
}

Vec2 Model::worldAnchor(int bodyIdx, int anchorIdx, const std::vector<double>& q) const {
    const Body& b = bodies[bodyIdx];
    Vec2 l = b.anchors[anchorIdx].local;
    if (b.qOffset < 0) return l;  // ground: local == world
    const double th = q[b.qOffset + 0];
    const Vec2 t{q[b.qOffset + 1], q[b.qOffset + 2]};
    return rotate(l, th) + t;
}

Vec2 Model::gearCenter(int gearIdx, const std::vector<double>& q) const {
    const Gear& g = gears[gearIdx];
    return worldAnchor(g.body, g.hostAnchor, q);
}

DofReport Model::dofReport() const {
    DofReport r;
    r.nBodies = (int)bodies.size();
    r.nPins = (int)pins.size();
    r.nSlides = (int)slides.size();
    r.nMeshes = (int)meshes.size();
    r.nTangentMeshes = 0;
    for (const MeshJoint& mm : meshes) if (mm.tangent) ++r.nTangentMeshes;
    r.nGears = (int)gears.size();
    r.nProfiles = (int)profiles.size();
    r.nTouches = (int)touches.size();
    r.nPolarMeshes = (int)polarMeshes.size();
    r.nDrives = 0;
    for (const Drive& d : drives) if (d.enabled) ++r.nDrives;
    r.nTorques = (int)torques.size();
    r.nSprings = (int)springs.size();
    r.nWeights = (int)weights.size();
    r.nModes = (int)modes.size();
    r.nHolds = 0;
    for (const Mode& mo : modes) {
        for (size_t hi = 0; hi < mo.holdBodies.size(); ++hi) {
            if (drives[(size_t)mo.holdDrive[hi]].enabled) ++r.nHolds;
        }
    }
    // Planar Grubler-Kutzbach: pins and slides each remove 2 DOF; a mesh's
    // roll-without-slip row removes 1; a tangent mesh's contact row removes
    // one more; a (bilateral) touch's gap row removes 1. Springs, torque drives and weights are forces, not
    // constraints. A mode's active holds are rate-0 drive rows: they remove
    // one DOF each while engaged (the report shows the initial states).
    const int j = r.nPins + r.nSlides;
    r.freeDof = 3 * (r.nBodies - 1) - 2 * j - r.nMeshes - r.nTangentMeshes - r.nTouches -
                 r.nPolarMeshes;
    r.residualDof = r.freeDof - r.nDrives;
    return r;
}

bool Model::bodyDriven(int bodyIdx) const {
    for (const Drive& d : drives)
        if (d.body == bodyIdx && d.enabled) return true;
    return false;
}

double Model::driveTheta0(int bodyIdx) const {
    for (const Drive& d : drives)
        if (d.body == bodyIdx && d.enabled) return d.theta0Rad;
    return 0.0;
}

double Model::seedAngleFor(int bodyIdx) const {
    if (bodyDriven(bodyIdx)) return driveTheta0(bodyIdx);
    if (bodies[bodyIdx].hasSeed) return bodies[bodyIdx].seedAngle;
    // Deterministic tilt for undriven bodies: a freshly assembled mechanism
    // must not sit on a collinear dead-point singularity (rank-deficient J).
    static const double kSeed[5] = {0.42, -0.31, 0.26, -0.44, 0.35};
    return kSeed[bodyIdx % 5];
}

void Model::placeBody(std::vector<double>& q, int bodyIdx, int viaAnchor,
                      Vec2 target, double theta) const {
    const Body& b = bodies[bodyIdx];
    const Vec2 local = b.anchors[viaAnchor].local;
    const Vec2 t = target - rotate(local, theta);
    q[b.qOffset + 0] = theta;
    q[b.qOffset + 1] = t.x;
    q[b.qOffset + 2] = t.y;
}

std::vector<std::string> Model::assemble(std::vector<double>& q) const {
    q.assign(nq(), 0.0);
    const size_t n = bodies.size();
    std::vector<char> placed(n, 0);
    placed[0] = 1;  // ground at identity

    std::vector<std::string> warnings;
    std::vector<char> pinConsumed(pins.size(), 0);
    std::vector<char> slideConsumed(slides.size(), 0);

    // Fixpoint pass over pins and slides. A slide places its guide body on
    // the rail, oriented along the rail; a pin places the unplaced side on
    // the placed side's anchor. Loop-closure residuals belong to the solver.
    bool progress = true;
    while (progress) {
        progress = false;
        for (size_t k = 0; k < pins.size(); ++k) {
            if (pinConsumed[k]) continue;
            const PinJoint& p = pins[k];
            const bool a = placed[p.bodyA], b = placed[p.bodyB];
            if (a && !b) {
                const double th = seedAngleFor(p.bodyB);
                placeBody(q, p.bodyB, p.anchB, worldAnchor(p.bodyA, p.anchA, q), th);
                placed[p.bodyB] = 1; pinConsumed[k] = 1; progress = true;
            } else if (!a && b) {
                const double th = seedAngleFor(p.bodyA);
                placeBody(q, p.bodyA, p.anchA, worldAnchor(p.bodyB, p.anchB, q), th);
                placed[p.bodyA] = 1; pinConsumed[k] = 1; progress = true;
            } else if (a && b) {
                pinConsumed[k] = 1;
            }
        }
        for (size_t k = 0; k < slides.size(); ++k) {
            if (slideConsumed[k]) continue;
            const SlideJoint& s = slides[k];
            // rail side is ground (enforced at build) -> always placed
            if (!placed[s.guideBody]) {
                const double th = std::atan2(s.railDir.y, s.railDir.x);
                placeBody(q, s.guideBody, s.guideAnchor, s.railPos, th);
                placed[s.guideBody] = 1; slideConsumed[k] = 1; progress = true;
            } else {
                slideConsumed[k] = 1;
            }
        }
    }

    // M6d: a non-circular pair has two mirrored solutions of its row for a given
    // angle of the first shaft, only one of which is tangent contact. Seed the
    // second shaft on the tangent branch (the one whose log-derivatives agree),
    // unless the author drove it.
    for (const PolarMesh& pm : polarMeshes) {
        const Profile& pa = profiles[(size_t)pm.profA];
        const Profile& pb = profiles[(size_t)pm.profB];
        const bool drivenA = bodyDriven(pa.body), drivenB = bodyDriven(pb.body);
        if (drivenA && drivenB) continue;
        const bool seedB = !drivenB;      // else the driven B fixes A
        const Profile& kn = seedB ? pa : pb;
        const Profile& ot = seedB ? pb : pa;
        const double thKnown = q[(size_t)bodies[(size_t)kn.body].qOffset];
        // polar angle of the contact point on the known side: A sees the other
        // shaft in direction psi, B sees A in direction psi + pi
        const double phiKnown = seedB ? pm.psi - thKnown : pm.psi + kPi - thKnown;
        double phiOther, defect;
        if (conjugateRoot(*kn.curve, phiKnown, *ot.curve, pm.d, phiOther, defect)) {
            const double thOther = seedB ? pm.psi + kPi - phiOther : pm.psi - phiOther;
            placeBody(q, ot.body, ot.hostAnchor, seedB ? pm.cB : pm.cA, thOther);
        }
        // the phase constant: wherever assembly landed is the touching pose
        double rho, drho;
        pm.thB0 = q[(size_t)bodies[(size_t)pb.body].qOffset] -
                  pm.phase(*pa.curve, pm.psi - q[(size_t)bodies[(size_t)pa.body].qOffset], rho, drho);
    }

    // Bodies floating free of ground are an error in kinematic mode (their
    // motion is indeterminate); in dynamic mode they are simply free bodies
    // acted on by forces, which is legal (a falling weight, a flywheel).
    // A body hosting a gear whose mesh reaches ground is connected too —
    // tangent meshes are the whole joint (carrierless planets).
    if (!world.dynamic) {
        std::vector<char> meshLinked(n, 0);
        for (const MeshJoint& mm : meshes) {
            const int ha = gears[mm.gearA].body, hb = gears[mm.gearB].body;
            if (placed[ha] || placed[hb]) { meshLinked[ha] = 1; meshLinked[hb] = 1; }
        }
        // M6: a contact links the touching body to the profile's host
        for (const TouchJoint& tj : touches) {
            const int hb = profiles[(size_t)tj.profile].body;
            if (placed[tj.body] || placed[hb]) { meshLinked[tj.body] = 1; meshLinked[hb] = 1; }
        }
        for (size_t i = 1; i < n; ++i) {
            if (!placed[i] && !meshLinked[i]) {
                warnings.push_back("body '" + bodies[i].name +
                                   "' is not connected to ground; it stays frozen at its initial pose");
            }
        }
    }

    // M6b: a follower whose roller differs from the one the cam was cut for
    // will not follow the diagram
    for (const TouchJoint& tj : touches) {
        const Profile& pr = profiles[(size_t)tj.profile];
        if (pr.fromCam && std::fabs(tj.roller - pr.designRoller) > 1e-12) {
            char buf[256];
            std::snprintf(buf, sizeof buf,
                "touch '%s.%s' uses roller %g m but cam '%s' was cut for %g m — the "
                "follower will not follow the displacement diagram",
                bodies[(size_t)tj.body].name.c_str(),
                bodies[(size_t)tj.body].anchors[(size_t)tj.anchor].name.c_str(),
                tj.roller, pr.name.c_str(), pr.designRoller);
            warnings.push_back(buf);
        }
    }

    // Detect mesh type from the assembled geometry (external vs internal),
    // then capture the roll phase: wherever assembly landed is the engaged
    // configuration (tooth phase is not modelled).
    for (const MeshJoint& mm : meshes) {
        const Gear& ga = gears[mm.gearA];
        const Gear& gb = gears[mm.gearB];
        const Vec2 ca = gearCenter(mm.gearA, q);
        const Vec2 cb = gearCenter(mm.gearB, q);
        const Vec2 d = cb - ca;
        const double dist = vlen(d);
        const double dExt = std::fabs(dist - (ga.radius + gb.radius));
        const double dInt = std::fabs(dist - std::fabs(ga.radius - gb.radius));
        mm.sgn = (dInt < dExt) ? -1 : 1;

        const Body& ba = bodies[ga.body];
        const Body& bb = bodies[gb.body];
        const double thA = ba.qOffset >= 0 ? q[ba.qOffset + 0] : 0.0;
        const double thB = bb.qOffset >= 0 ? q[bb.qOffset + 0] : 0.0;
        const double s = mm.sgn >= 0 ? 1.0 : -1.0;
        if (dist > 1e-12) {
            mm.psiTrack = std::atan2(d.y, d.x);
            mm.psiInit = true;
        } else {
            mm.psiTrack = 0.0;
            mm.psiInit = false;
        }
        mm.rollConst = ga.radius * thA + s * gb.radius * thB
                       - (ga.radius + s * gb.radius) * mm.psiTrack;
    }

    // Springs whose rest length was not given capture the assembled length.
    for (const SpringJoint& sp : springs) {
        if (sp.rest < 0.0) {
            const Vec2 pa = worldAnchor(sp.bodyA, sp.anchA, q);
            const Vec2 pb = worldAnchor(sp.bodyB, sp.anchB, q);
            sp.rest = vlen(pb - pa);
        }
    }

    // M6c: initial contact state. A kinematic contact is always bilateral. In
    // a dynamic mechanism a contact starts engaged only if the point is
    // already on the curve where it was placed; otherwise the follower starts
    // in flight and the first touch-down is a real impact.
    for (const TouchJoint& tj : touches) {
        tj.engaged = !world.dynamic ||
                     std::fabs(touchState(tj, q).gap - tj.roller) <= 1e-6;
    }

    // M4: dials without an explicit center/radius capture them from the
    // assembled hands (auto-center on the average mount, auto-size to the
    // longest hand).
    for (const Dial& d : dials) {
        if (!d.hasCenter && !hands.empty()) {
            Vec2 avg{0.0, 0.0};
            for (const Hand& h : hands) {
                avg += worldAnchor(h.body, h.mountAnchor, q);
            }
            d.center = (1.0 / (double)hands.size()) * avg;
        }
        if (d.radius < 0.0 && !hands.empty()) {
            double mx = 0.0;
            for (const Hand& h : hands) mx = std::max(mx, h.length);
            d.radius = 1.2 * mx;
        }
    }

    // M5: hybrid elements enter their initial state from the assembled
    // configuration — hold targets are captured (a hold pins the body exactly
    // where assembly left it) and advance origins are recorded. Weight
    // drives rewind to a fresh cord. Re-running assemble (viewer reset)
    // resets the discrete state deterministically.
    for (const Mode& mo : modes) {
        mo.current = 0;
        mo.fires = 0;
        mo.entryTheta.assign(mo.guardBodies.size(), 0.0);
        for (size_t gi = 0; gi < mo.guardBodies.size(); ++gi) {
            const int o = bodies[(size_t)mo.guardBodies[gi]].qOffset;
            mo.entryTheta[gi] = o >= 0 ? q[(size_t)o + 0] : 0.0;
        }
        for (size_t hi = 0; hi < mo.holdBodies.size(); ++hi) {
            const Drive& d = drives[(size_t)mo.holdDrive[hi]];
            const int o = bodies[(size_t)mo.holdBodies[hi]].qOffset;
            d.enabled = mo.stateHolds(mo.current, mo.holdBodies[hi]);
            if (d.enabled) d.theta0Rad = q[(size_t)o + 0];
        }
    }
    for (const WeightDrive& w : weights) {
        w.paid = 0.0;
        w.exhausted = false;
    }
    return warnings;
}

std::vector<std::string> Model::validateAssembly(const std::vector<double>& q) const {
    std::vector<std::string> errs;
    for (const MeshJoint& mm : meshes) {
        const Gear& ga = gears[mm.gearA];
        const Gear& gb = gears[mm.gearB];
        const Vec2 ca = gearCenter(mm.gearA, q);
        const Vec2 cb = gearCenter(mm.gearB, q);
        const double d = vlen(cb - ca);
        const double s = mm.sgn >= 0 ? 1.0 : -1.0;
        const double want = ga.radius + s * gb.radius;
        if (std::fabs(d - std::fabs(want)) > 1e-6 * std::max(1.0, std::fabs(want))) {
            char buf[420];
            std::snprintf(buf, sizeof buf,
                "mesh '%s' <-> '%s' (%s): center distance %.6f, but pitch radii need "
                "%.6f (off by %.6f) — move the host anchors or adjust teeth/module",
                ga.name.c_str(), gb.name.c_str(),
                mm.sgn < 0 ? "internal" : "external", d, std::fabs(want),
                std::fabs(d - std::fabs(want)));
            errs.push_back(buf);
        }
    }
    for (const PolarMesh& pm : polarMeshes) {
        const Profile& pa = profiles[(size_t)pm.profA];
        const Profile& pb = profiles[(size_t)pm.profB];
        double rA, rAp, rA2, rB, rBp, rB2;
        pa.curve->polarEval(pm.psi - q[(size_t)bodies[(size_t)pa.body].qOffset], rA, rAp, rA2);
        pb.curve->polarEval(pm.psi + kPi - q[(size_t)bodies[(size_t)pb.body].qOffset], rB, rBp, rB2);
        const double touch = rA + rB - pm.d;
        const double defect = std::fabs(rAp * rB - rA * rBp) / (rA * rB);
        if (std::fabs(touch) > 1e-6 || defect > 1e-3) {
            char buf[420];
            std::snprintf(buf, sizeof buf,
                "profiles '%s' and '%s' are not in rolling contact at the initial pose "
                "(contact off the line of centers by %.3g m, tangency defect %.3g) — the "
                "shafts' starting angles are inconsistent; leave one shaft undriven (and "
                "without 'from') so assembly can place it, or give both the matching angles",
                pa.name.c_str(), pb.name.c_str(), touch, defect);
            errs.push_back(buf);
        }
    }
    for (const TouchJoint& tj : touches) {
        if (!tj.engaged) continue;      // a follower starting in flight is not an error
        const Profile& pr = profiles[(size_t)tj.profile];
        const double err = touchState(tj, q).gap - tj.roller;
        if (std::fabs(err) > 1e-6) {
            char buf[320];
            std::snprintf(buf, sizeof buf,
                "touch '%s.%s' on profile '%s': gap %.3g m at the initial pose — "
                "the contact did not close (is the point near the curve?)",
                bodies[(size_t)tj.body].name.c_str(),
                bodies[(size_t)tj.body].anchors[(size_t)tj.anchor].name.c_str(),
                pr.name.c_str(), err);
            errs.push_back(buf);
        }
    }
    return errs;
}

namespace {

Vec2 normalizeDir(Vec2 d, int line, const std::string& anchorName) {
    const double l = vlen(d);
    if (l < 1e-12) {
        throw SemanticError{line, "anchor '" + anchorName +
            "' has a zero-length direction"};
    }
    return {d.x / l, d.y / l};
}

// Default areal density for auto-sized gear discs (kg/m^2) and linear
// density for auto-sized rods (kg/m). Chosen so plain geometric files get
// sensible physics without a single `mass:` anywhere.
const double kRodDensity = 1.0;
const double kGearArealDensity = 1.0;

struct MassContribution {
    double m;
    Vec2 com;      // in body frame
    double I;      // about this contribution's own com
};


// ---------------------------------------------------------------- M6d conjugacy

// Given one profile's polar angle, find where the other profile can meet it on
// the line of centers: r_other(phiOther) = d - r_known(phiKnown). Of the (up to
// two mirrored) roots, return the one whose log-derivative agrees best with the
// known side — the branch on which the curves are actually tangent. Returns
// false when no root exists (the pair cannot touch there); `defect` is then 1e9.
bool conjugateRoot(const ProfileCurve& known, double phiKnown, const ProfileCurve& other,
                   double d, double& phiOther, double& defect) {
    double rK, rKp, rK2;
    known.polarEval(phiKnown, rK, rKp, rK2);
    const double target = d - rK;
    defect = 1e9;
    if (!(target > 0.0)) return false;

    const int M = 1024;
    auto f = [&](double phi) {
        double r, rp, r2;
        other.polarEval(phi, r, rp, r2);
        return r - target;
    };
    auto consider = [&](double phi) {
        double r, rp, r2;
        other.polarEval(phi, r, rp, r2);
        const double dft = std::fabs(rKp * r - rK * rp) / (rK * r);
        if (dft < defect) { defect = dft; phiOther = phi; }
    };
    double prevPhi = 0.0, prevF = f(0.0);
    if (std::fabs(prevF) < 1e-12) consider(0.0);
    for (int j = 1; j <= M; ++j) {
        const double phi = 2.0 * kPi * (double)j / (double)M;
        const double fv = f(phi);
        if (std::fabs(fv) < 1e-12) {
            consider(phi);
        } else if ((prevF < 0.0) != (fv < 0.0) && std::fabs(prevF) >= 1e-12) {
            double lo = prevPhi, hi = phi, flo = prevF;
            for (int it = 0; it < 60; ++it) {
                const double mid = 0.5 * (lo + hi), fm = f(mid);
                if ((fm < 0.0) == (flo < 0.0)) { lo = mid; flo = fm; } else hi = mid;
            }
            consider(0.5 * (lo + hi));
        }
        prevPhi = phi;
        prevF = fv;
    }
    return defect < 1e8;
}


// rho = r/(d - r) and its derivative in phi, from profile A's polar form
void rhoOf(const ProfileCurve& A, double d, double phi, double& rho, double& drho) {
    double r, rp, r2;
    A.polarEval(phi, r, rp, r2);
    const double den = d - r;
    rho = r / den;
    drho = d * rp / (den * den);
}

void buildPhaseTable(PolarMesh& pm, const ProfileCurve& A) {
    const int N = 2048;
    const double h = 2.0 * kPi / (double)N;
    pm.Fnodes.assign((size_t)N + 1, 0.0);
    pm.rhoNodes.assign((size_t)N + 1, 0.0);
    pm.drhoNodes.assign((size_t)N + 1, 0.0);
    for (int i = 0; i <= N; ++i)
        rhoOf(A, pm.d, (double)i * h, pm.rhoNodes[(size_t)i], pm.drhoNodes[(size_t)i]);
    // node values of F = integral of rho: 3-point Gauss-Legendre on each interval
    // (error ~ h^7), so the table is not the weak link
    const double g = std::sqrt(0.6);
    for (int i = 0; i < N; ++i) {
        const double mid = ((double)i + 0.5) * h;
        double f0, f1, f2, dd;
        rhoOf(A, pm.d, mid - 0.5 * h * g, f0, dd);
        rhoOf(A, pm.d, mid, f1, dd);
        rhoOf(A, pm.d, mid + 0.5 * h * g, f2, dd);
        pm.Fnodes[(size_t)i + 1] = pm.Fnodes[(size_t)i] +
                                   0.5 * h * (5.0 / 9.0 * f0 + 8.0 / 9.0 * f1 + 5.0 / 9.0 * f2);
    }
    pm.Ftot = pm.Fnodes[(size_t)N];
}

struct ConjReport {
    double maxDefect = 0.0, atPhiA = 0.0;
    double minRatio = 1e300, maxRatio = 0.0;
};

// Sweep the whole turn: the pair is conjugate iff, at every polar angle of A,
// the other profile meets it tangentially on the line of centers.
ConjReport conjugacySweep(const ProfileCurve& A, const ProfileCurve& B, double d) {
    ConjReport rep;
    const int N = 360;
    for (int k = 0; k < N; ++k) {
        const double phiA = 2.0 * kPi * (double)k / (double)N;
        double phiB, defect;
        conjugateRoot(A, phiA, B, d, phiB, defect);
        if (defect > rep.maxDefect) { rep.maxDefect = defect; rep.atPhiA = phiA; }
        double rA, rAp, rA2;
        A.polarEval(phiA, rA, rAp, rA2);
        if (d - rA > 0.0) {
            const double ratio = rA / (d - rA);
            rep.minRatio = std::min(rep.minRatio, ratio);
            rep.maxRatio = std::max(rep.maxRatio, ratio);
        }
    }
    return rep;
}

// ------------------------------------------------------------ M4 synthesis
//
// 'reduce' turns a speed-ratio intent into a concrete compound train:
// teeth counts, meshes, and (unless the author pinned them) shaft
// positions. Everything lands in the plain M3 kernel — no new solver rows.

// Tooth bounds for synthesized stages. Pinions 8..16 leaves and wheels
// 24..100 leaves bound every stage ratio to [1.5, 12.5]; the defaults are
// clockmaking-shaped (small pinions, modest wheels).
const int kSynthPinionMin = 8, kSynthPinionMax = 16;
const int kSynthWheelMin = 24, kSynthWheelMax = 100;
const int kSynthMaxStages = 5;

// Layout defaults: the first stage leaves the fast shaft at 'spread'
// (default 0 deg = +x); each later stage turns by 'turn' (default -60 deg,
// a gentle clockwise curl — like a train bending around a clock plate).
const double kSynthSpreadDeg = 0.0;
const double kSynthTurnDeg = -60.0;

long long gcdLL(long long a, long long b) {
    while (b != 0) { const long long t = a % b; a = b; b = t; }
    return a < 0 ? -a : a;
}

// One candidate stage: the ratio in lowest terms plus the smallest concrete
// tooth pair (wheel, pinion) that realizes it within the bounds.
struct StageCand {
    long long wn, wd;   // reduced ratio = wn/wd
    int w, p;           // concrete teeth
    double r;           // ratio value
};

std::vector<StageCand> buildStageCandidates() {
    std::map<std::pair<long long, long long>, StageCand> byRatio;
    for (int p = kSynthPinionMin; p <= kSynthPinionMax; ++p) {
        for (int w = kSynthWheelMin; w <= kSynthWheelMax; ++w) {
            if (w <= p) continue;
            const long long g = gcdLL(w, p);
            const auto key = std::make_pair(w / g, p / g);
            auto it = byRatio.find(key);
            if (it == byRatio.end() || w + p < it->second.w + it->second.p) {
                StageCand c{w / g, p / g, w, p, (double)w / (double)p};
                byRatio[key] = c;
            }
        }
    }
    std::vector<StageCand> out;
    for (const auto& kv : byRatio) out.push_back(kv.second);
    return out;
}

// Depth-first exact search: sequences of n candidate ratios whose product is
// exactly num/den. Scored by (max stage ratio, total teeth); deterministic
// candidate order keeps ties stable.
struct TrainSearch {
    const std::vector<StageCand>* cands = nullptr;
    long long nodes = 0;

    std::vector<StageCand> best;
    bool hasBest = false;
    double bestMaxR = 0.0;
    int bestTeeth = 0;

    void dfs(long long num, long long den, int k,
             std::vector<StageCand>& cur, double maxR, int teeth) {
        if (++nodes > 2000000) return;   // safety valve (pruned tree stays far below)
        if (k == 0) {
            if (num == 1 && den == 1 &&
                (!hasBest || maxR < bestMaxR - 1e-12 ||
                 (std::fabs(maxR - bestMaxR) <= 1e-12 && teeth < bestTeeth))) {
                best = cur;
                hasBest = true;
                bestMaxR = maxR;
                bestTeeth = teeth;
            }
            return;
        }
        // remaining stages each contribute a ratio in [24/16, 100/8]; the
        // product of k-1 more stages must be in [1.5^(k-1), 12.5^(k-1)] —
        // except when c is the last stage, where the remainder must be
        // exactly 1 (checked with exact integer arithmetic).
        for (const StageCand& c : *cands) {
            if (k == 1) {
                // last stage: c must consume the remaining ratio exactly
                if (num * c.wd != den * c.wn) continue;
            } else {
                const double rem = (double)num * (double)c.wd /
                                   ((double)den * (double)c.wn);
                const double lo = std::pow(24.0 / 16.0, k - 1);
                const double hi = std::pow(100.0 / 8.0, k - 1);
                if (rem < lo - 1e-9 || rem > hi + 1e-9) continue;
            }
            if (hasBest && std::max(maxR, c.r) > bestMaxR + 1e-12) continue;  // bound
            long long rn = num * c.wd, rd = den * c.wn;
            const long long g = gcdLL(rn, rd);
            rn /= g; rd /= g;
            cur.push_back(c);
            dfs(rn, rd, k - 1, cur, std::max(maxR, c.r), teeth + c.w + c.p);
            cur.pop_back();
        }
    }
};

bool searchExactStages(long long num, long long den, int n,
                       const std::vector<StageCand>& cands,
                       std::vector<StageCand>& out) {
    TrainSearch s;
    s.cands = &cands;
    std::vector<StageCand> cur;
    s.dfs(num, den, n, cur, 0.0, 0);
    if (s.hasBest) out = s.best;
    return s.hasBest;
}

// Solve |P - a| = rA, |P - b| = rB (circle intersection). Returns false if
// the circles do not meet. Both solutions are written; 'unique' is set when
// they coincide.
bool circleIntersect(Vec2 a, Vec2 b, double rA, double rB,
                     Vec2& s1, Vec2& s2, bool& unique) {
    const Vec2 d = b - a;
    const double D = vlen(d);
    if (D < 1e-12) return false;
    if (D > rA + rB + 1e-9 || D < std::fabs(rA - rB) - 1e-9) return false;
    const double x = (rA * rA - rB * rB + D * D) / (2.0 * D);
    const double h2 = rA * rA - x * x;
    const double h = h2 > 0.0 ? std::sqrt(h2) : 0.0;
    const Vec2 u = (1.0 / D) * d;
    const Vec2 base = a + x * u;
    s1 = base + h * perp(u);
    s2 = base - h * perp(u);
    unique = h < 1e-9;
    return true;
}

} // namespace

Model buildModel(const MechanismDecl& decl) {
    Model m;
    m.name = decl.name;

    // --- world parameters & dynamics mode ------------------------------------
    m.world.gravity = decl.world.present ? decl.world.gravity : 9.81;
    m.world.step = decl.world.present ? decl.world.step : 0.002;
    if (m.world.step <= 0.0 || m.world.step > 0.1) {
        throw SemanticError{decl.world.line, "world 'step' must be in (0, 0.1] seconds"};
    }
    if (m.world.gravity < 0.0) {
        throw SemanticError{decl.world.line, "world 'gravity' cannot be negative"};
    }
    m.world.dynamic = decl.world.present;
    if (!decl.springs.empty()) m.world.dynamic = true;
    for (const DriveDecl& d : decl.drives)
        if (d.isTorque || d.isWeight) m.world.dynamic = true;
    if (!decl.modes.empty()) m.world.dynamic = true;   // M5: hybrid = dynamic

    // --- exactly one ground body, parked at index 0 ------------------------
    const BodyDecl* groundDecl = nullptr;
    for (const BodyDecl& b : decl.bodies) {
        if (b.kind == BodyDecl::Kind::Ground) {
            if (groundDecl) {
                throw SemanticError{b.line, "mechanism '" + decl.name +
                    "' has more than one ground body ('" + groundDecl->name +
                    "' and '" + b.name + "')"};
            }
            groundDecl = &b;
        }
    }
    if (!groundDecl) {
        throw SemanticError{decl.line, "mechanism '" + decl.name + "' has no ground body"};
    }

    // --- bodies in declaration order (ground first) -------------------------
    std::vector<JointDecl> attachJoints;   // desugared rod/shaft attach pins
    std::unordered_set<std::string> seen;

    {
        Body g;
        g.name = groundDecl->name;
        g.isGround = true;
        g.qOffset = -1;
        g.declLine = groundDecl->line;
        for (const AnchorDecl& a : groundDecl->anchors) {
            Anchor r;
            r.name = a.name;
            r.local = a.pos;
            if (a.hasDir) {
                r.hasDir = true;
                r.dirLocal = normalizeDir(a.dir, a.line, a.name);
            }
            g.anchors.push_back(r);
        }
        m.bodies.push_back(g);
        seen.insert(g.name);
    }

    for (const BodyDecl& bd : decl.bodies) {
        if (bd.kind == BodyDecl::Kind::Ground) continue;

        if (!seen.insert(bd.name).second) {
            throw SemanticError{bd.line, "duplicate body name '" + bd.name + "'"};
        }
        Body b;
        b.name = bd.name;
        b.isGround = false;
        b.declLine = bd.line;
        b.qOffset = m.nq();  // this body's (theta,x,y) slot, assigned pre-push

        if (bd.kind == BodyDecl::Kind::Rod) {
            const bool hasBeat = bd.props.count("beat") > 0;
            if (hasBeat && bd.props.count("length")) {
                throw SemanticError{bd.line, "rod '" + bd.name +
                    "' specifies both 'length' and 'beat' — choose one"};
            }
            if (hasBeat) {
                // M4 period intent. Horology: a 'beat' is half a swing (tick,
                // tock), so T = 2*beat. For the uniform compound rod the
                // linearized period is T = 2*pi*sqrt(2L/(3g)), hence
                //   L = 3*g*beat^2 / (2*pi^2)
                if (!m.world.dynamic) {
                    throw SemanticError{bd.line, "rod '" + bd.name +
                        "' uses 'beat' but the mechanism is kinematic — add a "
                        "world block (gravity)"};
                }
                const double beat = bd.props.at("beat");
                if (beat <= 0.0) {
                    throw SemanticError{bd.line, "rod '" + bd.name +
                        "' needs a positive 'beat'"};
                }
                b.length = 3.0 * m.world.gravity * beat * beat / (2.0 * kPi * kPi);
                b.beatSeconds = beat;
            } else {
                auto it = bd.props.find("length");
                if (it == bd.props.end() || it->second <= 0.0) {
                    throw SemanticError{bd.line, "rod '" + bd.name +
                        "' needs a positive 'length' property (or 'beat')"};
                }
                b.length = it->second;
            }
            // Desugar: a rod of length L exports anchors A (origin end) and B (tip).
            Anchor A; A.name = "A"; A.local = Vec2{0.0, 0.0};
            Anchor B; B.name = "B"; B.local = Vec2{b.length, 0.0};
            b.anchors.push_back(A);
            b.anchors.push_back(B);
        } else if (bd.kind == BodyDecl::Kind::Shaft) {
            // Desugar: a shaft is a zero-extent rotational body (anchor A only).
            b.isShaft = true;
            Anchor A; A.name = "A"; A.local = Vec2{0.0, 0.0};
            b.anchors.push_back(A);
        } else { // Free
            for (const AnchorDecl& a : bd.anchors) {
                Anchor r;
                r.name = a.name;
                r.local = a.pos;
                if (a.hasDir) {
                    r.hasDir = true;
                    r.dirLocal = normalizeDir(a.dir, a.line, a.name);
                }
                b.anchors.push_back(r);
            }
        }

        // Release-angle sugar: `from: 60 deg` seeds assembly at that angle.
        if (bd.props.count("from")) {
            b.hasSeed = true;
            b.seedAngle = deg2rad(bd.props.at("from"));
        }

        // Desugar attach sugar into a pin on anchor A.
        if (bd.hasAttach) {
            if (bd.kind == BodyDecl::Kind::Free) {
                throw SemanticError{bd.line, "body '" + bd.name +
                    "' cannot use attach sugar — pin its anchors explicitly"};
            }
            if (bd.attachBody == bd.name) {
                throw SemanticError{bd.line, "'" + bd.name +
                    "' cannot be attached to itself"};
            }
            JointDecl j;
            j.kind = JointDecl::Kind::Pin;
            j.bodyA = bd.name;
            j.anchorA = "A";
            j.bodyB = bd.attachBody;
            j.anchorB = bd.attachAnchor;
            j.line = bd.line;
            attachJoints.push_back(j);
        }
        m.bodies.push_back(b);
    }

    // --- joints: pins (with attach sugar first) and slides ------------------
    std::vector<JointDecl> allJoints = attachJoints;
    allJoints.insert(allJoints.end(), decl.joints.begin(), decl.joints.end());

    for (const JointDecl& jd : allJoints) {
        const int biA = m.bodyIndex(jd.bodyA);
        if (biA < 0) {
            throw SemanticError{jd.line, "joint refers to unknown body '" + jd.bodyA + "'"};
        }
        const int biB = m.bodyIndex(jd.bodyB);
        if (biB < 0) {
            throw SemanticError{jd.line, "joint refers to unknown body '" + jd.bodyB + "'"};
        }
        const int anA = m.anchorIndex(biA, jd.anchorA);
        if (anA < 0) {
            throw SemanticError{jd.line, "body '" + jd.bodyA +
                "' has no anchor '" + jd.anchorA + "'"};
        }
        const int anB = m.anchorIndex(biB, jd.anchorB);
        if (anB < 0) {
            throw SemanticError{jd.line, "body '" + jd.bodyB +
                "' has no anchor '" + jd.anchorB + "'"};
        }

        if (jd.kind == JointDecl::Kind::Pin) {
            if (biA == biB) {
                throw SemanticError{jd.line, "cannot pin body '" + jd.bodyA + "' to itself"};
            }
            m.pins.push_back(PinJoint{biA, anA, biB, anB, jd.line});
        } else { // Slide
            // M2: the rail must be on ground; the guide must be a moving body.
            const bool ag = m.bodies[biA].isGround, bg = m.bodies[biB].isGround;
            if (ag == bg) {
                throw SemanticError{jd.line, "slide '" + jd.bodyA + "." + jd.anchorA +
                    " -> " + jd.bodyB + "." + jd.anchorB +
                    "' must join exactly one moving body (the guide) and one ground "
                    "anchor (the rail)"};
            }
            const int guideBody = ag ? biB : biA;
            const int guideAnch = ag ? anB : anA;
            const int railBody  = ag ? biA : biB;
            const int railAnch  = ag ? anA : anB;
            const Anchor& guide = m.bodies[guideBody].anchors[guideAnch];
            const Anchor& rail  = m.bodies[railBody].anchors[railAnch];
            if (!rail.hasDir) {
                throw SemanticError{jd.line, "rail anchor '" + jd.bodyB + "." + jd.anchorB +
                    "' needs a direction — e.g. anchor rail at (0, 0) dir (1, 0)"};
            }
            if (!guide.hasDir) {
                throw SemanticError{jd.line, "guide anchor '" + jd.bodyA + "." + jd.anchorA +
                    "' needs a direction — e.g. anchor guide at (0, 0) dir (1, 0)"};
            }
            SlideJoint s;
            s.guideBody = guideBody;
            s.guideAnchor = guideAnch;
            s.railPos = m.bodies[railBody].anchors[railAnch].local;   // ground: world
            s.railDir = rail.dirLocal;                                 // ground: world
            s.line = jd.line;
            m.slides.push_back(s);
        }
    }

    // --- gears ---------------------------------------------------------------
    std::unordered_set<std::string> gearNames;
    for (const GearDecl& gd : decl.gears) {
        if (!gearNames.insert(gd.name).second) {
            throw SemanticError{gd.line, "duplicate gear name '" + gd.name + "'"};
        }
        Gear g;
        g.name = gd.name;
        g.line = gd.line;
        g.body = m.bodyIndex(gd.host);
        if (g.body < 0) {
            // friendlier message when the name matches a gear
            throw SemanticError{gd.line, "gear '" + gd.name +
                "' refers to unknown host body '" + gd.host + "'"};
        }
        // M3: the pitch circle may sit on any host anchor; ground hosts are
        // allowed (fixed rings, stationary gears).
        const std::string anchName = gd.hostAnchor.empty() ? "A" : gd.hostAnchor;
        g.hostAnchor = m.anchorIndex(g.body, anchName);
        if (g.hostAnchor < 0) {
            if (gd.hostAnchor.empty() && m.bodies[g.body].isGround) {
                throw SemanticError{gd.line, "gear '" + gd.name + "' on ground '" +
                    gd.host + "' needs an explicit anchor — e.g. 'on " + gd.host +
                    ".p' (ground bodies have no default anchor)"};
            }
            throw SemanticError{gd.line, "gear '" + gd.name +
                "': host body '" + gd.host + "' has no anchor '" + anchName + "'"};
        }

        const double teeth = gd.props.count("teeth") ? gd.props.at("teeth") : 0.0;
        const double radius = gd.props.count("radius") ? gd.props.at("radius") : 0.0;
        const double module = gd.props.count("module") ? gd.props.at("module") : 0.01;
        if (gd.props.count("mass")) {
            g.massOverride = gd.props.at("mass");
            if (g.massOverride < 0.0) {
                throw SemanticError{gd.line, "gear '" + gd.name +
                    "' needs a non-negative 'mass'"};
            }
        }

        if (teeth > 0.0 && radius > 0.0) {
            throw SemanticError{gd.line, "gear '" + gd.name +
                "' specifies both 'teeth' and 'radius' — choose one"};
        }
        if (teeth > 0.0) {
            if (module <= 0.0) {
                throw SemanticError{gd.line, "gear '" + gd.name +
                    "' needs a positive 'module'"};
            }
            g.teeth = (int)teeth;
            g.module = module;
            g.radius = 0.5 * module * teeth;
        } else if (radius > 0.0) {
            g.radius = radius;
        } else {
            throw SemanticError{gd.line, "gear '" + gd.name +
                "' needs 'teeth' (with optional 'module', default 0.01) or 'radius'"};
        }
        m.gears.push_back(g);
    }

    // --- M6 profiles: closed contact curves on a host anchor --------------------
    std::unordered_set<std::string> profileNames;
    // host body + anchor of a contact curve (profile or cam); a ground host
    // needs an explicit anchor, as for gears
    auto resolveHost = [&](const char* kind, const std::string& name, const std::string& host,
                           const std::string& hostAnchor, int line, int& body, int& anchor) {
        body = m.bodyIndex(host);
        if (body < 0) {
            throw SemanticError{line, std::string(kind) + " '" + name +
                "' refers to unknown host body '" + host + "'"};
        }
        const std::string anchName = hostAnchor.empty() ? "A" : hostAnchor;
        anchor = m.anchorIndex(body, anchName);
        if (anchor < 0) {
            if (hostAnchor.empty() && m.bodies[(size_t)body].isGround) {
                throw SemanticError{line, std::string(kind) + " '" + name + "' on ground '" +
                    host + "' needs an explicit anchor — e.g. 'on " + host +
                    ".p' (ground bodies have no default anchor)"};
            }
            throw SemanticError{line, std::string(kind) + " '" + name +
                "': host body '" + host + "' has no anchor '" + anchName + "'"};
        }
    };
    for (const ProfileDecl& pd : decl.profiles) {
        if (!profileNames.insert(pd.name).second) {
            throw SemanticError{pd.line, "duplicate profile name '" + pd.name + "'"};
        }
        Profile pr;
        pr.name = pd.name;
        pr.line = pd.line;
        resolveHost("profile", pd.name, pd.host, pd.hostAnchor, pd.line,
                    pr.body, pr.hostAnchor);

        const size_t kMaxKnots = 1024;
        if (pd.kind == ProfileDecl::Kind::Circle) {
            if (!(pd.radius > 0.0)) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': circle needs a positive radius"};
            }
            pr.curve = std::make_shared<const ProfileCurve>(ProfileCurve::circle(pd.radius));
        } else if (pd.kind == ProfileDecl::Kind::Polygon) {
            const std::vector<Vec2>& pts = pd.points;
            if (pts.size() < 3) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': polygon needs at least 3 vertices"};
            }
            if (pts.size() > kMaxKnots) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': polygon has too many vertices (max 1024)"};
            }
            if (!pd.hasFillet || !(pd.fillet > 0.0)) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': polygon needs a positive 'fillet' radius — a perfectly sharp corner "
                    "has no contact normal (a small one, like 0.0002, is fine)"};
            }
            double area2 = 0.0, perim = 0.0;
            for (size_t i = 0; i < pts.size(); ++i) {
                const Vec2& a = pts[i];
                const Vec2& b = pts[(i + 1) % pts.size()];
                const double edge = vlen(b - a);
                if (edge < 1e-9) {
                    throw SemanticError{pd.line, "profile '" + pd.name +
                        "': polygon has two coincident neighbouring vertices"};
                }
                area2 += crossz(a, b);
                perim += edge;
            }
            if (std::fabs(area2) < 1e-9 * perim * perim) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': polygon encloses no area (collinear?)"};
            }
            const ProfileCurve pg = ProfileCurve::polygon(pts, pd.fillet);
            if (pg.minFillet() < 0.05 * pd.fillet) {
                char buf[300];
                std::snprintf(buf, sizeof buf,
                    "profile '%s': the %g m fillet does not fit — at some corner the edges are "
                    "too short, and it would shrink to %g m; shorten 'fillet' or lengthen the "
                    "edges", pd.name.c_str(), pd.fillet, pg.minFillet());
                throw SemanticError{pd.line, buf};
            }
            pr.curve = std::make_shared<const ProfileCurve>(pg);
        } else if (pd.kind == ProfileDecl::Kind::Ellipse) {
            if (!(pd.ellipseA > 0.0) || !(pd.ellipseB > 0.0)) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': ellipse needs two positive semi-axes"};
            }
            if (pd.ellipseFocus && pd.ellipseA < pd.ellipseB) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': an ellipse pivoted at its focus needs the major semi-axis first "
                    "(a >= b) — the focus lies on the x axis"};
            }
            pr.curve = std::make_shared<const ProfileCurve>(
                ProfileCurve::ellipse(pd.ellipseA, pd.ellipseB, pd.ellipseFocus));
        } else if (pd.kind == ProfileDecl::Kind::Polar) {
            if (pd.polar.size() < 3) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': polar needs at least 3 (angle, radius) pairs"};
            }
            if (pd.polar.size() > kMaxKnots) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': polar has too many pairs (max 1024)"};
            }
            std::vector<double> ang, rad;
            for (size_t i = 0; i < pd.polar.size(); ++i) {
                const double a = pd.polar[i].first, r = pd.polar[i].second;
                if (a < 0.0 || a >= 2.0 * kPi) {
                    throw SemanticError{pd.line, "profile '" + pd.name +
                        "': polar angles must lie in [0, 360) degrees"};
                }
                if (i > 0 && !(a > ang.back())) {
                    throw SemanticError{pd.line, "profile '" + pd.name +
                        "': polar angles must be strictly increasing"};
                }
                if (!(r > 0.0)) {
                    throw SemanticError{pd.line, "profile '" + pd.name +
                        "': polar radii must be positive"};
                }
                ang.push_back(a);
                rad.push_back(r);
            }
            pr.curve = std::make_shared<const ProfileCurve>(ProfileCurve::polar(ang, rad));
        } else {
            const std::vector<Vec2>& pts = pd.points;
            if (pts.size() < 3) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': points needs at least 3 points"};
            }
            if (pts.size() > kMaxKnots) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': points has too many points (max 1024)"};
            }
            double area2 = 0.0, perim = 0.0;
            for (size_t i = 0; i < pts.size(); ++i) {
                const Vec2& a = pts[i];
                const Vec2& b = pts[(i + 1) % pts.size()];
                const double edge = vlen(b - a);
                if (edge < 1e-9) {
                    throw SemanticError{pd.line, "profile '" + pd.name +
                        "': points has two coincident neighbours"};
                }
                area2 += crossz(a, b);
                perim += edge;
            }
            if (std::fabs(area2) < 1e-9 * perim * perim) {
                throw SemanticError{pd.line, "profile '" + pd.name +
                    "': points enclose no area (collinear?)"};
            }
            pr.curve = std::make_shared<const ProfileCurve>(ProfileCurve::points(pts));
        }
        m.profiles.push_back(pr);
    }

    // --- M6b cams: stated as motion, cut as a profile -----------------------------
    for (const CamDecl& cd : decl.cams) {
        if (!profileNames.insert(cd.name).second) {
            throw SemanticError{cd.line, "duplicate profile name '" + cd.name +
                "' (cams and profiles share one namespace)"};
        }
        Profile pr;
        pr.name = cd.name;
        pr.line = cd.line;
        resolveHost("cam", cd.name, cd.host, cd.hostAnchor, cd.line, pr.body, pr.hostAnchor);
        if (!cd.hasBase) {
            throw SemanticError{cd.line, "cam '" + cd.name +
                "' needs a 'base' radius — e.g. 'base 0.03'"};
        }
        if (!(cd.spec.base > 0.0)) {
            throw SemanticError{cd.line, "cam '" + cd.name + "': 'base' must be positive"};
        }
        if (cd.spec.roller < 0.0) {
            throw SemanticError{cd.line, "cam '" + cd.name + "': 'roller' cannot be negative"};
        }
        CamDiagram dia;
        CamError ce;
        if (!dia.build(cd.spec, ce)) {
            const int line = ce.segment >= 0 ? cd.segLines[(size_t)ce.segment] : cd.line;
            throw SemanticError{line, "cam '" + cd.name + "': " + ce.msg};
        }
        const CamDesign des = designCam(cd.spec, dia);
        if (des.undercut) {
            char buf[400];
            std::snprintf(buf, sizeof buf,
                "cam '%s': undercut — the pitch curve's radius of curvature falls to %.4g m "
                "at %.1f deg of cam rotation, below the %.4g m roller, so the cam surface "
                "would fold over itself; enlarge 'base', reduce the lift, lengthen the "
                "segment, or use a smaller 'roller'",
                cd.name.c_str(), des.minConvexRadius, rad2deg(des.minConvexAt), cd.spec.roller);
            throw SemanticError{cd.line, buf};
        }
        pr.curve = std::make_shared<const ProfileCurve>(ProfileCurve::points(des.surface));
        pr.fromCam = true;
        pr.designRoller = cd.spec.roller;

        CamInfo ci;
        ci.name = cd.name;
        ci.profile = (int)m.profiles.size();
        ci.spec = cd.spec;
        ci.maxLift = des.maxLift;
        ci.maxPressure = des.maxPressure;
        ci.maxPressureAt = des.maxPressureAt;
        ci.minConvexRadius = des.minConvexRadius;
        ci.minConvexAt = des.minConvexAt;
        ci.line = cd.line;
        m.cams.push_back(ci);
        m.profiles.push_back(pr);
    }

    // --- meshes ---------------------------------------------------------------
    for (const MeshDecl& md : decl.meshes) {
        // M6d: two profiles instead of two gears -> a non-circular gear pair
        const int pa = m.profileIndex(md.gearA), pb = m.profileIndex(md.gearB);
        const bool bothGears = m.gearIndex(md.gearA) >= 0 && m.gearIndex(md.gearB) >= 0;
        if (!bothGears && (pa >= 0 || pb >= 0)) {
            if (pa < 0 || pb < 0) {
                const std::string& bad = pa < 0 ? md.gearA : md.gearB;
                throw SemanticError{md.line, "mesh joins a profile and '" + bad +
                    "', which is not a profile — mesh two gears or two profiles"};
            }
            if (pa == pb) {
                throw SemanticError{md.line, "profile '" + md.gearA + "' cannot mesh with itself"};
            }
            if (md.tangent) {
                throw SemanticError{md.line, "'tangent' applies to gear meshes; a profile mesh "
                    "already keeps the contact on the line of centers"};
            }
            const Profile& PA = m.profiles[(size_t)pa];
            const Profile& PB = m.profiles[(size_t)pb];
            for (const Profile* P : {&PA, &PB}) {
                if (!P->curve->starShaped()) {
                    throw SemanticError{md.line, "profile '" + P->name + "' is a point list; a "
                        "mesh needs star-shaped profiles (circle, polar or ellipse) so each "
                        "ray from the shaft meets the curve once"};
                }
                if (m.bodies[(size_t)P->body].isGround) {
                    throw SemanticError{md.line, "profile '" + P->name + "' is on ground; a "
                        "mesh needs both profiles on rotating shafts"};
                }
            }
            if (PA.body == PB.body) {
                throw SemanticError{md.line, "profiles '" + PA.name + "' and '" + PB.name +
                    "' sit on the same body — coaxial profiles cannot mesh"};
            }
            // fixed centers: each host must be pinned to ground at the profile's anchor
            auto groundCenter = [&](const Profile& P, Vec2& out) {
                for (const PinJoint& pj : m.pins) {
                    if (pj.bodyA == P.body && pj.anchA == P.hostAnchor &&
                        m.bodies[(size_t)pj.bodyB].isGround) {
                        out = m.bodies[(size_t)pj.bodyB].anchors[(size_t)pj.anchB].local;
                        return true;
                    }
                    if (pj.bodyB == P.body && pj.anchB == P.hostAnchor &&
                        m.bodies[(size_t)pj.bodyA].isGround) {
                        out = m.bodies[(size_t)pj.bodyA].anchors[(size_t)pj.anchA].local;
                        return true;
                    }
                }
                return false;
            };
            PolarMesh pm;
            pm.line = md.line;
            pm.profA = pa;
            pm.profB = pb;
            for (int side = 0; side < 2; ++side) {
                const Profile& P = side == 0 ? PA : PB;
                if (!groundCenter(P, side == 0 ? pm.cA : pm.cB)) {
                    throw SemanticError{md.line, "profile '" + P.name + "' is meshed but its "
                        "shaft is not pinned to ground at the profile's anchor; a non-circular "
                        "mesh needs fixed centers (pin the host's anchor to a ground anchor)"};
                }
            }
            pm.d = vlen(pm.cB - pm.cA);
            if (!(pm.d > 1e-9)) {
                throw SemanticError{md.line, "profiles '" + PA.name + "' and '" + PB.name +
                    "' have coincident centers"};
            }
            pm.psi = std::atan2(pm.cB.y - pm.cA.y, pm.cB.x - pm.cA.x);

            // a row that merely *describes* rolling is only rolling for a conjugate pair
            const ConjReport cr = conjugacySweep(*PA.curve, *PB.curve, pm.d);
            if (cr.maxDefect > 1e-3) {
                char buf[420];
                if (cr.maxDefect > 1e8) {
                    std::snprintf(buf, sizeof buf,
                        "profiles '%s' and '%s' cannot roll on each other at center distance "
                        "%.4f m: at a polar angle of %.1f deg on '%s' the other profile never "
                        "reaches the line of centers at the needed radius — check the center "
                        "distance against the two radii",
                        PA.name.c_str(), PB.name.c_str(), pm.d, rad2deg(cr.atPhiA),
                        PA.name.c_str());
                } else {
                    std::snprintf(buf, sizeof buf,
                        "profiles '%s' and '%s' are not a conjugate pair at center distance "
                        "%.4f m: they meet on the line of centers but not tangentially (the "
                        "log-slope of r differs by %.3g at %.1f deg), so they would cross, not "
                        "roll — an elliptical pair needs identical ellipses pivoted at their "
                        "foci, a distance 2a apart",
                        PA.name.c_str(), PB.name.c_str(), pm.d, cr.maxDefect, rad2deg(cr.atPhiA));
                }
                throw SemanticError{md.line, buf};
            }
            pm.conjDefect = cr.maxDefect;
            pm.minRatio = cr.minRatio;
            pm.maxRatio = cr.maxRatio;
            buildPhaseTable(pm, *PA.curve);
            m.polarMeshes.push_back(pm);
            continue;
        }

        MeshJoint mm;
        mm.line = md.line;
        mm.tangent = md.tangent;
        mm.gearA = m.gearIndex(md.gearA);
        if (mm.gearA < 0) {
            throw SemanticError{md.line, "mesh refers to unknown gear '" + md.gearA + "'"};
        }
        mm.gearB = m.gearIndex(md.gearB);
        if (mm.gearB < 0) {
            throw SemanticError{md.line, "mesh refers to unknown gear '" + md.gearB + "'"};
        }
        if (mm.gearA == mm.gearB) {
            throw SemanticError{md.line, "gear '" + md.gearA + "' cannot mesh with itself"};
        }
        const Gear& ga = m.gears[mm.gearA];
        const Gear& gb = m.gears[mm.gearB];
        if (m.bodies[ga.body].isGround && m.bodies[gb.body].isGround) {
            throw SemanticError{md.line, "mesh '" + ga.name + "' <-> '" + gb.name +
                "' is between two gears on ground — nothing can roll"};
        }
        if (ga.body == gb.body) {
            throw SemanticError{md.line, "gears '" + ga.name + "' and '" + gb.name +
                "' sit on the same body — coaxial gears cannot mesh"};
        }
        if (ga.teeth > 0 && gb.teeth > 0 && ga.module != gb.module) {
            char buf[192];
            std::snprintf(buf, sizeof buf,
                "gears '%s' (module %g) and '%s' (module %g) cannot mesh — "
                "meshing gears must share a module",
                ga.name.c_str(), ga.module, gb.name.c_str(), gb.module);
            throw SemanticError{md.line, buf};
        }
        m.meshes.push_back(mm);
    }

    // --- M6 touches: a point rides a profile -------------------------------------
    for (const TouchDecl& td : decl.touches) {
        TouchJoint tj;
        tj.line = td.line;
        tj.body = m.bodyIndex(td.body);
        if (tj.body < 0) {
            throw SemanticError{td.line, "touch refers to unknown body '" + td.body + "'"};
        }
        tj.anchor = m.anchorIndex(tj.body, td.anchor);
        if (tj.anchor < 0) {
            throw SemanticError{td.line, "body '" + td.body +
                "' has no anchor '" + td.anchor + "'"};
        }
        tj.profile = m.profileIndex(td.profile);
        if (tj.profile < 0) {
            throw SemanticError{td.line, "touch refers to unknown profile '" +
                td.profile + "'"};
        }
        const Profile& pr = m.profiles[(size_t)tj.profile];
        if (tj.body == pr.body) {
            throw SemanticError{td.line, "body '" + td.body +
                "' cannot touch its own profile '" + pr.name + "'"};
        }
        if (td.hasRoller) {
            if (!(td.roller > 0.0)) {
                throw SemanticError{td.line, "touch roller needs a positive radius "
                    "(omit 'roller' for a knife-edge tip)"};
            }
            tj.roller = td.roller;
        } else {
            tj.roller = pr.designRoller;   // a cam is cut for its roller: follow it
        }
        if (td.hasRestitution) {
            if (!m.world.dynamic) {
                throw SemanticError{td.line, "touch 'restitution' has no effect in a "
                    "kinematic mechanism (the contact is bilateral and nothing lands) — "
                    "add a world block, or drop it"};
            }
            if (!(td.restitution >= 0.0 && td.restitution <= 1.0)) {
                throw SemanticError{td.line, "touch 'restitution' must lie in [0, 1]"};
            }
            tj.restitution = td.restitution;
        }
        if (td.hasFriction) {
            if (!m.world.dynamic) {
                throw SemanticError{td.line, "touch 'friction' has no effect in a kinematic "
                    "mechanism (nothing slides under a force there) — add a world block, or "
                    "drop it"};
            }
            if (!(td.friction >= 0.0)) {
                throw SemanticError{td.line, "touch 'friction' (the Coulomb coefficient) "
                    "cannot be negative"};
            }
            tj.friction = td.friction;
        }
        m.touches.push_back(tj);
    }

    // --- M4 intent layer: 'reduce' synthesizes whole gear trains ----------
    //
    // Each reduce desugars into kernel objects only: intermediate shafts
    // (if not declared), gears (wheel on the driving shaft, pinion on the
    // driven one, compound pairs in between), meshes, and pins to freshly
    // created ground anchors for every shaft the author did not pin. The
    // author pins what they care about; the solver lays out the rest.
    {
        std::map<int, Vec2> synthPos;   // bodies placed by earlier reduces
        const std::vector<StageCand> cands = buildStageCandidates();

        // Position of a body if it is statically known at build time: a pin
        // to a ground anchor, or placement by an earlier reduce.
        auto staticPos = [&](int bi, Vec2& out) -> bool {
            if (bi <= 0) return false;
            for (const PinJoint& p : m.pins) {
                if (p.bodyA == bi && m.bodies[p.bodyB].isGround) {
                    out = m.bodies[p.bodyB].anchors[p.anchB].local;
                    return true;
                }
                if (p.bodyB == bi && m.bodies[p.bodyA].isGround) {
                    out = m.bodies[p.bodyA].anchors[p.anchA].local;
                    return true;
                }
            }
            auto it = synthPos.find(bi);
            if (it != synthPos.end()) { out = it->second; return true; }
            return false;
        };

        auto uniqueGearName = [&](std::string base) -> std::string {
            while (m.gearIndex(base) >= 0) base += "_";
            return base;
        };
        auto uniqueBodyName = [&](std::string base) -> std::string {
            while (m.bodyIndex(base) >= 0) base += "_";
            return base;
        };
        // ground anchor for an auto-placed shaft, named after the shaft
        auto groundAnchorFor = [&](const std::string& shaftName, Vec2 pos) -> int {
            std::string name = shaftName;
            while (m.anchorIndex(0, name) >= 0) name += "_";
            Anchor a;
            a.name = name;
            a.local = pos;
            m.bodies[0].anchors.push_back(a);
            return (int)m.bodies[0].anchors.size() - 1;
        };
        auto addSynthGear = [&](const std::string& name, int bodyIdx,
                                 int teeth, double module) -> int {
            Gear g;
            g.name = uniqueGearName(name);
            g.body = bodyIdx;
            g.hostAnchor = 0;              // shaft anchor A / body origin
            g.teeth = teeth;
            g.module = module;
            g.radius = 0.5 * module * teeth;
            g.line = 0;
            m.gears.push_back(g);
            return (int)m.gears.size() - 1;
        };
        auto addSynthMesh = [&](int wheelIdx, int pinionIdx, int line) {
            MeshJoint mm;
            mm.gearA = wheelIdx;
            mm.gearB = pinionIdx;
            mm.tangent = false;
            mm.line = line;
            m.meshes.push_back(mm);
        };

        for (const ReduceDecl& rd : decl.reduces) {
            // -- resolve and sanity-check the chain -------------------------
            const int biA = m.bodyIndex(rd.bodyA);
            if (biA < 0) {
                throw SemanticError{rd.line, "reduce refers to unknown body '" +
                    rd.bodyA + "'"};
            }
            const int biB = m.bodyIndex(rd.bodyB);
            if (biB < 0) {
                throw SemanticError{rd.line, "reduce refers to unknown body '" +
                    rd.bodyB + "'"};
            }
            if (biA == biB) {
                throw SemanticError{rd.line, "reduce cannot go from '" + rd.bodyA +
                    "' to itself"};
            }
            if (m.bodies[biA].isGround || m.bodies[biB].isGround) {
                throw SemanticError{rd.line, "reduce endpoints must be moving bodies"};
            }

            // normalize the ratio: the larger side is the fast side, so the
            // reduction factor R = max/min as an exact reduced fraction.
            long long Rn = rd.ratioNum, Rd = rd.ratioDen;
            if (Rn < Rd) { std::swap(Rn, Rd); }
            const long long g0 = gcdLL(Rn, Rd);
            Rn /= g0; Rd /= g0;
            const double R = (double)Rn / (double)Rd;
            if (R < 1.5 - 1e-12) {
                char buf[192];
                std::snprintf(buf, sizeof buf,
                    "reduce ratio %lld:%lld is below the smallest synthesizable "
                    "stage (24:16 = 1.5)", rd.ratioNum, rd.ratioDen);
                throw SemanticError{rd.line, buf};
            }
            if (R > std::pow(100.0 / 8.0, kSynthMaxStages)) {
                char buf[224];
                std::snprintf(buf, sizeof buf,
                    "reduce ratio %lld:%lld exceeds 12.5^5 = 305175 — the most "
                    "5 stages of at most 100:8 can do", rd.ratioNum, rd.ratioDen);
                throw SemanticError{rd.line, buf};
            }

            // -- choose the stage count ------------------------------------
            // Stage count preference: fewest stages; parity (same/opposite)
            // is met by stage count when possible, otherwise one idler
            // shaft is inserted (an idler flips sense without changing ratio).
            const bool wantSame = rd.dir == ReduceDecl::Dir::Same;
            const bool wantOpp = rd.dir == ReduceDecl::Dir::Opposite;
            const int viaCount = (int)rd.via.size();
            int n = -1;                      // chosen stage count
            bool idler = false;              // insert an idler shaft?
            std::vector<StageCand> stages;

            if (viaCount > 0) {
                // 'via' fixes the topology: exactly viaCount + 1 stages
                n = viaCount + 1;
                if (n > kSynthMaxStages) {
                    throw SemanticError{rd.line, "reduce via lists " +
                        std::to_string(viaCount) + " shafts (=" +
                        std::to_string(n) + " stages) — the maximum is " +
                        std::to_string(kSynthMaxStages)};
                }
                if (!searchExactStages(Rn, Rd, n, cands, stages)) {
                    char buf[288];
                    std::snprintf(buf, sizeof buf,
                        "ratio %lld:%lld cannot be built from exactly %d stages "
                        "within tooth bounds (pinions 8..16, wheels 24..100)",
                        rd.ratioNum, rd.ratioDen, n);
                    throw SemanticError{rd.line, buf};
                }
                // meshes = stages (+1 with an idler); (meshes odd) -> opposite
                if (wantSame && (n % 2) != 0) idler = true;
                if (wantOpp && (n % 2) == 0) idler = true;
            } else {
                // try stage counts with the requested parity first
                for (int parityPass = 0; parityPass < 2 && n < 0; ++parityPass) {
                    for (int k = 1; k <= kSynthMaxStages; ++k) {
                        const bool odd = (k % 2) == 1;
                        if (parityPass == 0) {
                            if (wantSame && odd) continue;    // same wants even
                            if (wantOpp && !odd) continue;    // opposite wants odd
                        }
                        std::vector<StageCand> trial;
                        if (searchExactStages(Rn, Rd, k, cands, trial)) {
                            n = k;
                            stages = trial;
                            break;
                        }
                    }
                    // pass 1: any parity works — an idler will fix the sense
                    if (parityPass == 0 && n < 0 && (wantSame || wantOpp)) {
                        if (wantSame) idler = true;
                        if (wantOpp) idler = true;
                    }
                }
                if (n < 0) {
                    char buf[288];
                    std::snprintf(buf, sizeof buf,
                        "ratio %lld:%lld cannot be synthesized: no product of "
                        "1..5 stage ratios (each within 24:16..100:8) equals it — "
                        "try pinning explicit gears instead",
                        rd.ratioNum, rd.ratioDen);
                    throw SemanticError{rd.line, buf};
                }
            }

            // -- build the chain of bodies ---------------------------------
            // chain[0] = A, chain[n] = B; intermediates from 'via' or synthesized.
            std::vector<int> chain;
            chain.push_back(biA);
            for (const std::string& v : rd.via) {
                const int bi = m.bodyIndex(v);
                if (bi < 0) {
                    throw SemanticError{rd.line, "reduce 'via' refers to unknown "
                        "body '" + v + "'"};
                }
                if (m.bodies[bi].isGround) {
                    throw SemanticError{rd.line, "reduce 'via' body '" + v +
                        "' is ground — trains run between moving bodies"};
                }
                if (bi == biA || bi == biB) {
                    throw SemanticError{rd.line, "reduce 'via' body '" + v +
                        "' is an endpoint of the same reduce"};
                }
                for (size_t c = 0; c < chain.size(); ++c) {
                    if (chain[c] == bi) {
                        throw SemanticError{rd.line, "reduce 'via' lists '" + v +
                            "' more than once"};
                    }
                }
                chain.push_back(bi);
            }
            while ((int)chain.size() < n) {
                // synthesized intermediate: '<A>_1', '<A>_2', ...
                const std::string want = rd.bodyA + "_" +
                    std::to_string((int)chain.size());
                if (m.bodyIndex(want) >= 0) {
                    throw SemanticError{rd.line, "auto-named shaft '" + want +
                        "' collides with an existing body — use 'via' to name "
                        "the intermediates yourself"};
                }
                Body b;
                b.name = want;
                b.isGround = false;
                b.isShaft = true;
                b.declLine = rd.line;
                b.qOffset = m.nq();          // slot assigned pre-push (invariant)
                Anchor A; A.name = "A"; A.local = Vec2{0.0, 0.0};
                b.anchors.push_back(A);
                m.bodies.push_back(b);
                chain.push_back((int)m.bodies.size() - 1);
            }
            chain.push_back(biB);

            // -- idler shaft (direction control) ----------------------------
            // The idler sits right after A: A's stage wheel meshes the idler's
            // gear, which meshes the first pinion. One gear, two meshes, one
            // extra sense flip, ratio contribution 1.
            int idlerBody = -1, idlerTeeth = 0;
            if (idler) {
                idlerTeeth = (int)std::lround(std::sqrt(
                    (double)stages[0].w * (double)stages[0].p));
                if (idlerTeeth < 16) idlerTeeth = 16;
                if (idlerTeeth > 64) idlerTeeth = 64;
                const std::string want = uniqueBodyName(rd.bodyA + "_idler");
                Body b;
                b.name = want;
                b.isGround = false;
                b.isShaft = true;
                b.declLine = rd.line;
                b.qOffset = m.nq();
                Anchor A; A.name = "A"; A.local = Vec2{0.0, 0.0};
                b.anchors.push_back(A);
                m.bodies.push_back(b);
                idlerBody = (int)m.bodies.size() - 1;
            }

            // -- layout: pin the unpinned shafts of the chain ----------------
            const double module = rd.hasModule ? rd.module : 0.01;
            if (module <= 0.0) {
                throw SemanticError{rd.line, "reduce 'module' must be positive"};
            }
            const double spread = deg2rad(rd.hasSpread ? rd.spreadDeg
                                                        : kSynthSpreadDeg);
            const double turn = deg2rad(kSynthTurnDeg);

            // The train as an ordered line of bodies. With an idler it is
            //   A -> idler -> c1 -> ... -> B
            // and the first stage's two meshes both run through the idler's
            // single gear (ratio contribution 1, one extra sense flip).
            std::vector<int> line;                       // bodies, in order
            for (size_t k = 0; k + 1 < chain.size(); ++k) {
                line.push_back(chain[k]);
                if (idlerBody >= 0 && k == 0) line.push_back(idlerBody);
            }
            line.push_back(chain.back());
            const int nLinks = (int)line.size() - 1;

            // link j connects line[j] -> line[j+1]; a reduction stage carries
            // its PINION on the driving (fast) body and its WHEEL on the
            // driven (slow) one. fromTeeth/toTeeth are the actual gears at
            // each end (for the idler links they involve the shared gear).
            struct Link { int from, to; int fromTeeth, toTeeth; bool viaIdler; };
            std::vector<Link> links;
            for (int j = 0; j < nLinks; ++j) {
                if (idlerBody >= 0 && j == 0) {
                    // A's stage-1 pinion -> idler gear
                    links.push_back(Link{line[0], line[1], stages[0].p, idlerTeeth, true});
                } else if (idlerBody >= 0 && j == 1) {
                    // idler gear -> chain's stage-1 wheel
                    links.push_back(Link{line[1], line[2], idlerTeeth, stages[0].w, true});
                } else {
                    // stage index: with an idler the first two links consumed
                    // stage 0, so link j maps to stage j-1
                    const int s = idlerBody >= 0 ? j - 1 : j;
                    links.push_back(Link{line[j], line[j + 1],
                                         stages[s].p, stages[s].w, false});
                }
            }
            std::vector<double> d((size_t)nLinks);
            for (int j = 0; j < nLinks; ++j)
                d[(size_t)j] = 0.5 * module *
                               (links[(size_t)j].fromTeeth + links[(size_t)j].toTeeth);

            // static positions along the line (pinned or placed earlier)
            struct Node { int body; Vec2 pos; bool known; };
            std::vector<Node> nodes;
            for (int bi : line) {
                Vec2 p;
                const bool ok = staticPos(bi, p);
                nodes.push_back(Node{bi, ok ? p : Vec2{0, 0}, ok});
            }
            if (!nodes[0].known) {
                throw SemanticError{rd.line, "reduce needs '" + rd.bodyA +
                    "' pinned to ground (or placed by an earlier reduce) — "
                    "the train is laid out from there"};
            }

            ReduceInfo info;
            info.bodyA = rd.bodyA;
            info.bodyB = rd.bodyB;
            info.module = module;
            info.idlers = idler ? 1 : 0;
            info.idlerTeeth = idlerTeeth;
            info.line = rd.line;
            for (const StageCand& c : stages)
                info.stages.push_back({c.w, c.p});
            info.opposite = ((nLinks % 2) == 1);

            // walk the line, placing free bodies
            auto dirAt = [&](int j) {
                const double a = spread + (double)j * turn;
                return Vec2{std::cos(a), std::sin(a)};
            };
            for (int j = 0; j < nLinks; ++j) {
                Node& from = nodes[(size_t)j];
                Node& to = nodes[(size_t)(j + 1)];
                if (!to.known) {
                    Vec2 pos;
                    if (j + 1 < nLinks && nodes[(size_t)(j + 2)].known) {
                        // the next link's far end is pinned: place this node by
                        // circle intersection so both meshes come out tangent
                        Vec2 s1, s2;
                        bool uniq = false;
                        if (!circleIntersect(from.pos, nodes[(size_t)(j + 2)].pos,
                                             d[(size_t)j], d[(size_t)(j + 1)], s1, s2, uniq)) {
                            char buf[384];
                            std::snprintf(buf, sizeof buf,
                                "reduce cannot place '%s': the pinned positions "
                                "of '%s' and '%s' do not admit tangent meshes "
                                "(needs distances %.4f and %.4f from a common "
                                "point) — pin fewer shafts or adjust 'module'",
                                m.bodies[to.body].name.c_str(),
                                m.bodies[from.body].name.c_str(),
                                m.bodies[nodes[(size_t)(j + 2)].body].name.c_str(),
                                d[(size_t)j], d[(size_t)(j + 1)]);
                            throw SemanticError{rd.line, buf};
                        }
                        const Vec2 want = from.pos + d[(size_t)j] * dirAt(j);
                        pos = (vlen(s1 - want) <= vlen(s2 - want)) ? s1 : s2;
                    } else {
                        pos = from.pos + d[(size_t)j] * dirAt(j);
                    }
                    to.pos = pos;
                    to.known = true;
                    synthPos[to.body] = pos;
                    const int anch = groundAnchorFor(m.bodies[to.body].name, pos);
                    PinJoint pj;
                    pj.bodyA = to.body;
                    pj.anchA = 0;
                    pj.bodyB = 0;
                    pj.anchB = anch;
                    pj.line = rd.line;
                    m.pins.push_back(pj);
                    info.placed.push_back(m.bodies[to.body].name);
                }
                // both ends known (author-pinned): tangency is validated by
                // validateAssembly with a precise message — nothing to do here
            }

            // -- gears and meshes along the line ------------------------------
            // Reduction anatomy: each stage's pinion rides the driving (fast)
            // body and its wheel the driven (slow) body, so w_out = w_in x
            // (p/w) per stage. Intermediates end up compound: wheel of the
            // stage above, pinion of the stage below. The idler's single gear
            // plays pinion-then-wheel (ratio contribution 1, one extra flip).
            int idlerGear = -1;
            if (idlerBody >= 0) {
                idlerGear = addSynthGear(m.bodies[(size_t)idlerBody].name + "_g",
                                         idlerBody, idlerTeeth, module);
            }
            for (int j = 0; j < nLinks; ++j) {
                const Link& L = links[(size_t)j];
                // the 1-based stage number this link belongs to (for names)
                const int stageNo = idlerBody >= 0 ? (j <= 1 ? 1 : j) : j + 1;
                // gear on the driving end (a pinion, or the shared idler gear)
                int fromGear;
                if (L.viaIdler && j == 1) {
                    fromGear = idlerGear;
                } else {
                    fromGear = addSynthGear(m.bodies[(size_t)L.from].name + "_p" +
                                            std::to_string(stageNo),
                                            L.from, L.fromTeeth, module);
                }
                // gear on the driven end (a wheel, or the shared idler gear)
                int toGear;
                if (L.viaIdler && j == 0) {
                    toGear = idlerGear;
                } else {
                    toGear = addSynthGear(m.bodies[(size_t)L.to].name + "_w" +
                                          std::to_string(stageNo),
                                          L.to, L.toTeeth, module);
                }
                addSynthMesh(fromGear, toGear, rd.line);
            }
            m.reduceInfos.push_back(info);
        }
    }

    // --- M4 hands: render-only needles on rotating hosts ---------------------
    std::unordered_set<std::string> handNames;
    for (const HandDecl& hd : decl.hands) {
        if (!handNames.insert(hd.name).second) {
            throw SemanticError{hd.line, "duplicate hand name '" + hd.name + "'"};
        }
        Hand h;
        h.name = hd.name;
        h.line = hd.line;
        h.body = m.bodyIndex(hd.host);
        if (h.body < 0) {
            throw SemanticError{hd.line, "hand '" + hd.name +
                "' refers to unknown host body '" + hd.host + "'"};
        }
        if (m.bodies[(size_t)h.body].isGround) {
            throw SemanticError{hd.line, "hand '" + hd.name +
                "' is on ground '" + hd.host + "' — a hand on ground never moves"};
        }
        const std::string anchName = hd.hostAnchor.empty() ? "A" : hd.hostAnchor;
        h.mountAnchor = m.anchorIndex(h.body, anchName);
        if (h.mountAnchor < 0) {
            throw SemanticError{hd.line, "hand '" + hd.name + "': host body '" +
                hd.host + "' has no anchor '" + anchName + "'"};
        }
        h.length = hd.props.count("length") ? hd.props.at("length") : 0.3;
        if (h.length <= 0.0) {
            throw SemanticError{hd.line, "hand '" + hd.name +
                "' needs a positive 'length'"};
        }
        m.hands.push_back(h);
    }

    // --- M4 dials: static tick rings (center/radius may be captured later) ---
    for (const DialDecl& dd : decl.dials) {
        Dial d;
        d.line = dd.line;
        if (dd.hasAt) {
            const int bi = m.bodyIndex(dd.atBody);
            if (bi < 0) {
                throw SemanticError{dd.line, "dial refers to unknown body '" +
                    dd.atBody + "'"};
            }
            if (!m.bodies[(size_t)bi].isGround) {
                throw SemanticError{dd.line, "dial 'at' must reference a ground "
                    "anchor (the dial is static) — '" + dd.atBody +
                    "' is a moving body"};
            }
            const int ai = m.anchorIndex(bi, dd.atAnchor);
            if (ai < 0) {
                throw SemanticError{dd.line, "ground '" + dd.atBody +
                    "' has no anchor '" + dd.atAnchor + "'"};
            }
            d.hasCenter = true;
            d.center = m.bodies[(size_t)bi].anchors[(size_t)ai].local;
        } else if (decl.hands.empty()) {
            throw SemanticError{dd.line, "dial has no 'at' and there are no "
                "hands to auto-center on — add 'at frame.p' or a hand"};
        }
        if (dd.props.count("radius")) {
            const double r = dd.props.at("radius");
            if (r <= 0.0) {
                throw SemanticError{dd.line, "dial needs a positive 'radius'"};
            }
            d.hasRadius = true;
            d.radius = r;
        }
        if (dd.props.count("major")) {
            d.major = (int)dd.props.at("major");
            if (d.major < 0) throw SemanticError{dd.line,
                "dial 'major' must be a non-negative integer"};
        }
        if (dd.props.count("minor")) {
            d.minor = (int)dd.props.at("minor");
            if (d.minor < 0) throw SemanticError{dd.line,
                "dial 'minor' must be a non-negative integer"};
        }
        if (d.major > 0 && d.minor > 0 && d.minor % d.major != 0) {
            throw SemanticError{dd.line, "dial 'minor' must be a multiple of "
                "'major' (e.g. 60 and 12)"};
        }
        m.dials.push_back(d);
    }

    // --- M4 floating check: a bare shaft must end up pinned somewhere --------
    for (const BodyDecl& bd : decl.bodies) {
        if (bd.kind != BodyDecl::Kind::Shaft) continue;
        const int bi = m.bodyIndex(bd.name);
        bool pinned = false;
        for (const PinJoint& p : m.pins) {
            if (p.bodyA == bi || p.bodyB == bi) { pinned = true; break; }
        }
        if (!pinned) {
            throw SemanticError{bd.line, "shaft '" + bd.name + "' is floating: "
                "pin it with 'at frame.p', list it in a reduce 'via', or drop "
                "the declaration (a reduce without 'via' synthesizes its own "
                "intermediates)"};
        }
    }

    // --- springs ----------------------------------------------------------------
    std::unordered_set<std::string> springNames;
    for (const SpringDecl& sd : decl.springs) {
        if (!springNames.insert(sd.name).second) {
            throw SemanticError{sd.line, "duplicate spring name '" + sd.name + "'"};
        }
        SpringJoint s;
        s.name = sd.name;
        s.line = sd.line;
        s.k = sd.props.count("k") ? sd.props.at("k") : -1.0;
        if (s.k <= 0.0) {
            throw SemanticError{sd.line, "spring '" + sd.name +
                "' needs a positive 'k' property"};
        }
        s.rest = sd.props.count("rest") ? sd.props.at("rest") : -1.0;  // captured later
        if (s.rest < 0.0 && sd.props.count("rest")) {
            throw SemanticError{sd.line, "spring '" + sd.name +
                "' needs a non-negative 'rest' length"};
        }
        s.damp = sd.props.count("damp") ? sd.props.at("damp") : 0.0;
        if (s.damp < 0.0) {
            throw SemanticError{sd.line, "spring '" + sd.name +
                "' needs a non-negative 'damp'"};
        }
        const int biA = m.bodyIndex(sd.bodyA);
        if (biA < 0) throw SemanticError{sd.line,
            "spring '" + sd.name + "' refers to unknown body '" + sd.bodyA + "'"};
        const int biB = m.bodyIndex(sd.bodyB);
        if (biB < 0) throw SemanticError{sd.line,
            "spring '" + sd.name + "' refers to unknown body '" + sd.bodyB + "'"};
        const int anA = m.anchorIndex(biA, sd.anchorA);
        if (anA < 0) throw SemanticError{sd.line, "body '" + sd.bodyA +
            "' has no anchor '" + sd.anchorA + "'"};
        const int anB = m.anchorIndex(biB, sd.anchorB);
        if (anB < 0) throw SemanticError{sd.line, "body '" + sd.bodyB +
            "' has no anchor '" + sd.anchorB + "'"};
        s.bodyA = biA; s.anchA = anA;
        s.bodyB = biB; s.anchB = anB;
        m.springs.push_back(s);
    }

    // --- drives (rate rows + torque forces + M5 weights) ----------------------
    std::unordered_set<int> driven;
    for (const DriveDecl& dd : decl.drives) {
        const int bi = m.bodyIndex(dd.body);
        if (bi < 0) {
            if (m.gearIndex(dd.body) >= 0) {
                throw SemanticError{dd.line, "'" + dd.body +
                    "' is a gear — drive its host body instead"};
            }
            throw SemanticError{dd.line, "drive refers to unknown body '" + dd.body + "'"};
        }
        if (m.bodies[bi].isGround) {
            throw SemanticError{dd.line, "cannot drive the ground body '" + dd.body + "'"};
        }
        if (!driven.insert(bi).second) {
            throw SemanticError{dd.line, "body '" + dd.body + "' is driven more than once"};
        }
        if (dd.isTorque) {
            TorqueDrive t;
            t.body = bi;
            t.torque = dd.torque;
            t.line = dd.line;
            m.torques.push_back(t);
        } else if (dd.isWeight) {
            // M5: a weight is a bounded energy source. The cord's barrel
            // defaults to the host's largest gear (the wheel is the drum).
            WeightDrive w;
            w.body = bi;
            w.mass = dd.weightMass;
            w.barrelR = dd.hasBarrel ? dd.barrelR : -1.0;
            if (w.barrelR < 0.0) {
                double mx = 0.0;
                for (const Gear& g : m.gears)
                    if (g.body == bi) mx = std::max(mx, g.radius);
                if (mx <= 0.0) {
                    throw SemanticError{dd.line, "weight drive on '" + dd.body +
                        "' needs a 'barrel' radius — the body hosts no gear to "
                        "default the drum to"};
                }
                w.barrelR = mx;
            }
            w.torque = w.mass * m.world.gravity * w.barrelR;
            w.sign = dd.weightSign;
            w.drop = dd.hasDrop ? dd.dropM : -1.0;
            w.line = dd.line;
            m.weights.push_back(w);
        } else {
            Drive d;
            d.line = dd.line;
            d.body = bi;
            d.omegaRad = dd.omegaRad;
            d.theta0Rad = dd.theta0Rad;
            m.drives.push_back(d);
        }
    }

    // --- M5 hybrid elements: modes --------------------------------------------
    // Resolves states/transitions against bodies, dedupes held bodies into
    // one rate-0 drive row per body, and enables the initial states' holds
    // (their targets are captured at assembly time, like spring rests).
    // A hold coexists legally with a torque/weight drive on the same body —
    // the escapement needs exactly that pair — but not with a rate drive.
    {
        std::unordered_set<int> heldBodies;   // across all modes
        for (const ModeDecl& md : decl.modes) {
            Mode mo;
            mo.name = md.name;
            mo.line = md.line;
            if (md.states.empty()) {
                throw SemanticError{md.line, "mode '" + md.name +
                    "' has no states — declare at least one 'state'"};
            }
            // states (unique names, hold bodies resolved)
            std::map<std::string, int> stateIdx;
            for (const ModeDecl::State& sd : md.states) {
                if (stateIdx.count(sd.name)) {
                    throw SemanticError{sd.line, "mode '" + md.name +
                        "' declares state '" + sd.name + "' more than once"};
                }
                Mode::State st;
                st.name = sd.name;
                for (const std::string& h : sd.holds) {
                    const int bi = m.bodyIndex(h);
                    if (bi < 0) {
                        throw SemanticError{sd.line, "mode '" + md.name +
                            "' holds unknown body '" + h + "'"};
                    }
                    if (m.bodies[bi].isGround) {
                        throw SemanticError{sd.line, "mode '" + md.name +
                            "' cannot hold ground body '" + h + "'"};
                    }
                    if (heldBodies.count(bi)) {
                        throw SemanticError{sd.line, "body '" + h +
                            "' is held by more than one mode — one hold row per body"};
                    }
                    if (m.bodyDriven(bi)) {
                        throw SemanticError{sd.line, "body '" + h +
                            "' is driven by a rate drive — a mode hold cannot "
                            "pin a driven body"};
                    }
                    for (int prev : st.holdBodies)
                        if (prev == bi)
                            throw SemanticError{sd.line, "state '" + sd.name +
                                "' holds body '" + h + "' more than once"};
                    st.holdBodies.push_back(bi);
                }
                stateIdx[sd.name] = (int)mo.states.size();
                mo.states.push_back(st);
            }
            // transitions (states resolved; guards and resets resolved)
            std::map<int, int> guardSlot;   // body -> entryTheta slot
            for (const ModeDecl::Transition& td : md.transitions) {
                ModeTransition tr;
                tr.line = td.line;
                auto itF = stateIdx.find(td.from);
                if (itF == stateIdx.end()) {
                    throw SemanticError{td.line, "transition of mode '" + md.name +
                        "' starts in unknown state '" + td.from + "'"};
                }
                auto itT = stateIdx.find(td.to);
                if (itT == stateIdx.end()) {
                    throw SemanticError{td.line, "transition of mode '" + md.name +
                        "' goes to unknown state '" + td.to + "'"};
                }
                tr.from = itF->second;
                tr.to = itT->second;
                tr.body = m.bodyIndex(td.body);
                if (tr.body < 0) {
                    throw SemanticError{td.line, "guard of mode '" + md.name +
                        "' watches unknown body '" + td.body + "'"};
                }
                if (m.bodies[(size_t)tr.body].isGround) {
                    throw SemanticError{td.line, "guard of mode '" + md.name +
                        "' watches ground body '" + td.body +
                        "' — it never moves"};
                }
                tr.field = (ModeTransition::Field)td.field;
                tr.greater = td.greater;
                tr.threshold = td.threshold;
                // every guard body gets an advance-origin slot
                if (!guardSlot.count(tr.body)) {
                    guardSlot[tr.body] = (int)mo.guardBodies.size();
                    mo.guardBodies.push_back(tr.body);
                }
                for (const ModeDecl::Reset& rd : td.resets) {
                    ModeReset rs;
                    rs.line = rd.line;
                    rs.body = m.bodyIndex(rd.body);
                    if (rs.body < 0) {
                        throw SemanticError{rd.line, "impulse of mode '" + md.name +
                            "' targets unknown body '" + rd.body + "'"};
                    }
                    if (m.bodies[(size_t)rs.body].isGround) {
                        throw SemanticError{rd.line, "impulse of mode '" + md.name +
                            "' targets ground body '" + rd.body + "'"};
                    }
                    rs.maintain = rd.kind == ModeDecl::Reset::Kind::Maintain;
                    rs.energy = rd.energy;
                    rs.swingRad = deg2rad(rd.swingDeg);
                    tr.resets.push_back(rs);
                }
                mo.transitions.push_back(tr);
            }
            // held-body union -> one rate-0 drive row each
            for (const Mode::State& st : mo.states) {
                for (int bi : st.holdBodies) {
                    if (std::find(mo.holdBodies.begin(), mo.holdBodies.end(), bi)
                        != mo.holdBodies.end()) {
                        continue;   // same body held elsewhere: same row
                    }
                    if (!heldBodies.insert(bi).second) {
                        throw SemanticError{mo.line, "body '" + m.bodies[(size_t)bi].name +
                            "' is held by more than one mode — one hold row per body"};
                    }
                    Drive d;
                    d.body = bi;
                    d.omegaRad = 0.0;
                    d.theta0Rad = 0.0;   // captured at assembly
                    d.line = mo.line;
                    d.enabled = false;   // set for the initial state below
                    m.drives.push_back(d);
                    mo.holdBodies.push_back(bi);
                    mo.holdDrive.push_back((int)m.drives.size() - 1);
                }
            }
            // enable the initial state's holds
            for (size_t hi = 0; hi < mo.holdBodies.size(); ++hi) {
                m.drives[(size_t)mo.holdDrive[hi]].enabled =
                    mo.stateHolds(0, mo.holdBodies[hi]);
            }
            mo.entryTheta.assign(mo.guardBodies.size(), 0.0);
            m.modes.push_back(mo);
        }
    }

    // --- mass properties (dynamic mechanisms only) -------------------------------
    m.mass.assign(m.bodies.size(), MassProps{});
    if (m.world.dynamic) {
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const Body& b = m.bodies[bi];
            const BodyDecl* bd = nullptr;
            for (const BodyDecl& d : decl.bodies)
                if (d.name == b.name && d.kind != BodyDecl::Kind::Ground) { bd = &d; break; }

            std::vector<MassContribution> parts;
            if (b.length > 0.0) {  // rod: uniform bar of the given/derived mass
                double mm = kRodDensity * b.length;
                if (bd && bd->props.count("mass")) mm = bd->props.at("mass");
                parts.push_back(MassContribution{mm, Vec2{0.5 * b.length, 0.0},
                                                 mm * b.length * b.length / 12.0});
            } else if (b.isShaft) {   // includes synthesized reduce shafts
                parts.push_back(MassContribution{0.05, Vec2{0.0, 0.0}, 1.0e-4});
            } else {  // free body: unit mass at the frame origin unless given
                double mm = 1.0;
                if (bd && bd->props.count("mass")) mm = bd->props.at("mass");
                parts.push_back(MassContribution{mm, Vec2{0.0, 0.0}, 0.01 * mm});
            }

            // gears add discs centered on their host anchor
            for (const Gear& g : m.gears) {
                if (g.body != (int)bi) continue;
                const double disc = g.massOverride >= 0.0
                    ? g.massOverride
                    : kGearArealDensity * kPi * g.radius * g.radius;
                parts.push_back(MassContribution{disc, b.anchors[g.hostAnchor].local,
                                                 0.5 * disc * g.radius * g.radius});
            }

            MassProps mp;
            for (const MassContribution& p : parts) mp.mass += p.m;
            if (mp.mass <= 0.0) {
                throw SemanticError{b.declLine, "body '" + b.name +
                    "' has zero mass — dynamic mechanisms need mass on every body"};
            }
            for (const MassContribution& p : parts)
                mp.com = mp.com + (1.0 / mp.mass) * p.m * p.com;
            for (const MassContribution& p : parts) {
                const Vec2 r = p.com - mp.com;
                mp.inertia += p.I + p.m * dot(r, r);
            }
            m.mass[bi] = mp;
        }
    }

    return m;
}

} // namespace mech
