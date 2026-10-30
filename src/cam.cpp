// mech — cam.cpp: displacement-diagram synthesis of plate cams (M6b)
#include "cam.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mech {

namespace {

// Modified trapezoid, unnormalized (peak acceleration 1). Built from the
// acceleration: a sine ramp over [0, 1/8], a plateau over [1/8, 3/8] and a
// sine pass through zero over [3/8, 5/8] (then the mirror image, negated).
// Integrating in closed form for x <= 1/2 and using the antisymmetry
//   a(1-x) = -a(x),  v(1-x) = v(x),  s(1-x) = S - s(x)
// for the rest. S = s(1) is the normalization.
struct Raw { double s, v, a; };

Raw modTrapHalf(double x) {
    const double c = 1.0 / (4.0 * kPi);
    const double k16 = 1.0 / (16.0 * kPi * kPi);
    const double s1 = c / 8.0 - k16;                 // s(1/8)
    const double V2 = c + 0.25;                      // v(3/8)
    const double s2 = s1 + c / 4.0 + 1.0 / 32.0;     // s(3/8)
    if (x <= 0.125) {
        return {x * c - std::sin(4.0 * kPi * x) * k16,
                (1.0 - std::cos(4.0 * kPi * x)) * c,
                std::sin(4.0 * kPi * x)};
    }
    if (x <= 0.375) {
        const double d = x - 0.125;
        return {s1 + c * d + 0.5 * d * d, c + d, 1.0};
    }
    const double d = x - 0.375;
    return {s2 + V2 * d + (1.0 - std::cos(4.0 * kPi * d)) * k16,
            V2 + std::sin(4.0 * kPi * d) * c,
            std::cos(4.0 * kPi * d)};
}

const double kModTrapTotal = [] {
    const Raw h = modTrapHalf(0.5);
    return 2.0 * h.s;
}();

LawValue modTrap(double x) {
    Raw r;
    if (x <= 0.5) {
        r = modTrapHalf(x);
    } else {
        const Raw m = modTrapHalf(1.0 - x);
        r = {kModTrapTotal - m.s, m.v, -m.a};
    }
    return {r.s / kModTrapTotal, r.v / kModTrapTotal, r.a / kModTrapTotal};
}

} // namespace

LawValue evalLaw(MotionLaw law, double x) {
    x = std::min(1.0, std::max(0.0, x));
    switch (law) {
    case MotionLaw::Cycloidal: {
        const double w = 2.0 * kPi;
        return {x - std::sin(w * x) / w, 1.0 - std::cos(w * x), w * std::sin(w * x)};
    }
    case MotionLaw::Harmonic:
        return {0.5 * (1.0 - std::cos(kPi * x)),
                0.5 * kPi * std::sin(kPi * x),
                0.5 * kPi * kPi * std::cos(kPi * x)};
    case MotionLaw::ModTrap:
        return modTrap(x);
    }
    return {0.0, 0.0, 0.0};
}

const char* lawName(MotionLaw law) {
    switch (law) {
    case MotionLaw::Cycloidal: return "cycloidal";
    case MotionLaw::Harmonic:  return "harmonic";
    case MotionLaw::ModTrap:   return "modtrap";
    }
    return "?";
}

// ---- the diagram -------------------------------------------------------------

bool CamDiagram::build(const CamSpec& spec, CamError& err) {
    segs_.clear();
    maxLift_ = 0.0;
    const size_t n = spec.segments.size();
    if (n == 0) {
        err = {-1, "has no motion segments — give it 'rise', 'dwell', 'return'..."};
        return false;
    }
    const double full = 2.0 * kPi, tol = 1e-9;
    double th = 0.0, s = 0.0;
    char buf[200];

    for (size_t i = 0; i < n; ++i) {
        const CamSegment& cs = spec.segments[i];
        Seg g;
        g.th0 = th;
        g.s0 = s;
        g.law = cs.law;
        g.dwell = cs.kind == CamSegment::Kind::Dwell;

        if (cs.restOfTurn) {
            if (i + 1 != n) {
                err = {(int)i, "'dwell' without an angle must be the last segment"};
                return false;
            }
            g.beta = full - th;
            if (g.beta <= tol) {
                err = {(int)i, "no angle left for the closing 'dwell' — the diagram "
                               "already covers a full turn"};
                return false;
            }
        } else {
            if (!(cs.angle > 0.0)) {
                err = {(int)i, "segment needs a positive angle"};
                return false;
            }
            g.beta = cs.angle;
        }

        switch (cs.kind) {
        case CamSegment::Kind::Dwell:
            break;
        case CamSegment::Kind::Rise:
            if (!(cs.lift > 0.0)) { err = {(int)i, "'rise' needs a positive lift"}; return false; }
            g.lift = cs.lift;
            break;
        case CamSegment::Kind::Fall:
            if (!(cs.lift > 0.0)) { err = {(int)i, "'fall' needs a positive lift"}; return false; }
            g.lift = -cs.lift;
            break;
        case CamSegment::Kind::Return:
            if (s <= tol) {
                err = {(int)i, "'return' has nothing to return from — the follower "
                               "is already on the base circle"};
                return false;
            }
            g.lift = -s;
            break;
        }

        s += g.lift;
        if (s < -tol) {
            std::snprintf(buf, sizeof buf, "'fall' of %g m goes below the base circle "
                          "(the follower is only %g m above it)", -g.lift, g.s0);
            err = {(int)i, buf};
            return false;
        }
        if (std::fabs(s) <= tol) s = 0.0;
        th += g.beta;
        maxLift_ = std::max(maxLift_, s);
        segs_.push_back(g);
    }

    if (th > full + tol) {
        std::snprintf(buf, sizeof buf, "diagram covers %g degrees — more than one turn",
                      rad2deg(th));
        err = {-1, buf};
        return false;
    }
    if (th < full - tol) {
        std::snprintf(buf, sizeof buf, "diagram covers %g of 360 degrees — finish with "
                      "'dwell' to fill the turn", rad2deg(th));
        err = {-1, buf};
        return false;
    }
    if (s > tol) {
        std::snprintf(buf, sizeof buf, "diagram does not close: the follower ends %g m "
                      "above the base — add a 'return' (or a matching 'fall')", s);
        err = {-1, buf};
        return false;
    }
    segs_.back().beta += full - th;   // absorb sub-tolerance rounding
    return true;
}

void CamDiagram::eval(double theta, double& s, double& ds, double& dds) const {
    const double full = 2.0 * kPi;
    const double w = theta - full * std::floor(theta / full);
    size_t i = segs_.size() - 1;
    for (size_t k = 0; k < segs_.size(); ++k) {
        if (w < segs_[k].th0 + segs_[k].beta) { i = k; break; }
    }
    const Seg& g = segs_[i];
    if (g.dwell) {
        s = g.s0;
        ds = dds = 0.0;
        return;
    }
    const LawValue f = evalLaw(g.law, (w - g.th0) / g.beta);
    s = g.s0 + g.lift * f.s;
    ds = g.lift * f.v / g.beta;
    dds = g.lift * f.a / (g.beta * g.beta);
}

// ---- cutting the cam ----------------------------------------------------------

namespace {

// Pitch point and its first two derivatives w.r.t. theta, plus s and s'.
//   P  = r u,      u = (cos phi, sin phi),  phi = start - sigma*theta
//   P' = r' u - sigma r u_perp
//   P''= (r'' - r) u - 2 sigma r' u_perp          (sigma^2 = 1)
struct Pitch { Vec2 P, T, T2; double s, ds, r; };

Pitch pitchAt(const CamSpec& spec, const CamDiagram& dia, double theta) {
    const double sg = (double)spec.turn;
    double s, ds, dds;
    dia.eval(theta, s, ds, dds);
    const double r = spec.base + spec.roller + s;
    const double phi = spec.startRad - sg * theta;
    const Vec2 u{std::cos(phi), std::sin(phi)}, up{-std::sin(phi), std::cos(phi)};
    Pitch p;
    p.P = r * u;
    p.T = ds * u + (-sg * r) * up;
    p.T2 = (dds - r) * u + (-2.0 * sg * ds) * up;
    p.s = s;
    p.ds = ds;
    p.r = r;
    return p;
}

} // namespace

CamDesign designCam(const CamSpec& spec, const CamDiagram& dia) {
    CamDesign d;
    d.maxLift = dia.maxLift();
    const double sg = (double)spec.turn;
    const double full = 2.0 * kPi;

    // diagnostics on a fine grid (0.05 degrees), independent of the surface samples
    const int M = 7200;
    for (int k = 0; k < M; ++k) {
        const double th = full * (double)k / (double)M;
        const Pitch p = pitchAt(spec, dia, th);
        const double alpha = std::atan2(std::fabs(p.ds), p.r);
        if (alpha > d.maxPressure) { d.maxPressure = alpha; d.maxPressureAt = th; }
        const double tl = vlen(p.T);
        const double kappa = -sg * crossz(p.T, p.T2) / (tl * tl * tl);   // > 0: convex
        if (kappa > 1e-12 && 1.0 / kappa < d.minConvexRadius) {
            d.minConvexRadius = 1.0 / kappa;
            d.minConvexAt = th;
        }
    }
    d.undercut = spec.roller > 0.0 && d.minConvexRadius <= spec.roller;

    // the surface: the pitch curve offset inward by the roller radius
    const int N = spec.samples;
    d.surface.reserve((size_t)N);
    for (int k = 0; k < N; ++k) {
        const Pitch p = pitchAt(spec, dia, full * (double)k / (double)N);
        const Vec2 nOut = (sg / vlen(p.T)) * Vec2{-p.T.y, p.T.x};
        d.surface.push_back(p.P - spec.roller * nOut);
    }
    return d;
}

} // namespace mech
