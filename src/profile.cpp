// mech — profile.cpp: closed planar curves for contact (M6a)
#include "profile.hpp"

#include <algorithm>
#include <cmath>

namespace mech {

namespace {

// Solve a cyclic tridiagonal system (Sherman-Morrison on the tridiagonal
// part). Row i reads  a[i] x[i-1] + b[i] x[i] + c[i] x[i+1] = r[i]  with
// indices mod n; n >= 3.
std::vector<double> solveCyclic(const std::vector<double>& a, const std::vector<double>& b,
                                const std::vector<double>& c, const std::vector<double>& r) {
    const size_t n = b.size();
    const double alpha = c[n - 1];   // A[n-1][0]
    const double beta = a[0];        // A[0][n-1]
    const double gamma = -b[0];

    auto thomas = [&](const std::vector<double>& diag, const std::vector<double>& rhs) {
        std::vector<double> cp(n), dp(n), x(n);
        cp[0] = c[0] / diag[0];
        dp[0] = rhs[0] / diag[0];
        for (size_t i = 1; i < n; ++i) {
            const double den = diag[i] - a[i] * cp[i - 1];
            cp[i] = i + 1 < n ? c[i] / den : 0.0;
            dp[i] = (rhs[i] - a[i] * dp[i - 1]) / den;
        }
        x[n - 1] = dp[n - 1];
        for (size_t i = n - 1; i-- > 0;) x[i] = dp[i] - cp[i] * x[i + 1];
        return x;
    };

    std::vector<double> bb(b);
    bb[0] = b[0] - gamma;
    bb[n - 1] = b[n - 1] - alpha * beta / gamma;
    std::vector<double> u(n, 0.0);
    u[0] = gamma;
    u[n - 1] = alpha;

    const std::vector<double> y = thomas(bb, r);
    const std::vector<double> z = thomas(bb, u);
    const double fact = (y[0] + beta * y[n - 1] / gamma) /
                        (1.0 + z[0] + beta * z[n - 1] / gamma);
    std::vector<double> x(n);
    for (size_t i = 0; i < n; ++i) x[i] = y[i] - fact * z[i];
    return x;
}

} // namespace

// ---- periodic cubic spline ----------------------------------------------------

void ProfileCurve::Spline::build(const std::vector<double>& knots,
                                 const std::vector<double>& vals, double per) {
    t = knots;
    y = vals;
    period = per;
    const size_t n = t.size();
    std::vector<double> h(n);
    for (size_t i = 0; i + 1 < n; ++i) h[i] = t[i + 1] - t[i];
    h[n - 1] = t[0] + period - t[n - 1];

    std::vector<double> a(n), b(n), c(n), r(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t im = (i + n - 1) % n, ip = (i + 1) % n;
        const double hm = h[im], hp = h[i];
        a[i] = hm / 6.0;
        b[i] = (hm + hp) / 3.0;
        c[i] = hp / 6.0;
        r[i] = (y[ip] - y[i]) / hp - (y[i] - y[im]) / hm;
    }
    M = solveCyclic(a, b, c, r);
}

void ProfileCurve::Spline::eval(double s, double& v, double& d1, double& d2) const {
    const size_t n = t.size();
    double w = s - t[0];
    w -= period * std::floor(w / period);
    const double sw = t[0] + w;
    size_t i = (size_t)(std::upper_bound(t.begin(), t.end(), sw) - t.begin());
    i = i == 0 ? 0 : i - 1;
    const size_t ip = (i + 1) % n;
    const double t0 = t[i];
    const double t1 = i + 1 < n ? t[i + 1] : t[0] + period;
    const double h = t1 - t0;
    const double A = (t1 - sw) / h, B = (sw - t0) / h;
    v = A * y[i] + B * y[ip] +
        ((A * A * A - A) * M[i] + (B * B * B - B) * M[ip]) * h * h / 6.0;
    d1 = (y[ip] - y[i]) / h - (3.0 * A * A - 1.0) / 6.0 * h * M[i] +
         (3.0 * B * B - 1.0) / 6.0 * h * M[ip];
    d2 = A * M[i] + B * M[ip];
}

// ---- construction -----------------------------------------------------------

ProfileCurve ProfileCurve::circle(double radius) {
    ProfileCurve c;
    c.kind_ = Kind::Circle;
    c.radius_ = radius;
    c.period_ = 2.0 * kPi;
    c.finish();
    return c;
}

ProfileCurve ProfileCurve::ellipse(double a, double b, bool focus) {
    ProfileCurve c;
    c.kind_ = Kind::Ellipse;
    c.ea_ = a;
    c.eb_ = b;
    c.efocus_ = focus;
    c.ecx_ = focus ? std::sqrt(std::max(0.0, a * a - b * b)) : 0.0;
    c.period_ = 2.0 * kPi;
    c.finish();
    return c;
}

ProfileCurve ProfileCurve::polygon(const std::vector<Vec2>& in, double fillet) {
    std::vector<Vec2> v(in);
    double area2 = 0.0;
    for (size_t i = 0; i < v.size(); ++i) area2 += crossz(v[i], v[(i + 1) % v.size()]);
    if (area2 < 0.0) std::reverse(v.begin(), v.end());

    const size_t n = v.size();
    struct Corner { Vec2 T1, T2, C; double r, a0, sweep, sgn; bool arc; };
    std::vector<Corner> cn(n);
    double minF = 1e300;
    for (size_t i = 0; i < n; ++i) {
        const Vec2 P = v[(i + n - 1) % n], V = v[i], N = v[(i + 1) % n];
        const Vec2 d1 = normalize(V - P), d2 = normalize(N - V);
        const double turn = std::atan2(crossz(d1, d2), dot(d1, d2));    // signed, left turn > 0
        Corner c;
        c.arc = std::fabs(turn) > 1e-9;
        c.sgn = turn >= 0.0 ? 1.0 : -1.0;
        if (!c.arc) {
            c.T1 = c.T2 = V;
            c.r = c.a0 = c.sweep = 0.0;
            c.C = V;
        } else {
            const double half = 0.5 * std::fabs(turn);
            const double tmax = 0.5 * std::min(vlen(V - P), vlen(N - V));   // half an edge each
            const double t = std::min(fillet * std::tan(half), tmax);
            c.r = t / std::tan(half);
            c.T1 = V - t * d1;
            c.T2 = V + t * d2;
            c.C = c.T1 + (c.sgn * c.r) * perp(d1);
            c.a0 = std::atan2(c.T1.y - c.C.y, c.T1.x - c.C.x);
            c.sweep = std::fabs(turn);
            minF = std::min(minF, c.r);
        }
        cn[i] = c;
    }

    ProfileCurve pc;
    pc.kind_ = Kind::Polygon;
    pc.minFillet_ = minF > 1e299 ? 0.0 : minF;
    double s = 0.0;
    auto addLine = [&](Vec2 a, Vec2 b) {
        const double L = vlen(b - a);
        if (L < 1e-12) return;
        Prim q;
        q.arc = false;
        q.a = a;
        q.dir = (1.0 / L) * (b - a);
        q.len = L;
        q.s0 = s;
        s += L;
        pc.prims_.push_back(q);
    };
    for (size_t i = 0; i < n; ++i) {
        if (cn[i].arc) {
            Prim q;
            q.arc = true;
            q.c = cn[i].C;
            q.r = cn[i].r;
            q.a0 = cn[i].a0;
            q.sgn = cn[i].sgn;
            q.len = cn[i].r * cn[i].sweep;
            q.s0 = s;
            s += q.len;
            pc.prims_.push_back(q);
        }
        addLine(cn[i].T2, cn[(i + 1) % n].T1);
    }
    pc.period_ = s;
    pc.finish();
    return pc;
}

ProfileCurve ProfileCurve::polar(const std::vector<double>& angles,
                                 const std::vector<double>& radii) {
    ProfileCurve c;
    c.kind_ = Kind::Polar;
    c.period_ = 2.0 * kPi;
    c.a_.build(angles, radii, c.period_);
    c.finish();
    return c;
}

ProfileCurve ProfileCurve::points(const std::vector<Vec2>& in) {
    std::vector<Vec2> pts(in);
    double area2 = 0.0;
    for (size_t i = 0; i < pts.size(); ++i)
        area2 += crossz(pts[i], pts[(i + 1) % pts.size()]);
    if (area2 < 0.0) std::reverse(pts.begin(), pts.end());

    const size_t n = pts.size();
    std::vector<double> knots(n), xs(n), ys(n);
    double s = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if (i > 0) s += vlen(pts[i] - pts[i - 1]);
        knots[i] = s;
        xs[i] = pts[i].x;
        ys[i] = pts[i].y;
    }
    ProfileCurve c;
    c.kind_ = Kind::Points;
    c.period_ = s + vlen(pts[0] - pts[n - 1]);
    c.a_.build(knots, xs, c.period_);
    c.b_.build(knots, ys, c.period_);
    c.finish();
    return c;
}

void ProfileCurve::finish() {
    if (kind_ == Kind::Polygon) {      // no sample table: the foot search is analytic
        maxRadius_ = 0.0;
        minRadius_ = 1e300;
        const int M = 4096;
        for (int k = 0; k < M; ++k) {
            Vec2 p, d1, d2;
            eval(period_ * (double)k / (double)M, p, d1, d2);
            maxRadius_ = std::max(maxRadius_, vlen(p));
            minRadius_ = std::min(minRadius_, vlen(p));
        }
        return;
    }
    const size_t knots = a_.t.size();
    const size_t K = std::max<size_t>(256, 8 * knots);
    sampleS_.resize(K);
    sampleP_.resize(K);
    maxRadius_ = 0.0;
    minRadius_ = 1e300;
    for (size_t k = 0; k < K; ++k) {
        const double s = period_ * (double)k / (double)K;
        Vec2 p, d1, d2;
        eval(s, p, d1, d2);
        sampleS_[k] = s;
        sampleP_[k] = p;
        maxRadius_ = std::max(maxRadius_, vlen(p));
        minRadius_ = std::min(minRadius_, vlen(p));
    }
}

// ---- evaluation ---------------------------------------------------------------

void ProfileCurve::eval(double s, Vec2& p, Vec2& d1, Vec2& d2) const {
    switch (kind_) {
    case Kind::Circle: {
        const double cs = std::cos(s), sn = std::sin(s);
        p = {radius_ * cs, radius_ * sn};
        d1 = {-radius_ * sn, radius_ * cs};
        d2 = {-radius_ * cs, -radius_ * sn};
        return;
    }
    case Kind::Polar: {
        double r, r1, r2;
        a_.eval(s, r, r1, r2);
        const double cs = std::cos(s), sn = std::sin(s);
        p = {r * cs, r * sn};
        d1 = {r1 * cs - r * sn, r1 * sn + r * cs};
        d2 = {r2 * cs - 2.0 * r1 * sn - r * cs, r2 * sn + 2.0 * r1 * cs - r * sn};
        return;
    }
    case Kind::Points: {
        a_.eval(s, p.x, d1.x, d2.x);
        b_.eval(s, p.y, d1.y, d2.y);
        return;
    }
    case Kind::Ellipse: {
        const double cs = std::cos(s), sn = std::sin(s);
        p = {ecx_ + ea_ * cs, eb_ * sn};
        d1 = {-ea_ * sn, eb_ * cs};
        d2 = {-ea_ * cs, -eb_ * sn};
        return;
    }
    case Kind::Polygon: {
        double w = s - period_ * std::floor(s / period_);
        size_t lo = 0, hi = prims_.size() - 1;      // last piece whose s0 <= w
        while (lo < hi) {
            const size_t mid = (lo + hi + 1) / 2;
            if (prims_[mid].s0 <= w) lo = mid; else hi = mid - 1;
        }
        const Prim& q = prims_[lo];
        const double u = w - q.s0;
        p = primPoint(q, u);
        if (!q.arc) {
            d1 = q.dir;
            d2 = {0.0, 0.0};
        } else {
            const double ang = q.a0 + q.sgn * u / q.r;
            d1 = q.sgn * Vec2{-std::sin(ang), std::cos(ang)};
            d2 = (-1.0 / q.r) * Vec2{std::cos(ang), std::sin(ang)};
        }
        return;
    }
    }
}

Vec2 ProfileCurve::primPoint(const Prim& q, double u) const {
    if (!q.arc) return q.a + u * q.dir;
    const double ang = q.a0 + q.sgn * u / q.r;
    return q.c + q.r * Vec2{std::cos(ang), std::sin(ang)};
}

void ProfileCurve::polarEval(double phi, double& r, double& dr, double& d2r) const {
    switch (kind_) {
    case Kind::Circle:
        r = radius_;
        dr = d2r = 0.0;
        return;
    case Kind::Polar:
        a_.eval(phi, r, dr, d2r);
        return;
    case Kind::Ellipse: {
        const double cs = std::cos(phi), sn = std::sin(phi);
        if (efocus_) {
            // r = p / (1 - e cos phi), p = b^2 / a, e = sqrt(a^2 - b^2) / a
            const double e = ecx_ / ea_, p = eb_ * eb_ / ea_;
            const double D = 1.0 - e * cs;
            r = p / D;
            dr = -p * e * sn / (D * D);
            d2r = -p * e * cs / (D * D) + 2.0 * p * e * e * sn * sn / (D * D * D);
        } else {
            // r = ab / sqrt(N), N = b^2 cos^2 + a^2 sin^2
            const double N = eb_ * eb_ * cs * cs + ea_ * ea_ * sn * sn;
            const double Np = (ea_ * ea_ - eb_ * eb_) * std::sin(2.0 * phi);
            const double Npp = 2.0 * (ea_ * ea_ - eb_ * eb_) * std::cos(2.0 * phi);
            const double ab = ea_ * eb_;
            r = ab * std::pow(N, -0.5);
            dr = -0.5 * ab * std::pow(N, -1.5) * Np;
            d2r = 0.75 * ab * std::pow(N, -2.5) * Np * Np - 0.5 * ab * std::pow(N, -1.5) * Npp;
        }
        return;
    }
    case Kind::Points:
    case Kind::Polygon:
        r = dr = d2r = 0.0;   // not star-shaped; callers check starShaped()
        return;
    }
}

ProfileCurve::Foot ProfileCurve::nearest(Vec2 u) const {
    Foot f;
    if (kind_ == Kind::Circle) {
        const double rr = vlen(u);
        double s = rr > 1e-300 ? std::atan2(u.y, u.x) : 0.0;
        if (s < 0.0) s += period_;
        const double cs = std::cos(s), sn = std::sin(s);
        f.s = s;
        f.point = {radius_ * cs, radius_ * sn};
        f.tangent = {-radius_ * sn, radius_ * cs};
        f.normal = {cs, sn};
        f.signedDist = rr - radius_;
        f.curvature = 1.0 / radius_;
        return f;
    }

    if (kind_ == Kind::Polygon) {
        // exact: the closest point of every straight piece and fillet, best wins
        double bestD2 = 1e300;
        size_t bi = 0;
        double bu = 0.0;
        // Exact distance of one piece (closest point and its parameter).
        auto exact = [&](const Prim& q, double& uu) {
            if (!q.arc) {
                uu = std::min(q.len, std::max(0.0, dot(u - q.a, q.dir)));
            } else {
                const Vec2 vv = u - q.c;
                double rel = q.sgn * (std::atan2(vv.y, vv.x) - q.a0);
                rel -= 2.0 * kPi * std::floor(rel / (2.0 * kPi));       // in [0, 2 pi)
                const double sweep = q.len / q.r;
                if (rel <= sweep) {
                    uu = rel * q.r;
                } else {   // off the end of the arc: whichever endpoint is nearer
                    const Vec2 e0 = primPoint(q, 0.0), e1 = primPoint(q, q.len);
                    uu = dot(u - e0, u - e0) <= dot(u - e1, u - e1) ? 0.0 : q.len;
                }
            }
            const Vec2 d = u - primPoint(q, uu);
            return dot(d, d);
        };
        // A cheap lower bound on a piece's distance: the distance to its bounding circle (a line's midpoint
        // and half length; an arc's centre and radius, which it lies on). A piece whose bound is above the
        // best so far cannot win, so the closest piece is found with a handful of exact evaluations
        // instead of one per piece, and the answer (including which piece wins a tie) is unchanged.
        auto lowerBound2 = [&](const Prim& q) {
            double lb;
            if (!q.arc) lb = vlen(u - (q.a + q.dir * (0.5 * q.len))) - 0.5 * q.len;
            else lb = std::fabs(vlen(u - q.c) - q.r);
            return lb > 0.0 ? lb * lb : 0.0;
        };
        thread_local std::vector<double> lbs;               // each piece's bound, computed once per query
        lbs.resize(prims_.size());
        {   // seed with the piece of smallest bound
            size_t seed = 0;
            double sb = 1e300;
            for (size_t i = 0; i < prims_.size(); ++i) {
                const double lb2 = lowerBound2(prims_[i]);
                lbs[i] = lb2;
                if (lb2 < sb) { sb = lb2; seed = i; }
            }
            double uu;
            bestD2 = exact(prims_[seed], uu);
            bi = seed;
            bu = uu;
        }
        for (size_t i = 0; i < prims_.size(); ++i) {
            const Prim& q = prims_[i];
            if (lbs[i] > bestD2 * (1.0 + 1e-9) + 1e-24) continue;
            double uu;
            const double dd = exact(q, uu);
            if (dd < bestD2 || (dd == bestD2 && i < bi)) { bestD2 = dd; bi = i; bu = uu; }
        }
        const Prim& q = prims_[bi];
        Vec2 p, d1, d2;
        eval(q.s0 + bu, p, d1, d2);
        f.s = q.s0 + bu;
        f.point = p;
        f.tangent = d1;                                   // unit speed
        f.normal = {d1.y, -d1.x};
        f.signedDist = dot(f.normal, u - p);
        f.curvature = q.arc ? q.sgn / q.r : 0.0;
        return f;
    }

    // coarse global scan over the precomputed outline
    const size_t K = sampleP_.size();
    size_t kb = 0;
    double best = 1e300;
    for (size_t k = 0; k < K; ++k) {
        const Vec2 d = u - sampleP_[k];
        const double d2 = dot(d, d);
        if (d2 < best) { best = d2; kb = k; }
    }
    const double ds = period_ / (double)K;
    const double sk = sampleS_[kb];

    // f(s) = c'(s) . (u - c(s)) vanishes at a foot; f > 0 before it, < 0 after
    auto F = [&](double s, double& fv, double& fp) {
        Vec2 p, d1, d2;
        eval(s, p, d1, d2);
        const Vec2 d = u - p;
        fv = dot(d1, d);
        fp = dot(d2, d) - dot(d1, d1);
    };

    double lo = sk - ds, hi = sk + ds;
    double flo, fhi, fp;
    F(lo, flo, fp);
    F(hi, fhi, fp);
    const bool bracketed = flo >= 0.0 && fhi <= 0.0;

    double s = sk;
    for (int it = 0; it < 60; ++it) {
        double fv;
        F(s, fv, fp);
        if (bracketed) {
            if (fv > 0.0) lo = s; else hi = s;
        }
        double sn;
        if (fp < 0.0) sn = s - fv / fp;                      // Newton toward the minimum
        else sn = s + (fv > 0.0 ? 0.5 : -0.5) * ds;          // curvature-dominated: nudge
        if (bracketed) {
            if (!(sn > lo && sn < hi)) sn = 0.5 * (lo + hi);
        } else {
            sn = std::min(sk + ds, std::max(sk - ds, sn));
        }
        const bool done = std::fabs(sn - s) < 1e-14 * period_;
        s = sn;
        if (done) break;
    }

    Vec2 p, d1, d2;
    eval(s, p, d1, d2);
    f.s = s - period_ * std::floor(s / period_);
    f.point = p;
    f.tangent = d1;
    f.normal = normalize(Vec2{d1.y, -d1.x});
    f.signedDist = dot(f.normal, u - p);
    const double tl = vlen(d1);
    f.curvature = crossz(d1, d2) / (tl * tl * tl);
    return f;
}

std::vector<Vec2> ProfileCurve::outline(int n) const {
    std::vector<Vec2> out;
    if (kind_ == Kind::Polygon) {                  // piece by piece; fillets in <= 4 deg steps
        for (const Prim& q : prims_) {
            if (!q.arc) { out.push_back(q.a); continue; }
            const int m = std::max(2, (int)std::ceil(q.len / q.r / deg2rad(4.0)));
            for (int k = 0; k < m; ++k) out.push_back(primPoint(q, q.len * (double)k / (double)m));
        }
        return out;
    }
    out.reserve((size_t)n);
    for (int k = 0; k < n; ++k) {
        Vec2 p, d1, d2;
        eval(period_ * (double)k / (double)n, p, d1, d2);
        out.push_back(p);
    }
    return out;
}

} // namespace mech
