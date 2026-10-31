// mech — render.cpp: SVG drawings of a mechanism (M13)
#include "render.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "geom.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace mech {

namespace {

struct Rgb { double r, g, b; };

struct Theme {
    Rgb bg, ground, hub, bore, trace, spring, dial, dialMajor, hand, outline;
    Rgb palette[5];
};

Theme lightTheme() {
    return {{1.00, 1.00, 1.00}, {0.35, 0.38, 0.42}, {1.00, 1.00, 1.00}, {0.12, 0.13, 0.16},
            {0.20, 0.62, 0.40}, {0.86, 0.42, 0.20}, {0.55, 0.58, 0.62}, {0.30, 0.33, 0.38},
            {0.16, 0.17, 0.20}, {0.14, 0.15, 0.18},
            {{0.85, 0.55, 0.12}, {0.20, 0.50, 0.70}, {0.72, 0.28, 0.28}, {0.35, 0.62, 0.30}, {0.55, 0.42, 0.78}}};
}

Theme darkTheme() {   // the viewer's colours
    return {{0.055, 0.065, 0.080}, {0.60, 0.64, 0.68}, {1.00, 1.00, 1.00}, {0.05, 0.06, 0.08},
            {0.42, 0.85, 0.62}, {0.92, 0.55, 0.36}, {0.72, 0.75, 0.78}, {0.85, 0.88, 0.90},
            {0.96, 0.93, 0.84}, {0.02, 0.03, 0.04},
            {{0.91, 0.69, 0.29}, {0.44, 0.70, 0.81}, {0.79, 0.44, 0.44}, {0.62, 0.82, 0.54}, {0.70, 0.62, 0.86}}};
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.7g", v);
    return b;
}

std::string hex(Rgb c) {
    auto h = [](double v) { return (int)std::lround(std::min(1.0, std::max(0.0, v)) * 255.0); };
    char b[16];
    std::snprintf(b, sizeof b, "#%02x%02x%02x", h(c.r), h(c.g), h(c.b));
    return b;
}

std::string xml(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '&') o += "&amp;";
        else if (c == '<') o += "&lt;";
        else if (c == '"') o += "&quot;";
        else o += c;
    }
    return o;
}

struct View {
    double minx, miny, maxx, maxy;
};

// How far a slide's guide travels along its rail over everything drawn, so the rail can be shown
// where the follower goes.
struct RailSpan {
    Vec2 pos, dir;
    double t0 = 1e30, t1 = -1e30;
};

// Everything that depends on the pose, for one frame.
class Drawer {
public:
    Drawer(const Model& m, const Theme& th, double pxWorld, double extent, std::vector<RailSpan> rails)
        : m_(m), th_(th), px_(pxWorld), extent_(extent), rails_(std::move(rails)) {}

    std::string line(Vec2 a, Vec2 b, Rgb c, double wPx, const char* cls = "", const std::string& extra = "",
                     double opacity = 1.0, bool round = false) const {
        std::string s = "<line";
        if (*cls) s += std::string(" class=\"") + cls + "\"";
        s += extra + " x1=\"" + num(a.x) + "\" y1=\"" + num(a.y) + "\" x2=\"" + num(b.x) + "\" y2=\"" + num(b.y) +
             "\" stroke=\"" + hex(c) + "\" stroke-width=\"" + num(wPx * px_) + "\"";
        if (opacity < 1.0) s += " stroke-opacity=\"" + num(opacity) + "\"";
        if (round) s += " stroke-linecap=\"round\"";
        return s + "/>\n";
    }

    std::string circle(Vec2 c, double rWorld, const char* fill, Rgb stroke, double wPx, const char* cls = "",
                       const std::string& extra = "", double strokeOpacity = 1.0) const {
        std::string s = "<circle";
        if (*cls) s += std::string(" class=\"") + cls + "\"";
        s += extra + " cx=\"" + num(c.x) + "\" cy=\"" + num(c.y) + "\" r=\"" + num(rWorld) + "\" fill=\"" + fill + "\"";
        if (wPx > 0.0) {
            s += " stroke=\"" + hex(stroke) + "\" stroke-width=\"" + num(wPx * px_) + "\"";
            if (strokeOpacity < 1.0) s += " stroke-opacity=\"" + num(strokeOpacity) + "\"";
        }
        return s + "/>\n";
    }

    std::string disc(Vec2 c, double rPx, Rgb fill, const char* cls = "", const std::string& extra = "") const {
        std::string s = "<circle";
        if (*cls) s += std::string(" class=\"") + cls + "\"";
        return s + extra + " cx=\"" + num(c.x) + "\" cy=\"" + num(c.y) + "\" r=\"" + num(rPx * px_) + "\" fill=\"" + hex(fill) + "\"/>\n";
    }

    std::string polygon(const std::vector<Vec2>& pts, const char* fill, double fillOpacity, Rgb stroke, double wPx,
                        const char* cls, const std::string& extra) const {
        std::string s = std::string("<polygon class=\"") + cls + "\"" + extra + " points=\"";
        for (size_t i = 0; i < pts.size(); ++i) s += (i ? " " : "") + num(pts[i].x) + "," + num(pts[i].y);
        s += "\" fill=\"" + std::string(fill) + "\"";
        if (fillOpacity < 1.0 && std::string(fill) != "none") s += " fill-opacity=\"" + num(fillOpacity) + "\"";
        if (wPx > 0.0) s += " stroke=\"" + hex(stroke) + "\" stroke-width=\"" + num(wPx * px_) + "\" stroke-linejoin=\"round\"";
        return s + "/>\n";
    }

    std::string polyline(const std::vector<Vec2>& pts, Rgb stroke, double wPx, double opacity, const char* cls,
                         const std::string& extra = "") const {
        std::string s = std::string("<polyline class=\"") + cls + "\"" + extra + " points=\"";
        for (size_t i = 0; i < pts.size(); ++i) s += (i ? " " : "") + num(pts[i].x) + "," + num(pts[i].y);
        s += "\" fill=\"none\" stroke=\"" + hex(stroke) + "\" stroke-width=\"" + num(wPx * px_) + "\" stroke-linejoin=\"round\"";
        if (opacity < 1.0) s += " stroke-opacity=\"" + num(opacity) + "\"";
        return s + "/>\n";
    }

    // hatched fixed segment: a capsule with tick marks on one side
    std::string hatched(Vec2 a, Vec2 b, double halfSpan, const char* cls) const {
        const Vec2 d = b - a;
        const double L = vlen(d);
        if (L < 1e-9) return "";
        const Vec2 u = (1.0 / L) * d, n = perp(u);
        const Vec2 aa = a - u * (halfSpan - 0.5 * L), bb = b + u * (halfSpan - 0.5 * L);
        std::string s = line(aa, bb, th_.ground, 3.0, cls, "", 1.0, true);
        const double step = 12.0 * px_;
        const Vec2 tick = normalize(n - u) * (9.0 * px_);
        std::string ticks = "<g class=\"ticks\">\n";
        for (double t = 0; t <= vlen(bb - aa); t += step) {
            const Vec2 p = aa + u * t;
            ticks += line(p, p + tick, th_.ground, 1.5);
        }
        return s + ticks + "</g>\n";
    }

    std::string spring(Vec2 a, Vec2 b) const {
        const Vec2 d = b - a;
        const double L = vlen(d);
        if (L < 1e-9) return "";
        const Vec2 u = (1.0 / L) * d, n = perp(u);
        const double amp = std::min(0.045 * L + 4.0 * px_, 0.09 * L + 12.0 * px_);
        const double lead = std::min(0.12 * L, 0.35);
        const int coils = std::max(6, (int)(L / (6.0 * amp + 1e-9)));
        std::vector<Vec2> pts;
        pts.push_back(a);
        pts.push_back(a + u * lead);
        for (int i = 0; i < coils; ++i) {
            const double s1 = lead + (L - 2.0 * lead) * (i + 0.5) / coils;
            pts.push_back(a + u * s1 + n * ((i % 2 == 0) ? amp : -amp));
        }
        pts.push_back(b - u * lead);
        pts.push_back(b);
        return polyline(pts, th_.spring, 1.8, 0.95, "spring");
    }

    // the whole pose
    std::string pose(const std::vector<double>& q) const {
        std::ostringstream o;
        // gears
        for (size_t gi = 0; gi < m_.gears.size(); ++gi) {
            const Gear& g = m_.gears[gi];
            const Body& b = m_.bodies[(size_t)g.body];
            const double th = b.qOffset >= 0 ? q[(size_t)b.qOffset] : 0.0;
            const Vec2 c = m_.worldAnchor(g.body, g.hostAnchor, q);
            const Rgb col = th_.palette[(size_t)(g.body > 0 ? g.body - 1 : 0) % 5];
            // a ring gear: the larger of the two in an internal mesh (the planet inside it is an ordinary gear)
            bool internal = false;
            for (const MeshJoint& mm : m_.meshes) {
                if (mm.sgn >= 0 || (mm.gearA != (int)gi && mm.gearB != (int)gi)) continue;
                const Gear& other = m_.gears[(size_t)(mm.gearA == (int)gi ? mm.gearB : mm.gearA)];
                if (g.radius > other.radius) internal = true;
            }
            const double tooth = g.module > 0.0 ? g.module : std::max(0.08 * g.radius, 3.0 * px_);
            const std::string tag = " data-name=\"" + xml(g.name) + "\"";
            o << "<g class=\"gear\"" << tag << ">\n";
            if (internal) {
                const double r0 = g.radius + 1.1 * tooth, r1 = g.radius + 3.2 * tooth;
                o << circle(c, 0.5 * (r0 + r1), "none", col, (r1 - r0) / px_, "", "", 1.0);
                const double q0 = g.radius - 1.05 * tooth, q1 = g.radius + 1.1 * tooth;
                o << circle(c, 0.5 * (q0 + q1), "none", col, (q1 - q0) / px_, "", "", 0.28);
            }
            o << circle(c, g.radius, "none", col, 2.0, "pitch", "", internal ? 0.6 : 0.45);
            const int N = g.teeth > 0 ? g.teeth : 16;
            for (int i = 0; i < N; ++i) {
                const double ang = th + 2.0 * kPi * (double)i / N;
                const Vec2 u{std::cos(ang), std::sin(ang)};
                const double r0 = internal ? g.radius + 1.1 * tooth : 0.88 * g.radius;
                const double r1 = internal ? g.radius - 1.05 * tooth : 1.10 * g.radius;
                o << line(c + u * r0, c + u * r1, col, 1.5, "tooth", "", 0.9);
            }
            o << "</g>\n";
        }
        // profiles (cams, ratchet wheels, ...): the curve rides its host
        for (const Profile& pr : m_.profiles) {
            const Body& hb = m_.bodies[(size_t)pr.body];
            const double th = hb.qOffset >= 0 ? q[(size_t)hb.qOffset] : 0.0;
            const Vec2 tr = hb.qOffset >= 0 ? Vec2{q[(size_t)hb.qOffset + 1], q[(size_t)hb.qOffset + 2]} : Vec2{0.0, 0.0};
            const Vec2 origin = hb.anchors[(size_t)pr.hostAnchor].local;
            const Rgb col = th_.palette[(size_t)(pr.body > 0 ? pr.body - 1 : 0) % 5];
            std::vector<Vec2> pts;
            for (const Vec2& p : pr.curve->outline(240)) pts.push_back(rotate(origin + p, th) + tr);
            o << polygon(pts, hex(col).c_str(), 0.16, col, 2.5, "profile", " data-name=\"" + xml(pr.name) + "\"");
        }
        // moving bodies: rods as capsules, shafts as hubs with an index line
        for (size_t bi = 1; bi < m_.bodies.size(); ++bi) {
            const Body& b = m_.bodies[bi];
            const Rgb col = th_.palette[(bi - 1) % 5];
            const std::string tag = " data-body=\"" + xml(b.name) + "\"";
            if (b.anchors.size() >= 2) {
                const Vec2 pa = m_.worldAnchor((int)bi, 0, q), pb = m_.worldAnchor((int)bi, 1, q);
                o << "<g class=\"rod\"" << tag << ">\n";
                o << line(pa, pb, th_.outline, 9.0, "", "", 1.0, true);
                o << line(pa, pb, col, 7.0, "bar", "", 1.0, true);
                o << "</g>\n";
            } else if (b.anchors.size() == 1) {
                const Vec2 c = m_.worldAnchor((int)bi, 0, q);
                const double th = q[(size_t)b.qOffset];
                o << "<g class=\"shaft\"" << tag << ">\n";
                o << circle(c, 6.5 * px_, hex(col).c_str(), th_.outline, 1.2, "hub");
                o << disc(c, 2.4, th_.bore);
                o << line(c, c + Vec2{std::cos(th), std::sin(th)} * (11.0 * px_), th_.hub, 2.0, "index");
                o << "</g>\n";
            }
        }
        // springs
        for (const SpringJoint& sp : m_.springs)
            o << spring(m_.worldAnchor(sp.bodyA, sp.anchA, q), m_.worldAnchor(sp.bodyB, sp.anchB, q));
        // touches: the roller (if any) and the contact point on the curve
        for (const TouchJoint& tj : m_.touches) {
            const Vec2 tip = m_.worldAnchor(tj.body, tj.anchor, q);
            const ContactState cs = m_.touchState(tj, q);
            if (tj.roller > 0.0) o << circle(tip, tj.roller, "none", th_.outline, 2.0, "roller");
            o << disc(cs.foot, 3.2, th_.spring, "foot");
        }
        // pins
        for (const PinJoint& p : m_.pins) {
            const Vec2 pos = m_.worldAnchor(p.bodyA, p.anchA, q);
            o << "<g class=\"pin\">\n" << circle(pos, 5.0 * px_, hex(th_.hub).c_str(), th_.outline, 1.2, "hub")
              << disc(pos, 2.2, th_.bore) << "</g>\n";
        }
        // hands on top
        for (const Hand& h : m_.hands) {
            const Body& b = m_.bodies[(size_t)h.body];
            const double th = b.qOffset >= 0 ? q[(size_t)b.qOffset] : 0.0;
            const Vec2 mount = m_.worldAnchor(h.body, h.mountAnchor, q);
            const double L = h.length, w1 = 4.0 * px_, w2 = 1.6 * px_;
            auto local = [&](double lx, double ly) { return rotate(Vec2{lx, ly}, th) + mount; };
            const std::vector<Vec2> pts = {local(-0.22 * L, 0), local(-0.10 * L, w1), local(0.72 * L, w2), local(L, 0),
                                           local(0.72 * L, -w2), local(-0.10 * L, -w1)};
            o << polygon(pts, hex(th_.hand).c_str(), 0.96, th_.outline, 0.8, "hand", " data-name=\"" + xml(h.name) + "\"");
            o << disc(mount, 5.5, th_.hub) << disc(mount, 2.3, th_.bore);
        }
        return o.str();
    }

    // what does not move: dials and the ground
    std::string fixed() const {
        std::ostringstream o;
        for (const Dial& d : m_.dials) {
            if (d.radius <= 0.0) continue;
            const double R = d.radius;
            o << "<g class=\"dial\">\n";
            o << circle(d.center, R - 1.25 * px_, "none", th_.dial, 2.5, "ring", "", 0.9);
            for (int i = 0; i < d.minor; ++i) {
                const double ang = -0.5 * kPi + 2.0 * kPi * (double)i / d.minor;
                const Vec2 u{std::cos(ang), std::sin(ang)};
                o << line(d.center + u * (R - 8.0 * px_), d.center + u * (R - 2.5 * px_), th_.dial, 1.2);
            }
            for (int i = 0; i < d.major; ++i) {
                const double ang = -0.5 * kPi + 2.0 * kPi * (double)i / d.major;
                const Vec2 u{std::cos(ang), std::sin(ang)};
                o << line(d.center + u * (R - 16.0 * px_), d.center + u * (R - 2.5 * px_), th_.dialMajor, 2.2);
            }
            o << "</g>\n";
        }
        const Body& g = m_.bodies[0];
        o << "<g class=\"ground\">\n";
        // The frame: one hatched bar if the (rail-free) anchors lie on a line, a small mark under
        // each one otherwise — a bar through scattered points means nothing.
        std::vector<Vec2> pts;
        for (const Anchor& a : g.anchors) if (!a.hasDir) pts.push_back(a.local);
        bool bar = false;
        Vec2 pmin{1e30, 1e30}, pmax{-1e30, -1e30};
        if (pts.size() >= 2) {
            Vec2 lo = pts[0], hi = pts[0];
            for (const Vec2& p : pts) {
                if (p.x < lo.x || (p.x == lo.x && p.y < lo.y)) lo = p;
                if (p.x > hi.x || (p.x == hi.x && p.y > hi.y)) hi = p;
            }
            if (vlen(hi - lo) < 1e-9) {           // a vertical column: order by y instead
                for (const Vec2& p : pts) { if (p.y < lo.y) lo = p; if (p.y > hi.y) hi = p; }
            }
            const double L = vlen(hi - lo);
            bar = L > 1e-9;
            for (const Vec2& p : pts)
                if (bar && std::fabs(crossz(hi - lo, p - lo)) / L > 0.01 * L) bar = false;
            pmin = lo; pmax = hi;
        }
        if (bar) {
            o << hatched(pmin, pmax, 0.30 + 0.5 * vlen(pmax - pmin), "base");
        } else {
            for (const Vec2& p : pts) o << hatched(p - Vec2{12.0 * px_, 0.0}, p + Vec2{12.0 * px_, 0.0}, 12.0 * px_, "mount");
        }
        for (const Anchor& a : g.anchors) {
            if (!a.hasDir) continue;
            double t0 = -0.06 * extent_, t1 = 0.06 * extent_;          // no slide uses it: a short stub
            for (const RailSpan& r : rails_) {
                if (vlen(r.pos - a.local) > 1e-9 || vlen(r.dir - a.dirLocal) > 1e-9 || r.t1 < r.t0) continue;
                const double margin = 0.03 * extent_;
                t0 = std::min(r.t0 - margin, -0.0);
                t1 = std::max(r.t1 + margin, 0.0);
                if (t1 - t0 < 0.12 * extent_) { const double c = 0.5 * (t0 + t1); t0 = c - 0.06 * extent_; t1 = c + 0.06 * extent_; }
            }
            // a rail is drawn from the anchor's own line: positions t along the direction through the anchor
            const Vec2 a0 = a.local + a.dirLocal * t0, a1 = a.local + a.dirLocal * t1;
            o << hatched(a0, a1, 0.5 * (t1 - t0), "rail");
        }
        o << "</g>\n";
        return o.str();
    }

private:
    const Model& m_;
    const Theme& th_;
    double px_, extent_;
    std::vector<RailSpan> rails_;
};

} // namespace

std::vector<std::vector<double>> samplePoses(const Model& m, const std::vector<double>& times, std::string& error) {
    error.clear();
    std::vector<std::vector<double>> out;
    std::vector<double> q;
    m.assemble(q);
    ConstraintSystem sys(m);
    int iters = 0;
    double res = 0.0;
    const bool ok0 = sys.solve(q, 0.0, iters, &res);
    const std::vector<std::string> bad = m.validateAssembly(q);
    if (!bad.empty()) {
        error = bad[0];
        return {};
    }
    // a drawing of a mechanism that does not close would be a picture of nothing
    if (!ok0 || res > 1e-6) {
        char b[160];
        std::snprintf(b, sizeof b, "the mechanism does not assemble (largest constraint residual %.3g); nothing to draw", res);
        error = b;
        return {};
    }
    if (!m.world.dynamic) {
        const double dt = 1.0 / 60.0;
        double cur = 0.0;
        for (double t : times) {
            while (cur + dt < t - 1e-12) { cur += dt; sys.solve(q, cur, iters); }
            cur = std::max(cur, t);
            double r = 0.0;
            const bool ok = sys.solve(q, t, iters, &r);
            if (!ok || r > 1e-6) {
                char b[160];
                std::snprintf(b, sizeof b, "the mechanism cannot be posed at t = %g s (largest constraint residual %.3g)", t, r);
                error = b;
                return {};
            }
            out.push_back(q);
        }
        return out;
    }
    DynamicsSystem dyn(m, sys);
    std::vector<double> qd((size_t)m.nq(), 0.0);
    dyn.projectVelocities(q, qd);
    const double h = m.world.step;
    long k = 0;
    for (double t : times) {
        const long target = (long)std::llround(t / h);
        while (k < target) { dyn.step(q, qd, (double)k * h); ++k; }
        out.push_back(q);
    }
    return out;
}

RenderResult renderSvg(const Model& m, const RenderOptions& opt) {
    RenderResult res;
    const int frames = std::max(1, opt.frames);
    int traceBody = -1, traceAnchor = -1;
    if (!opt.trace.empty()) {
        const size_t dot = opt.trace.rfind('.');
        if (dot != std::string::npos) {
            traceBody = m.bodyIndex(opt.trace.substr(0, dot));
            traceAnchor = traceBody >= 0 ? m.anchorIndex(traceBody, opt.trace.substr(dot + 1)) : -1;
        }
        if (traceBody < 0 || traceAnchor < 0) {
            res.ok = false;
            res.error = "--trace expects an existing body.anchor, got '" + opt.trace + "'";
            return res;
        }
    }

    // the poses to draw, then (for a trace) a denser walk over the same span
    std::vector<double> times;
    for (int k = 0; k < frames; ++k)
        times.push_back(opt.at + (frames > 1 ? opt.span * (double)k / (double)frames : 0.0));
    std::vector<double> traceTimes;
    if (traceBody >= 0) {
        const int n = 240;
        for (int k = 0; k <= n; ++k) traceTimes.push_back(opt.at + opt.span * (double)k / (double)n);
    }
    std::vector<double> all = times;
    all.insert(all.end(), traceTimes.begin(), traceTimes.end());
    std::vector<size_t> order(all.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return all[a] < all[b]; });
    std::vector<double> sorted;
    for (size_t i : order) sorted.push_back(all[i]);
    std::string err;
    const std::vector<std::vector<double>> sampled = samplePoses(m, sorted, err);
    if (sampled.empty() && !sorted.empty()) {
        res.ok = false;
        res.error = err.empty() ? "the mechanism could not be posed" : err;
        return res;
    }
    std::vector<std::vector<double>> poses(all.size());
    for (size_t i = 0; i < order.size(); ++i) poses[order[i]] = sampled[i];

    std::vector<Vec2> trail;
    for (size_t k = 0; k < traceTimes.size(); ++k)
        trail.push_back(m.worldAnchor(traceBody, traceAnchor, poses[times.size() + k]));

    // the view: everything that can appear, in every pose drawn
    double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
    auto grow = [&](Vec2 p) {
        minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
    };
    auto growCircle = [&](Vec2 c, double r) { grow(c + Vec2{r, r}); grow(c - Vec2{r, r}); };
    for (size_t k = 0; k < times.size(); ++k) {
        const std::vector<double>& q = poses[k];
        for (size_t bi = 0; bi < m.bodies.size(); ++bi)
            for (size_t ai = 0; ai < m.bodies[bi].anchors.size(); ++ai) grow(m.worldAnchor((int)bi, (int)ai, q));
        for (const Gear& g : m.gears) growCircle(m.worldAnchor(g.body, g.hostAnchor, q), 1.1 * g.radius);
        for (const Profile& pr : m.profiles) growCircle(m.worldAnchor(pr.body, pr.hostAnchor, q), pr.curve->maxRadius());
        for (const Hand& h : m.hands) growCircle(m.worldAnchor(h.body, h.mountAnchor, q), h.length);
        for (const TouchJoint& tj : m.touches) growCircle(m.worldAnchor(tj.body, tj.anchor, q), tj.roller);
    }
    for (const Dial& d : m.dials) if (d.radius > 0.0) growCircle(d.center, d.radius);
    for (const Vec2& p : trail) grow(p);
    double ex = maxx - minx, ey = maxy - miny;
    const double ext = std::max({ex, ey, 0.05});
    const double pad = 0.07 * ext;
    minx -= pad; maxx += pad; miny -= pad; maxy += pad;
    ex = std::max(maxx - minx, 0.05 + 2 * pad);
    ey = std::max(maxy - miny, 0.05 + 2 * pad);
    const double cx = 0.5 * (minx + maxx), cy = 0.5 * (miny + maxy);
    const double scale = opt.width / ex;                // pixels per metre
    const double Wpx = opt.width, Hpx = ey * scale;
    const double pxWorld = 1.0 / scale;

    std::vector<RailSpan> rails;
    for (const SlideJoint& sj : m.slides) {
        RailSpan r;
        r.pos = sj.railPos;
        r.dir = sj.railDir;
        for (size_t k = 0; k < times.size(); ++k) {
            const double t = dot(m.worldAnchor(sj.guideBody, sj.guideAnchor, poses[k]) - sj.railPos, sj.railDir);
            r.t0 = std::min(r.t0, t);
            r.t1 = std::max(r.t1, t);
        }
        rails.push_back(r);
    }

    const Theme th = opt.dark ? darkTheme() : lightTheme();
    const Drawer dr(m, th, pxWorld, std::max(ex, ey), rails);

    std::ostringstream o;
    o << "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" width=\"" << num(Wpx)
      << "\" height=\"" << num(Hpx) << "\" viewBox=\"0 0 " << num(Wpx) << " " << num(Hpx) << "\" data-mechanism=\""
      << xml(m.name) << "\" data-scale=\"" << num(scale) << "\">\n";
    o << "<title>" << xml(m.name) << "</title>\n";
    o << "<rect width=\"" << num(Wpx) << "\" height=\"" << num(Hpx) << "\" fill=\"" << hex(th.bg) << "\"/>\n";
    // world coordinates (metres, y up) -> pixels
    o << "<g id=\"world\" transform=\"matrix(" << num(scale) << " 0 0 " << num(-scale) << " " << num(Wpx / 2 - cx * scale)
      << " " << num(Hpx / 2 + cy * scale) << ")\">\n";
    o << dr.fixed();
    if (!trail.empty()) o << dr.polyline(trail, th.trace, 2.0, 0.75, "trace", " data-anchor=\"" + xml(opt.trace) + "\"");
    if (frames == 1) {
        o << "<g class=\"pose\" data-t=\"" << num(times[0]) << "\">\n" << dr.pose(poses[0]) << "</g>\n";
    } else {
        o << "<defs>\n";
        for (int k = 0; k < frames; ++k)
            o << "<g id=\"f" << k << "\" class=\"pose\" data-t=\"" << num(times[(size_t)k]) << "\">\n"
              << dr.pose(poses[(size_t)k]) << "</g>\n";
        o << "</defs>\n";
        const double dur = std::max(0.2, (opt.span > 0.0 ? opt.span : 1.0) / std::max(1e-9, opt.speed));
        std::string values;
        for (int k = 0; k < frames; ++k) values += (k ? ";" : "") + std::string("#f") + std::to_string(k);
        o << "<use href=\"#f0\" xlink:href=\"#f0\">\n<animate attributeName=\"href\" values=\"" << values
          << "\" dur=\"" << num(dur) << "s\" calcMode=\"discrete\" repeatCount=\"indefinite\"/>\n"
          << "<animate attributeName=\"xlink:href\" values=\"" << values << "\" dur=\"" << num(dur)
          << "s\" calcMode=\"discrete\" repeatCount=\"indefinite\"/>\n</use>\n";
    }
    o << "</g>\n</svg>\n";
    res.svg = o.str();
    res.frames = frames;
    return res;
}

} // namespace mech
