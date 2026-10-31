// mech — scene.cpp: what a pose looks like, as drawing items (M15)
#include "scene.hpp"
#include "kinematics.hpp"
#include "geom.hpp"

#include <algorithm>
#include <cmath>

namespace mech {

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

namespace {

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

void groupOpen(std::vector<SceneItem>& out, const char* cls, const std::string& attrs = "") {
    SceneItem it;
    it.kind = SceneItem::Kind::GroupOpen;
    it.cls = cls;
    it.attrs = attrs;
    out.push_back(it);
}

void groupClose(std::vector<SceneItem>& out) {
    SceneItem it;
    it.kind = SceneItem::Kind::GroupClose;
    out.push_back(it);
}

void line(std::vector<SceneItem>& out, Vec2 a, Vec2 b, Rgb c, double wPx, const char* cls = "",
          const std::string& extra = "", double opacity = 1.0, bool round = false) {
    SceneItem it;
    it.kind = SceneItem::Kind::Line;
    it.cls = cls;
    it.attrs = extra;
    it.pts = {a, b};
    it.hasStroke = true;
    it.stroke = c;
    it.strokePx = wPx;
    it.strokeOpacity = opacity;
    it.round = round;
    out.push_back(it);
}

// A circle of `r` world units: an outline (wPx > 0), a fill, or both.
void circle(std::vector<SceneItem>& out, Vec2 c, double r, bool hasFill, Rgb fill, Rgb stroke, double wPx,
            const char* cls = "", const std::string& extra = "", double strokeOpacity = 1.0) {
    SceneItem it;
    it.kind = SceneItem::Kind::Circle;
    it.cls = cls;
    it.attrs = extra;
    it.pts = {c};
    it.r = r;
    it.hasFill = hasFill;
    it.fill = fill;
    it.hasStroke = wPx > 0.0;
    it.stroke = stroke;
    it.strokePx = wPx;
    it.strokeOpacity = strokeOpacity;
    out.push_back(it);
}

void polygon(std::vector<SceneItem>& out, const std::vector<Vec2>& pts, Rgb fill, double fillOpacity, Rgb stroke,
             double wPx, const char* cls, const std::string& extra) {
    SceneItem it;
    it.kind = SceneItem::Kind::Polygon;
    it.cls = cls;
    it.attrs = extra;
    it.pts = pts;
    it.hasFill = true;
    it.fill = fill;
    it.fillOpacity = fillOpacity;
    it.hasStroke = wPx > 0.0;
    it.stroke = stroke;
    it.strokePx = wPx;
    out.push_back(it);
}

void polyline(std::vector<SceneItem>& out, const std::vector<Vec2>& pts, Rgb stroke, double wPx, double opacity,
              const char* cls, const std::string& extra = "") {
    SceneItem it;
    it.kind = SceneItem::Kind::Polyline;
    it.cls = cls;
    it.attrs = extra;
    it.pts = pts;
    it.hasStroke = true;
    it.stroke = stroke;
    it.strokePx = wPx;
    it.strokeOpacity = opacity;
    out.push_back(it);
}

} // namespace

// hatched fixed segment: a capsule with tick marks on one side
static void hatched(const Theme& th, double px, std::vector<SceneItem>& out, Vec2 a, Vec2 b, double halfSpan,
                    const char* cls) {
    const Vec2 d = b - a;
    const double L = vlen(d);
    if (L < 1e-9) return;
    const Vec2 u = (1.0 / L) * d, n = perp(u);
    const Vec2 aa = a - u * (halfSpan - 0.5 * L), bb = b + u * (halfSpan - 0.5 * L);
    line(out, aa, bb, th.ground, 3.0, cls, "", 1.0, true);
    const double step = 12.0 * px;
    const Vec2 tick = normalize(n - u) * (9.0 * px);
    groupOpen(out, "ticks");
    for (double t = 0; t <= vlen(bb - aa); t += step) {
        const Vec2 p = aa + u * t;
        line(out, p, p + tick, th.ground, 1.5);
    }
    groupClose(out);
}

static void spring(const Theme& th, double px, std::vector<SceneItem>& out, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const double L = vlen(d);
    if (L < 1e-9) return;
    const Vec2 u = (1.0 / L) * d, n = perp(u);
    const double amp = std::min(0.045 * L + 4.0 * px, 0.09 * L + 12.0 * px);
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
    polyline(out, pts, th.spring, 1.8, 0.95, "spring");
}

void SceneBuilder::pose(const std::vector<double>& q, std::vector<SceneItem>& out) const {
    const Rgb none{0, 0, 0};
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
        groupOpen(out, "gear", " data-name=\"" + xml(g.name) + "\"");
        if (internal) {
            const double r0 = g.radius + 1.1 * tooth, r1 = g.radius + 3.2 * tooth;
            circle(out, c, 0.5 * (r0 + r1), false, none, col, (r1 - r0) / px_, "", "", 1.0);
            const double q0 = g.radius - 1.05 * tooth, q1 = g.radius + 1.1 * tooth;
            circle(out, c, 0.5 * (q0 + q1), false, none, col, (q1 - q0) / px_, "", "", 0.28);
        }
        circle(out, c, g.radius, false, none, col, 2.0, "pitch", "", internal ? 0.6 : 0.45);
        const int N = g.teeth > 0 ? g.teeth : 16;
        for (int i = 0; i < N; ++i) {
            const double ang = th + 2.0 * kPi * (double)i / N;
            const Vec2 u{std::cos(ang), std::sin(ang)};
            const double r0 = internal ? g.radius + 1.1 * tooth : 0.88 * g.radius;
            const double r1 = internal ? g.radius - 1.05 * tooth : 1.10 * g.radius;
            line(out, c + u * r0, c + u * r1, col, 1.5, "tooth", "", 0.9);
        }
        groupClose(out);
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
        polygon(out, pts, col, 0.16, col, 2.5, "profile", " data-name=\"" + xml(pr.name) + "\"");
    }
    // moving bodies: rods as capsules, shafts as hubs with an index line
    for (size_t bi = 1; bi < m_.bodies.size(); ++bi) {
        const Body& b = m_.bodies[bi];
        const Rgb col = th_.palette[(bi - 1) % 5];
        const std::string tag = " data-body=\"" + xml(b.name) + "\"";
        if (b.anchors.size() >= 2) {
            const Vec2 pa = m_.worldAnchor((int)bi, 0, q), pb = m_.worldAnchor((int)bi, 1, q);
            groupOpen(out, "rod", tag);
            line(out, pa, pb, th_.outline, 9.0, "", "", 1.0, true);
            line(out, pa, pb, col, 7.0, "bar", "", 1.0, true);
            // a body with more anchors (a boom with a pivot, an attachment and a tip) is a chain through them
            for (size_t ai = 2; ai < b.anchors.size(); ++ai) {
                const Vec2 p0 = m_.worldAnchor((int)bi, (int)ai - 1, q), p1 = m_.worldAnchor((int)bi, (int)ai, q);
                line(out, p0, p1, th_.outline, 9.0, "", "", 1.0, true);
                line(out, p0, p1, col, 7.0, "link", "", 1.0, true);
            }
            groupClose(out);
        } else if (b.anchors.size() == 1) {
            const Vec2 c = m_.worldAnchor((int)bi, 0, q);
            const double th = q[(size_t)b.qOffset];
            groupOpen(out, "shaft", tag);
            circle(out, c, 6.5 * px_, true, col, th_.outline, 1.2, "hub");
            circle(out, c, 2.4 * px_, true, th_.bore, none, 0.0);
            line(out, c, c + Vec2{std::cos(th), std::sin(th)} * (11.0 * px_), th_.hub, 2.0, "index");
            groupClose(out);
        }
    }
    // M14: a rail that rides a moving body is drawn with it, as a line through its anchor that reaches
    // a little past the guide's travel (t0, t1 are measured from the rail anchor, over every frame)
    for (size_t k = 0; k < m_.slides.size() && k < moving_.size(); ++k) {
        const SlideJoint& sj = m_.slides[k];
        if (sj.railBody == 0 || moving_[k].t1 < moving_[k].t0) continue;
        const SlideGeom g = slideGeom(m_, sj, q);
        const double margin = 0.03 * extent_;
        // from the rail's own anchor (so the body it belongs to shows) to a little past the guide's farthest reach
        double t0 = std::min(moving_[k].t0 - margin, 0.0), t1 = std::max(moving_[k].t1 + margin, 0.0);
        if (t1 - t0 < 0.12 * extent_) { const double c = 0.5 * (t0 + t1); t0 = c - 0.06 * extent_; t1 = c + 0.06 * extent_; }
        const Rgb col = th_.palette[(size_t)(sj.railBody - 1) % 5];
        const std::string tag = " data-body=\"" + xml(m_.bodies[(size_t)sj.railBody].name) + "\" data-guide=\"" +
                                xml(m_.bodies[(size_t)sj.guideBody].name) + "\"";
        line(out, g.pB + g.u * t0, g.pB + g.u * t1, th_.outline, 7.0, "", "", 1.0, true);
        line(out, g.pB + g.u * t0, g.pB + g.u * t1, th_.hub, 4.5, sj.lockDir ? "slide" : "slot", tag, 1.0, true);
        line(out, g.pB + g.u * t0, g.pB + g.u * t1, col, 4.5, "", "", 0.45, true);
    }
    // springs
    for (const SpringJoint& sp : m_.springs)
        spring(th_, px_, out, m_.worldAnchor(sp.bodyA, sp.anchA, q), m_.worldAnchor(sp.bodyB, sp.anchB, q));
    // touches: the roller (if any) and the contact point on the curve
    for (const TouchJoint& tj : m_.touches) {
        const Vec2 tip = m_.worldAnchor(tj.body, tj.anchor, q);
        const ContactState cs = m_.touchState(tj, q);
        if (tj.roller > 0.0) circle(out, tip, tj.roller, false, none, th_.outline, 2.0, "roller");
        circle(out, cs.foot, 3.2 * px_, true, th_.spring, none, 0.0, "foot");
    }
    // pins
    for (const PinJoint& p : m_.pins) {
        const Vec2 pos = m_.worldAnchor(p.bodyA, p.anchA, q);
        groupOpen(out, "pin");
        circle(out, pos, 5.0 * px_, true, th_.hub, th_.outline, 1.2, "hub");
        circle(out, pos, 2.2 * px_, true, th_.bore, none, 0.0);
        groupClose(out);
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
        polygon(out, pts, th_.hand, 0.96, th_.outline, 0.8, "hand", " data-name=\"" + xml(h.name) + "\"");
        circle(out, mount, 5.5 * px_, true, th_.hub, none, 0.0);
        circle(out, mount, 2.3 * px_, true, th_.bore, none, 0.0);
    }
}

void SceneBuilder::fixed(std::vector<SceneItem>& out) const {
    const Rgb none{0, 0, 0};
    for (const Dial& d : m_.dials) {
        if (d.radius <= 0.0) continue;
        const double R = d.radius;
        groupOpen(out, "dial");
        circle(out, d.center, R - 1.25 * px_, false, none, th_.dial, 2.5, "ring", "", 0.9);
        for (int i = 0; i < d.minor; ++i) {
            const double ang = -0.5 * kPi + 2.0 * kPi * (double)i / d.minor;
            const Vec2 u{std::cos(ang), std::sin(ang)};
            line(out, d.center + u * (R - 8.0 * px_), d.center + u * (R - 2.5 * px_), th_.dial, 1.2);
        }
        for (int i = 0; i < d.major; ++i) {
            const double ang = -0.5 * kPi + 2.0 * kPi * (double)i / d.major;
            const Vec2 u{std::cos(ang), std::sin(ang)};
            line(out, d.center + u * (R - 16.0 * px_), d.center + u * (R - 2.5 * px_), th_.dialMajor, 2.2);
        }
        groupClose(out);
    }
    const Body& g = m_.bodies[0];
    groupOpen(out, "ground");
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
        hatched(th_, px_, out, pmin, pmax, 0.30 + 0.5 * vlen(pmax - pmin), "base");
    } else {
        for (const Vec2& p : pts) hatched(th_, px_, out, p - Vec2{12.0 * px_, 0.0}, p + Vec2{12.0 * px_, 0.0}, 12.0 * px_, "mount");
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
        hatched(th_, px_, out, a0, a1, 0.5 * (t1 - t0), "rail");
    }
    groupClose(out);
}

} // namespace mech
