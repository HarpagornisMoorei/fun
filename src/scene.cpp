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

std::vector<RailSpan> newRackSpans(const Model& m) {
    return std::vector<RailSpan>(m.racks.size());
}

void growRackSpans(const Model& m, const std::vector<double>& q, std::vector<RailSpan>& spans) {
    for (size_t k = 0; k < m.racks.size() && k < spans.size(); ++k) {
        const SlideGeom g = slideGeom(m, m.racks[k].geom, q);
        const double t = dot(g.u, g.pA - g.pB);
        spans[k].t0 = std::min(spans[k].t0, t);
        spans[k].t1 = std::max(spans[k].t1, t);
    }
}

namespace {

// One run of a belt's closed path: a straight line or an arc of a pulley.
struct BeltLeg {
    bool arc = false;
    Vec2 a, b;                   // line: end points
    Vec2 c;                      // arc: centre
    double r = 0.0, phi0 = 0.0, sweep = 0.0;   // arc: radius, start angle, signed sweep
    double len = 0.0;
    Vec2 at(double s) const {
        if (!arc) return len > 0.0 ? a + (b - a) * (s / len) : a;
        const double ang = phi0 + (sweep >= 0.0 ? 1.0 : -1.0) * s / r;
        return c + Vec2{std::cos(ang), std::sin(ang)} * r;
    }
};

// The path of a belt over its pulleys (tangent lines and wrapped arcs). Pulley i is wrapped counter-clockwise
// when w_i = +1: w_i is the belt's circulation (the sign of the centres' polygon) times the pulley's sign, so a
// crossed pulley is wrapped the other way and the tangent between unlike wraps is an inner (crossing) one.
// For the run from pulley 1 to pulley 2 with direction angle alpha, the belt leaves and meets the circles at
// c - w r perp(t); the run is straight when perp(t).d = w2 r2 - w1 r1, i.e. alpha = beta - asin(k / L).
bool beltPath(const Model& m, const Belt& b, const std::vector<double>& q, std::vector<BeltLeg>& legs) {
    const size_t n = b.gears.size();
    std::vector<Vec2> c(n);
    std::vector<double> r(n), w(n);
    double area = 0.0;
    for (size_t i = 0; i < n; ++i) {
        c[i] = m.gearCenter(b.gears[i], q);
        r[i] = m.gears[(size_t)b.gears[i]].radius;
    }
    for (size_t i = 0; i < n; ++i) area += crossz(c[i], c[(i + 1) % n]);
    const double circ = area >= 0.0 ? 1.0 : -1.0;
    for (size_t i = 0; i < n; ++i) w[i] = circ * b.sign[i];
    std::vector<double> alpha(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const Vec2 d = c[j] - c[i];
        const double L = vlen(d);
        const double k = w[j] * r[j] - w[i] * r[i];
        if (L <= std::fabs(k) + 1e-12) return false;          // pulleys nested: no belt between them
        alpha[i] = std::atan2(d.y, d.x) - std::asin(k / L);
    }
    legs.clear();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n, h = (i + n - 1) % n;
        const Vec2 t{std::cos(alpha[i]), std::sin(alpha[i])};
        BeltLeg run;
        run.a = c[i] - perp(t) * (w[i] * r[i]);
        run.b = c[j] - perp(t) * (w[j] * r[j]);
        run.len = vlen(run.b - run.a);
        // the arc on pulley i: from where the previous run arrives to where this one leaves
        const Vec2 tin{std::cos(alpha[h]), std::sin(alpha[h])};
        const Vec2 pin = c[i] - perp(tin) * (w[i] * r[i]), pout = run.a;
        BeltLeg arc;
        arc.arc = true;
        arc.c = c[i];
        arc.r = r[i];
        arc.phi0 = std::atan2(pin.y - c[i].y, pin.x - c[i].x);
        const double phi1 = std::atan2(pout.y - c[i].y, pout.x - c[i].x);
        double sw = w[i] * (phi1 - arc.phi0);
        sw -= 2.0 * kPi * std::floor(sw / (2.0 * kPi));
        arc.sweep = w[i] * sw;
        arc.len = r[i] * sw;
        legs.push_back(arc);
        legs.push_back(run);
    }
    return true;
}

} // namespace

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
    // M16: racks — a toothed bar along the pitch line, on the side away from the pinion, riding its body
    for (size_t k = 0; k < m_.racks.size(); ++k) {
        const RackMesh& rm = m_.racks[k];
        const Gear& gp = m_.gears[(size_t)rm.gear];
        const SlideGeom g = slideGeom(m_, rm.geom, q);
        const double cur = dot(g.u, g.pA - g.pB);
        double t0 = cur, t1 = cur;
        if (k < racks_.size() && racks_[k].t1 >= racks_[k].t0) { t0 = racks_[k].t0; t1 = racks_[k].t1; }
        const double margin = std::max(0.03 * extent_, 2.5 * gp.radius);
        t0 -= margin; t1 += margin;
        const double tooth = gp.module > 0.0 ? gp.module : std::max(0.08 * gp.radius, 3.0 * px_);
        const double pitch = kPi * tooth;
        const Vec2 n = perp(g.u) * (double)rm.side;            // toward the pinion
        const Rgb col = th_.palette[(size_t)(rm.geom.railBody > 0 ? rm.geom.railBody - 1 : 0) % 5];
        const std::string tag = " data-name=\"" + xml(gp.name) + "\"";
        groupOpen(out, "rack", tag);
        const double depth = 3.0 * tooth + 6.0 * px_;
        polygon(out, {g.pB + g.u * t0, g.pB + g.u * t1, g.pB + g.u * t1 - n * depth, g.pB + g.u * t0 - n * depth},
                col, 0.16, col, 2.0, "bar", tag);
        line(out, g.pB + g.u * t0, g.pB + g.u * t1, col, 2.0, "pitch", tag, 0.6);
        groupOpen(out, "teeth");
        for (long i = (long)std::ceil(t0 / pitch); (double)i * pitch <= t1; ++i) {
            const Vec2 p = g.pB + g.u * ((double)i * pitch);
            line(out, p + n * (0.12 * tooth), p - n * (1.1 * tooth), col, 1.5, "tooth", "", 0.9);
        }
        groupClose(out);
        groupClose(out);
    }
    // M16: belts — tangent runs, wrapped arcs, and marks that travel with the belt
    for (const Belt& b : m_.belts) {
        std::vector<BeltLeg> legs;
        if (!beltPath(m_, b, q, legs)) continue;
        double total = 0.0;
        for (const BeltLeg& l : legs) total += l.len;
        const std::string tag = " data-name=\"" + xml(b.name) + "\"";
        const Rgb col = th_.spring;
        groupOpen(out, "belt", tag);
        for (const BeltLeg& l : legs) {
            if (!l.arc) {
                line(out, l.a, l.b, col, 3.0, "run", tag, 0.95, true);
            } else if (l.len > 0.0) {
                std::vector<Vec2> pts;
                const int steps = std::max(2, (int)std::ceil(std::fabs(l.sweep) / (kPi / 36.0)));
                for (int i = 0; i <= steps; ++i) pts.push_back(l.at(l.len * i / steps));
                polyline(out, pts, col, 3.0, 0.95, "wrap", tag);
            }
        }
        // marks: spaced round the belt, advanced by the first pulley's rim travel (circulation: forward is the
        // direction the belt is drawn in, which is the direction of the pulley whose wrap sense matches its spin)
        const Gear& g0 = m_.gears[(size_t)b.gears.front()];
        const double th0 = m_.bodies[(size_t)g0.body].qOffset >= 0 ? q[(size_t)m_.bodies[(size_t)g0.body].qOffset] : 0.0;
        double area = 0.0;
        for (size_t i = 0; i < b.gears.size(); ++i)
            area += crossz(m_.gearCenter(b.gears[i], q), m_.gearCenter(b.gears[(i + 1) % b.gears.size()], q));
        const double circ = area >= 0.0 ? 1.0 : -1.0;
        const double travel = circ * b.sign.front() * g0.radius * th0;          // arclength moved forward
        const double spacing = std::max(total / 24.0, 1e-6);
        groupOpen(out, "marks");
        for (double s = std::fmod(travel, spacing) - spacing; s < total + spacing; s += spacing) {
            double sc = s - total * std::floor(s / total);
            for (const BeltLeg& l : legs) {
                if (sc <= l.len) {
                    const Vec2 p = l.at(sc), p2 = l.at(std::min(sc + 1e-4 * total, l.len));
                    Vec2 dir = p2 - p;
                    if (vlen(dir) < 1e-12) { const Vec2 pm = l.at(std::max(sc - 1e-4 * total, 0.0)); dir = p - pm; }
                    if (vlen(dir) < 1e-12) break;
                    const Vec2 nn = perp(normalize(dir));
                    line(out, p - nn * (4.0 * px_), p + nn * (4.0 * px_), th_.outline, 2.0, "mark");
                    break;
                }
                sc -= l.len;
            }
        }
        groupClose(out);
        groupClose(out);
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
        bool rackLine = false;                    // M16: a ground rack draws its own toothed bar
        for (const RackMesh& rm : m_.racks)
            if (rm.geom.railBody == 0 && vlen(rm.geom.railPos - a.local) < 1e-9 && vlen(rm.geom.railDir - a.dirLocal) < 1e-9) rackLine = true;
        if (rackLine) continue;
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
