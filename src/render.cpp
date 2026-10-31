// mech — render.cpp: SVG drawings of a mechanism (M13)
#include "render.hpp"
#include "scene.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "geom.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace mech {

namespace {

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

// The scene as SVG. The attributes and their order are the file's format (docs/RENDER.md): a test reads them back.
std::string toSvg(const std::vector<SceneItem>& items, double px) {
    std::string o;
    auto pointsOf = [](const std::vector<Vec2>& pts) {
        std::string s;
        for (size_t i = 0; i < pts.size(); ++i) s += (i ? " " : "") + num(pts[i].x) + "," + num(pts[i].y);
        return s;
    };
    for (const SceneItem& it : items) {
        switch (it.kind) {
        case SceneItem::Kind::GroupOpen:
            o += "<g class=\"" + it.cls + "\"" + it.attrs + ">\n";
            break;
        case SceneItem::Kind::GroupClose:
            o += "</g>\n";
            break;
        case SceneItem::Kind::Line: {
            o += "<line";
            if (!it.cls.empty()) o += " class=\"" + it.cls + "\"";
            o += it.attrs + " x1=\"" + num(it.pts[0].x) + "\" y1=\"" + num(it.pts[0].y) + "\" x2=\"" + num(it.pts[1].x) +
                 "\" y2=\"" + num(it.pts[1].y) + "\" stroke=\"" + hex(it.stroke) + "\" stroke-width=\"" + num(it.strokePx * px) + "\"";
            if (it.strokeOpacity < 1.0) o += " stroke-opacity=\"" + num(it.strokeOpacity) + "\"";
            if (it.round) o += " stroke-linecap=\"round\"";
            o += "/>\n";
            break;
        }
        case SceneItem::Kind::Circle: {
            o += "<circle";
            if (!it.cls.empty()) o += " class=\"" + it.cls + "\"";
            o += it.attrs + " cx=\"" + num(it.pts[0].x) + "\" cy=\"" + num(it.pts[0].y) + "\" r=\"" + num(it.r) + "\" fill=\"" +
                 (it.hasFill ? hex(it.fill) : std::string("none")) + "\"";
            if (it.hasStroke) {
                o += " stroke=\"" + hex(it.stroke) + "\" stroke-width=\"" + num(it.strokePx * px) + "\"";
                if (it.strokeOpacity < 1.0) o += " stroke-opacity=\"" + num(it.strokeOpacity) + "\"";
            }
            o += "/>\n";
            break;
        }
        case SceneItem::Kind::Polygon:
            o += "<polygon class=\"" + it.cls + "\"" + it.attrs + " points=\"" + pointsOf(it.pts) + "\" fill=\"" +
                 (it.hasFill ? hex(it.fill) : std::string("none")) + "\"";
            if (it.fillOpacity < 1.0 && it.hasFill) o += " fill-opacity=\"" + num(it.fillOpacity) + "\"";
            if (it.hasStroke) o += " stroke=\"" + hex(it.stroke) + "\" stroke-width=\"" + num(it.strokePx * px) + "\" stroke-linejoin=\"round\"";
            o += "/>\n";
            break;
        case SceneItem::Kind::Polyline:
            o += "<polyline class=\"" + it.cls + "\"" + it.attrs + " points=\"" + pointsOf(it.pts) + "\" fill=\"none\" stroke=\"" +
                 hex(it.stroke) + "\" stroke-width=\"" + num(it.strokePx * px) + "\" stroke-linejoin=\"round\"";
            if (it.strokeOpacity < 1.0) o += " stroke-opacity=\"" + num(it.strokeOpacity) + "\"";
            o += "/>\n";
            break;
        }
    }
    return o;
}

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

    std::vector<RailSpan> rails, movingRails(m.slides.size());
    for (size_t si = 0; si < m.slides.size(); ++si) {
        const SlideJoint& sj = m.slides[si];
        RailSpan r;
        r.pos = sj.railPos;
        r.dir = sj.railDir;
        for (size_t k = 0; k < times.size(); ++k) {
            if (sj.railBody == 0) {
                const double t = dot(m.worldAnchor(sj.guideBody, sj.guideAnchor, poses[k]) - sj.railPos, sj.railDir);
                r.t0 = std::min(r.t0, t);
                r.t1 = std::max(r.t1, t);
            } else {                                              // measured from the rail anchor, which moves
                const SlideGeom g = slideGeom(m, sj, poses[k]);
                const double t = dot(g.u, g.pA - g.pB);
                movingRails[si].t0 = std::min(movingRails[si].t0, t);
                movingRails[si].t1 = std::max(movingRails[si].t1, t);
            }
        }
        if (sj.railBody == 0) rails.push_back(r);
    }

    std::vector<RailSpan> rackSpans = newRackSpans(m);
    for (size_t k = 0; k < times.size(); ++k) growRackSpans(m, poses[k], rackSpans);

    const Theme th = opt.dark ? darkTheme() : lightTheme();
    const SceneBuilder sb(m, th, pxWorld, std::max(ex, ey), rails, movingRails, rackSpans);
    auto scene = [&](const std::vector<double>& q) {
        std::vector<SceneItem> items;
        sb.pose(q, items);
        return toSvg(items, pxWorld);
    };

    std::ostringstream o;
    o << "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" width=\"" << num(Wpx)
      << "\" height=\"" << num(Hpx) << "\" viewBox=\"0 0 " << num(Wpx) << " " << num(Hpx) << "\" data-mechanism=\""
      << xml(m.name) << "\" data-scale=\"" << num(scale) << "\">\n";
    o << "<title>" << xml(m.name) << "</title>\n";
    o << "<rect width=\"" << num(Wpx) << "\" height=\"" << num(Hpx) << "\" fill=\"" << hex(th.bg) << "\"/>\n";
    // world coordinates (metres, y up) -> pixels
    o << "<g id=\"world\" transform=\"matrix(" << num(scale) << " 0 0 " << num(-scale) << " " << num(Wpx / 2 - cx * scale)
      << " " << num(Hpx / 2 + cy * scale) << ")\">\n";
    {
        std::vector<SceneItem> fixedItems;
        sb.fixed(fixedItems);
        if (!trail.empty()) {
            SceneItem tr;
            tr.kind = SceneItem::Kind::Polyline;
            tr.cls = "trace";
            tr.attrs = " data-anchor=\"" + xml(opt.trace) + "\"";
            tr.pts = trail;
            tr.hasStroke = true;
            tr.stroke = th.trace;
            tr.strokePx = 2.0;
            tr.strokeOpacity = 0.75;
            fixedItems.push_back(tr);
        }
        o << toSvg(fixedItems, pxWorld);
    }
    if (frames == 1) {
        o << "<g class=\"pose\" data-t=\"" << num(times[0]) << "\">\n" << scene(poses[0]) << "</g>\n";
    } else {
        o << "<defs>\n";
        for (int k = 0; k < frames; ++k)
            o << "<g id=\"f" << k << "\" class=\"pose\" data-t=\"" << num(times[(size_t)k]) << "\">\n"
              << scene(poses[(size_t)k]) << "</g>\n";
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
