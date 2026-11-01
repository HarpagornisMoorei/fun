// mech — viewer.cpp: GLFW + OpenGL animation window (M4)
//
// Rendering uses the compatibility profile with immediate mode on purpose:
// for the walking skeleton the priority is zero-friction portability over
// shader glamour. M2+ will move to core profile + shaders when visuals grow.
//
// Dynamic mechanisms integrate on a fixed-step accumulator (deterministic,
// speed-scalable); kinematic ones re-solve every frame as before. The window
// title carries the live energy readout for dynamic runs.
//
// M4 draws the intent layer's artifacts: dial tick rings (under everything
// but the grid) and clock hands (needles on top of everything).
//
// When compiled without HAVE_VIEWER (no GLFW on the build machine) the file
// builds a stub so the rest of `mech` still works headless.

#include "viewer.hpp"
#include "diagnose.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "geom.hpp"
#include "scene.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#ifdef HAVE_VIEWER

#include <GLFW/glfw3.h>

namespace mech {
namespace viewer {

namespace {

struct Cam {
    double cx = 0, cy = 0, halfH = 1.0;   // world-space window
};

const float kBg[4]     = {0.055f, 0.065f, 0.080f, 1.0f};
const float kGrid[4]   = {1.0f, 1.0f, 1.0f, 0.045f};
const float kTrace[4]  = {0.42f, 0.85f, 0.62f, 0.55f};


void setColor(const float c[4]) { glColor4f(c[0], c[1], c[2], c[3]); }

void drawQuad(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    glBegin(GL_QUADS);
    glVertex2d(a.x, a.y); glVertex2d(b.x, b.y);
    glVertex2d(c.x, c.y); glVertex2d(d.x, d.y);
    glEnd();
}

void drawCircle(Vec2 p, double r, int seg = 18) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(p.x, p.y);
    for (int i = 0; i <= seg; ++i) {
        double a = 2.0 * kPi * (double)i / seg;
        glVertex2d(p.x + r * std::cos(a), p.y + r * std::sin(a));
    }
    glEnd();
}

void drawRing(Vec2 p, double r0, double r1, int seg = 48) {
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= seg; ++i) {
        double a = 2.0 * kPi * (double)i / seg;
        glVertex2d(p.x + r0 * std::cos(a), p.y + r0 * std::sin(a));
        glVertex2d(p.x + r1 * std::cos(a), p.y + r1 * std::sin(a));
    }
    glEnd();
}

// ---- the scene (scene.hpp) in OpenGL ---------------------------------------------------------------
// The same items `mech render` writes as SVG, drawn with the legacy immediate-mode calls this file already uses.

void colorRgb(Rgb c, double a) { glColor4f((float)c.r, (float)c.g, (float)c.b, (float)a); }

int circleSegments(double r, double px) {
    return std::max(16, std::min(128, (int)(2.0 * kPi * r / (4.0 * px))));
}

// Ear-clipping triangulation of a simple polygon, either winding (a cam or a ratchet wheel is not convex).
std::vector<int> earClip(const std::vector<Vec2>& p) {
    const int n = (int)p.size();
    std::vector<int> tri;
    if (n < 3) return tri;
    double area = 0.0;
    for (int i = 0; i < n; ++i) area += crossz(p[(size_t)i], p[(size_t)((i + 1) % n)]);
    const double sgn = area >= 0.0 ? 1.0 : -1.0;
    std::vector<int> idx((size_t)n);
    for (int i = 0; i < n; ++i) idx[(size_t)i] = i;
    auto inside = [&](Vec2 a, Vec2 b, Vec2 c, Vec2 q) {
        return sgn * crossz(b - a, q - a) >= 0.0 && sgn * crossz(c - b, q - b) >= 0.0 && sgn * crossz(a - c, q - c) >= 0.0;
    };
    int guard = 0;
    while (idx.size() > 3 && guard++ < 4 * n) {
        bool clipped = false;
        const size_t m = idx.size();
        for (size_t i = 0; i < m; ++i) {
            const int ia = idx[(i + m - 1) % m], ib = idx[i], ic = idx[(i + 1) % m];
            const Vec2 a = p[(size_t)ia], b = p[(size_t)ib], c = p[(size_t)ic];
            if (sgn * crossz(b - a, c - b) <= 0.0) continue;            // a reflex (or flat) corner is not an ear
            bool empty = true;
            for (size_t k = 0; k < m && empty; ++k) {
                const int ik = idx[k];
                if (ik == ia || ik == ib || ik == ic) continue;
                if (inside(a, b, c, p[(size_t)ik])) empty = false;
            }
            if (!empty) continue;
            tri.push_back(ia); tri.push_back(ib); tri.push_back(ic);
            idx.erase(idx.begin() + (long)i);
            clipped = true;
            break;
        }
        if (!clipped) break;                                           // a degenerate outline: fan what is left
    }
    for (size_t i = 1; i + 1 < idx.size(); ++i) { tri.push_back(idx[0]); tri.push_back(idx[i]); tri.push_back(idx[i + 1]); }
    return tri;
}

void drawItems(const std::vector<SceneItem>& items, double px) {
    for (const SceneItem& it : items) {
        switch (it.kind) {
        case SceneItem::Kind::GroupOpen:
        case SceneItem::Kind::GroupClose:
            break;
        case SceneItem::Kind::Line: {
            const double w = it.strokePx * px;
            colorRgb(it.stroke, it.strokeOpacity);
            if (it.round && it.strokePx > 2.5) {                      // a bar: a rectangle and two round ends
                const Vec2 d = it.pts[1] - it.pts[0];
                const double L = vlen(d);
                if (L < 1e-12) { drawCircle(it.pts[0], 0.5 * w, 12); break; }
                const Vec2 n = perp((1.0 / L) * d) * (0.5 * w);
                drawQuad(it.pts[0] + n, it.pts[1] + n, it.pts[1] - n, it.pts[0] - n);
                drawCircle(it.pts[0], 0.5 * w, 12);
                drawCircle(it.pts[1], 0.5 * w, 12);
            } else {
                glLineWidth((float)it.strokePx);
                glBegin(GL_LINES);
                glVertex2d(it.pts[0].x, it.pts[0].y);
                glVertex2d(it.pts[1].x, it.pts[1].y);
                glEnd();
            }
            break;
        }
        case SceneItem::Kind::Circle: {
            const Vec2 c = it.pts[0];
            const int seg = circleSegments(it.r, px);
            if (it.hasFill) { colorRgb(it.fill, 1.0); drawCircle(c, it.r, seg); }
            if (it.hasStroke) {
                const double w = it.strokePx * px;
                colorRgb(it.stroke, it.strokeOpacity);
                if (it.strokePx > 3.0) {                                // a thick ring (a ring gear's rim): an annulus
                    drawRing(c, std::max(0.0, it.r - 0.5 * w), it.r + 0.5 * w, seg);
                } else {
                    glLineWidth((float)it.strokePx);
                    glBegin(GL_LINE_LOOP);
                    for (int i = 0; i < seg; ++i) {
                        const double a = 2.0 * kPi * (double)i / seg;
                        glVertex2d(c.x + it.r * std::cos(a), c.y + it.r * std::sin(a));
                    }
                    glEnd();
                }
            }
            break;
        }
        case SceneItem::Kind::Polygon: {
            if (it.hasFill) {
                colorRgb(it.fill, it.fillOpacity);
                const std::vector<int> tri = earClip(it.pts);
                for (size_t i = 0; i + 2 < tri.size(); i += 3) {
                    glBegin(GL_TRIANGLE_FAN);
                    for (int k = 0; k < 3; ++k) glVertex2d(it.pts[(size_t)tri[i + (size_t)k]].x, it.pts[(size_t)tri[i + (size_t)k]].y);
                    glEnd();
                }
            }
            if (it.hasStroke) {
                colorRgb(it.stroke, it.strokeOpacity);
                glLineWidth((float)it.strokePx);
                glBegin(GL_LINE_LOOP);
                for (const Vec2& v : it.pts) glVertex2d(v.x, v.y);
                glEnd();
            }
            break;
        }
        case SceneItem::Kind::Polyline:
            colorRgb(it.stroke, it.strokeOpacity);
            glLineWidth((float)it.strokePx);
            glBegin(GL_LINE_STRIP);
            for (const Vec2& v : it.pts) glVertex2d(v.x, v.y);
            glEnd();
            break;
        }
    }
}

double niceStep(double raw) {
    double p = std::pow(10.0, std::floor(std::log10(raw)));
    double m = raw / p;
    double nice = (m < 1.5) ? 1.0 : (m < 3.5) ? 2.0 : (m < 7.5) ? 5.0 : 10.0;
    return nice * p;
}

void drawGrid(const Cam& cam, double aspect) {
    double l = cam.cx - cam.halfH * aspect, r = cam.cx + cam.halfH * aspect;
    double b = cam.cy - cam.halfH, t = cam.cy + cam.halfH;
    double span = std::max(r - l, t - b);
    double step = niceStep(span / 12.0);
    setColor(kGrid);
    glBegin(GL_LINES);
    for (double x = std::floor(l / step) * step; x <= r; x += step) {
        glVertex2d(x, b); glVertex2d(x, t);
    }
    for (double y = std::floor(b / step) * step; y <= t; y += step) {
        glVertex2d(l, y); glVertex2d(r, y);
    }
    glEnd();
}

} // namespace

int run(const Model& m, double speedMul, bool startPaused, const std::string& traceRef) {
    if (!glfwInit()) {
        std::fprintf(stderr, "error: glfwInit failed\n");
        return 1;
    }
    glfwSetErrorCallback([](int code, const char* desc) {
        std::fprintf(stderr, "glfw error %d: %s\n", code, desc ? desc : "?");
    });

    GLFWwindow* win = glfwCreateWindow(1000, 720, ("mech - " + m.name).c_str(), nullptr, nullptr);
    if (!win) {
        std::fprintf(stderr, "error: could not create window (no display / GL driver?)\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    // --- simulation state ---------------------------------------------------
    const bool dynamic = m.world.dynamic;
    ConstraintSystem sys(m);
    DynamicsSystem dyn(m, sys);
    std::vector<double> q;
    std::vector<double> qd(m.nq(), 0.0);
    double accum = 0.0;
    std::vector<std::string> warns = m.assemble(q);
    for (const std::string& w : warns) std::fprintf(stderr, "warning: %s\n", w.c_str());
    int iters = 0;
    double res0 = 0;
    if (!sys.solve(q, 0.0, iters, &res0)) {
        std::fprintf(stderr,
                     "warning: initial assembly did not fully converge (max residual %.3e)\n",
                     res0);
    }
    if (dynamic) dyn.projectVelocities(q, qd);   // rate-driven bodies start at speed
    // M22: the window says so when the mechanism reaches a singular pose (a branch point, the end of its range). A pose it starts
    // on was reported by `mech run` before the window opened.
    RunMonitor monitor(sys);
    monitor.observe(q, 0.0);

    int traceBody = -1, traceAnchor = -1;
    if (!traceRef.empty()) {
        size_t dot = traceRef.find('.');
        if (dot == std::string::npos) {
            std::fprintf(stderr, "error: --trace expects 'body.anchor', got '%s'\n", traceRef.c_str());
            glfwDestroyWindow(win); glfwTerminate(); return 1;
        }
        traceBody = m.bodyIndex(traceRef.substr(0, dot));
        traceAnchor = traceBody >= 0 ? m.anchorIndex(traceBody, traceRef.substr(dot + 1)) : -1;
        if (traceBody < 0 || traceAnchor < 0) {
            std::fprintf(stderr, "error: unknown trace anchor '%s'\n", traceRef.c_str());
            glfwDestroyWindow(win); glfwTerminate(); return 1;
        }
    }

    std::vector<Vec2> trail;

    // How far each slide's guide has travelled along its rail so far, so a rail is drawn where the follower
    // goes (a live view cannot know the whole motion in advance). One entry per slide.
    std::vector<RailSpan> spans(m.slides.size());
    for (size_t si = 0; si < m.slides.size(); ++si) {
        spans[si].pos = m.slides[si].railPos;
        spans[si].dir = m.slides[si].railDir;
    }
    std::vector<RailSpan> rackSpans = newRackSpans(m);
    auto updateSpans = [&]() {
        growRackSpans(m, q, rackSpans);
        for (size_t si = 0; si < m.slides.size(); ++si) {
            const SlideGeom g = slideGeom(m, m.slides[si], q);
            const double t = m.slides[si].railBody == 0 ? dot(g.u, g.pA - m.slides[si].railPos) : dot(g.u, g.pA - g.pB);
            spans[si].t0 = std::min(spans[si].t0, t);
            spans[si].t1 = std::max(spans[si].t1, t);
        }
    };
    updateSpans();

    // camera fit over all anchor positions, gear circles, dials and hands
    Cam cam;
    auto fitCamera = [&]() {
        double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
        auto grow = [&](Vec2 p) {
            minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        };
        for (size_t bi = 0; bi < m.bodies.size(); ++bi)
            for (size_t ai = 0; ai < m.bodies[bi].anchors.size(); ++ai) {
                grow(m.worldAnchor((int)bi, (int)ai, q));
            }
        for (const Gear& g : m.gears) {
            const Vec2 c = m.worldAnchor(g.body, g.hostAnchor, q);
            grow(c + Vec2{g.radius, 0.0});
            grow(c - Vec2{g.radius, 0.0});
            grow(c + Vec2{0.0, g.radius});
            grow(c - Vec2{0.0, g.radius});
        }
        for (const Profile& pr : m.profiles) {          // M6: the whole outline, any pose
            const Vec2 c = m.worldAnchor(pr.body, pr.hostAnchor, q);
            const double r = pr.curve->maxRadius();
            grow(c + Vec2{r, r});
            grow(c - Vec2{r, r});
        }
        for (const Dial& d : m.dials) {
            if (d.radius <= 0.0) continue;
            grow(d.center + Vec2{d.radius, 0.0});
            grow(d.center - Vec2{d.radius, 0.0});
            grow(d.center + Vec2{0.0, d.radius});
            grow(d.center - Vec2{0.0, d.radius});
        }
        for (const Hand& h : m.hands) {
            const Vec2 mount = m.worldAnchor(h.body, h.mountAnchor, q);
            grow(mount + Vec2{h.length, 0.0});
            grow(mount - Vec2{h.length, 0.0});
            grow(mount + Vec2{0.0, h.length});
            grow(mount - Vec2{0.0, h.length});
        }
        cam.cx = 0.5 * (minx + maxx);
        cam.cy = 0.5 * (miny + maxy);
        // padding is 0.35 m for meter-scale mechanisms, but shrinks with the
        // extent so a centimeter-scale cam does not drown in margin
        const double halfExtent = std::max(0.5 * (maxy - miny), 0.5 * (maxx - minx));
        cam.halfH = halfExtent * 1.35 + std::min(0.35, 2.0 * halfExtent);
    };
    fitCamera();

    double simT = 0.0;
    bool paused = startPaused;
    double speed = speedMul;
    double lastGlfwT = glfwGetTime();
    double titleClock = 0.0;

    bool dragging = false;
    double lastX = 0, lastY = 0;
    static const double kSpeeds[] = {0.25, 0.5, 1.0, 2.0, 4.0, 8.0};

    int prevSpace = GLFW_RELEASE, prevR = GLFW_RELEASE, prevF = GLFW_RELEASE;

    std::printf("controls: SPACE pause/resume | R reset | F refit view | wheel zoom | drag pan | 1-6 speed | Q/ESC quit\n");

    const char* autoQuit = std::getenv("MECH_FRAMES");   // testing hook
    long quitAt = autoQuit ? std::atol(autoQuit) : -1;
    long frames = 0;

    // testing hook: MECH_SHOT="frame:file.ppm" dumps the framebuffer after
    // <frame> rendered frames (reads the front buffer after swap)
    const char* shotEnv = std::getenv("MECH_SHOT");
    long shotFrame = -1;
    std::string shotPath;
    if (shotEnv) {
        std::string s(shotEnv);
        size_t colon = s.find(':');
        if (colon != std::string::npos) {
            shotFrame = std::atol(s.substr(0, colon).c_str());
            shotPath = s.substr(colon + 1);
        }
    }

    while (!glfwWindowShouldClose(win)) {
        int W = 0, H = 0;
        glfwGetFramebufferSize(win, &W, &H);
        if (W <= 0 || H <= 0) { glfwPollEvents(); continue; }
        glViewport(0, 0, W, H);
        const double aspect = (double)W / (double)H;
        const double pxWorld = 2.0 * cam.halfH / (double)H;   // world units per pixel

        double now = glfwGetTime();
        double dt = std::min(now - lastGlfwT, 0.05);
        lastGlfwT = now;

        // --- input ----------------------------------------------------------
        auto edge = [&](int key, int& prev) {
            int s = glfwGetKey(win, key);
            bool hit = (s == GLFW_PRESS && prev == GLFW_RELEASE);
            prev = s;
            return hit;
        };
        if (edge(GLFW_KEY_SPACE, prevSpace)) paused = !paused;
        if (edge(GLFW_KEY_R, prevR)) {
            m.assemble(q);
            sys.solve(q, 0.0, iters);
            qd.assign(m.nq(), 0.0);
            if (dynamic) dyn.projectVelocities(q, qd);
            dyn.resetHybrid();   // M5: rewind the discrete state and its ledger
            monitor.reset();     // M22
            monitor.observe(q, 0.0);
            simT = 0.0; accum = 0.0; trail.clear();
            for (RailSpan& sp : spans) { sp.t0 = 1e30; sp.t1 = -1e30; }
            updateSpans();
            std::printf("reset\n");
        }
        if (edge(GLFW_KEY_F, prevF)) fitCamera();
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS ||
            glfwGetKey(win, GLFW_KEY_Q) == GLFW_PRESS) {
            glfwSetWindowShouldClose(win, GLFW_TRUE);
        }
        for (int k = 0; k < 6; ++k) {
            if (glfwGetKey(win, GLFW_KEY_1 + k) == GLFW_PRESS) speed = kSpeeds[k];
        }

        double mx, my;
        glfwGetCursorPos(win, &mx, &my);
        if (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            if (dragging) {
                cam.cx -= (mx - lastX) * pxWorld;
                cam.cy += (my - lastY) * pxWorld;
            }
            dragging = true; lastX = mx; lastY = my;
        } else {
            dragging = false;
        }

        // --- advance ----------------------------------------------------------
        if (!paused) {
            if (dynamic) {
                // fixed-step accumulator: deterministic integration
                accum += speed * dt;
                const double h = m.world.step;
                long guard = 0;
                while (accum >= h && guard < 4000) {
                    dyn.step(q, qd, simT);
                    simT += h;
                    accum -= h;
                    ++guard;
                    monitor.observe(q, simT);
                }
                if (guard >= 4000) accum = 0.0;   // too far behind; drop the debt
            } else {
                simT += speed * dt;
                double stepRes = 0.0;
                sys.solve(q, simT, iters, &stepRes);
                monitor.observe(q, simT, stepRes);
            }
            for (const std::string& line : monitor.startedSince()) std::fprintf(stderr, "warning: %s\n", line.c_str());
            if (traceBody >= 0 && (long)trail.size() < 20000) {
                trail.push_back(m.worldAnchor(traceBody, traceAnchor, q));
            }
            updateSpans();
            titleClock += dt;
            if (dynamic && titleClock > 0.25) {
                titleClock = 0.0;
                const double E = dyn.kinetic(q, qd) + dyn.potential(q);
                char title[160];
                std::snprintf(title, sizeof title, "mech - %s | t = %.2f s | E = %.4f J",
                              m.name.c_str(), simT, E);
                glfwSetWindowTitle(win, title);
            }
        }

        // --- draw ---------------------------------------------------------------
        glClearColor(kBg[0], kBg[1], kBg[2], kBg[3]);
        glClear(GL_COLOR_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(cam.cx - cam.halfH * aspect, cam.cx + cam.halfH * aspect,
                cam.cy - cam.halfH, cam.cy + cam.halfH, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        drawGrid(cam, aspect);

        // The picture is the scene (scene.hpp): the same items `mech render` writes as SVG. The ground and
        // dials first, then the trace, then the mechanism on top.
        {
            std::vector<RailSpan> groundSpans;
            for (size_t si = 0; si < m.slides.size(); ++si)
                if (m.slides[si].railBody == 0) groundSpans.push_back(spans[si]);
            const SceneBuilder scene(m, darkTheme(), pxWorld, 2.0 * cam.halfH * std::max(aspect, 1.0), groundSpans, spans, rackSpans);
            std::vector<SceneItem> items;
            scene.fixed(items);
            drawItems(items, pxWorld);
            if (trail.size() > 1) {
                setColor(kTrace);
                glLineWidth(2.0f);
                glBegin(GL_LINE_STRIP);
                for (const Vec2& p : trail) glVertex2d(p.x, p.y);
                glEnd();
            }
            items.clear();
            scene.pose(q, items);
            drawItems(items, pxWorld);
        }

        glfwSwapBuffers(win);
        glfwPollEvents();

        if (shotFrame >= 0 && frames == shotFrame && W > 0 && H > 0) {
            std::vector<unsigned char> px((size_t)W * H * 3);
            glReadBuffer(GL_FRONT);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            if (FILE* f = std::fopen(shotPath.c_str(), "wb")) {
                std::fprintf(f, "P6\n%d %d\n255\n", W, H);
                for (int y = H - 1; y >= 0; --y)             // flip to top-down
                    std::fwrite(&px[(size_t)y * W * 3], 1, (size_t)W * 3, f);
                std::fclose(f);
                std::fprintf(stderr, "wrote screenshot %s (%dx%d)\n", shotPath.c_str(), W, H);
            }
            shotFrame = -1;
            if (quitAt < 0) glfwSetWindowShouldClose(win, GLFW_TRUE);
        }

        ++frames;
        if (quitAt >= 0 && frames >= quitAt) break;
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}

} // namespace viewer
} // namespace mech

#else // !HAVE_VIEWER — stub build (no GLFW found on this machine)

namespace mech {
namespace viewer {

int run(const Model& m, double speedMul, bool startPaused, const std::string& traceRef) {
    (void)m; (void)speedMul; (void)startPaused; (void)traceRef;
    std::fprintf(stderr,
        "error: this `mech` binary was built without the OpenGL viewer (GLFW not found\n"
        "       at build time). Install it and rebuild, e.g. on Arch:\n"
        "           sudo pacman -S glfw pkgconf && make clean && make\n"
        "       or run headless:  mech run <file.mech> --headless <seconds>\n");
    return 2;
}

} // namespace viewer
} // namespace mech

#endif // HAVE_VIEWER
