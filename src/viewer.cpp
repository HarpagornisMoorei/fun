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
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "geom.hpp"

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
const float kGround[4] = {0.60f, 0.64f, 0.68f, 1.0f};
const float kWhite[4]  = {1.0f, 1.0f, 1.0f, 0.92f};
const float kDark[4]   = {0.05f, 0.06f, 0.08f, 1.0f};
const float kTrace[4]  = {0.42f, 0.85f, 0.62f, 0.55f};
const float kSpring[4] = {0.92f, 0.55f, 0.36f, 0.95f};
const float kDial[4]   = {0.72f, 0.75f, 0.78f, 0.55f};
const float kDialMaj[4]= {0.85f, 0.88f, 0.90f, 0.95f};
const float kHand[4]   = {0.96f, 0.93f, 0.84f, 0.96f};

const float kPalette[][4] = {
    {0.91f, 0.69f, 0.29f, 1.0f},   // amber
    {0.44f, 0.70f, 0.81f, 1.0f},   // steel blue
    {0.79f, 0.44f, 0.44f, 1.0f},   // brick
    {0.62f, 0.82f, 0.54f, 1.0f},   // sage
    {0.70f, 0.62f, 0.86f, 1.0f},   // lilac
};

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

// rounded thick segment: rectangle + two end caps
void drawCapsule(Vec2 a, Vec2 b, double w, const float col[4]) {
    setColor(col);
    Vec2 d = b - a;
    double L = vlen(d);
    if (L < 1e-9) { drawCircle(a, w / 2); return; }
    Vec2 u = (1.0 / L) * d;
    Vec2 n = perp(u) * (w / 2);
    drawQuad(a + n, b + n, b - n, a - n);
    drawCircle(a, w / 2);
    drawCircle(b, w / 2);
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

// hatched fixed segment: a capsule with tick marks on one side
void drawHatched(const Vec2& a, const Vec2& b, double pxWorld, double halfSpan) {
    Vec2 d = b - a;
    const double L = vlen(d);
    if (L < 1e-9) return;
    const Vec2 u = (1.0 / L) * d;
    const Vec2 n = perp(u);
    const Vec2 aa = a - u * (halfSpan - 0.5 * L);
    const Vec2 bb = b + u * (halfSpan - 0.5 * L);
    drawCapsule(aa, bb, 3.0 * pxWorld, kGround);
    setColor(kGround);
    glLineWidth(1.5f);
    glBegin(GL_LINES);
    const double step = 12.0 * pxWorld;
    const Vec2 tick = normalize(n - u) * (9.0 * pxWorld);
    for (double s = 0; s <= vlen(bb - aa); s += step) {
        Vec2 p = aa + u * s;
        glVertex2d(p.x, p.y);
        glVertex2d(p.x + tick.x, p.y + tick.y);
    }
    glEnd();
}

// coil-style spring between two world points
void drawSpring(Vec2 a, Vec2 b, double pxWorld) {
    const Vec2 d = b - a;
    const double L = vlen(d);
    if (L < 1e-9) return;
    const Vec2 u = (1.0 / L) * d;
    const Vec2 n = perp(u);
    const double amp = std::min(0.045 * L + 4.0 * pxWorld, 0.09 * L + 12.0 * pxWorld);
    const double lead = std::min(0.12 * L, 0.35);
    const int coils = std::max(6, (int)(L / (6.0 * amp + 1e-9)));
    setColor(kSpring);
    glLineWidth(1.8f);
    glBegin(GL_LINE_STRIP);
    glVertex2d(a.x, a.y);
    Vec2 p = a + u * lead;
    glVertex2d(p.x, p.y);
    for (int i = 0; i < coils; ++i) {
        const double s1 = lead + (L - 2.0 * lead) * (i + 0.5) / coils;
        const Vec2 q1 = a + u * s1 + n * ((i % 2 == 0) ? amp : -amp);
        glVertex2d(q1.x, q1.y);
    }
    p = b - u * lead;
    glVertex2d(p.x, p.y);
    glVertex2d(b.x, b.y);
    glEnd();
}

// M4: a dial — a thin ring plus minor/major tick marks, static in the world.
// 12 o'clock is up; ticks run clockwise (the natural reading of a clock).
void drawDial(const Dial& d, double pxWorld) {
    if (d.radius <= 0.0) return;
    const double R = d.radius;
    // ring
    setColor(kDial);
    drawRing(d.center, R - 2.5 * pxWorld, R, 96);
    // minor ticks
    if (d.minor > 0) {
        setColor(kDial);
        glLineWidth(1.2f);
        glBegin(GL_LINES);
        for (int i = 0; i < d.minor; ++i) {
            const double ang = -0.5 * kPi + 2.0 * kPi * (double)i / d.minor;
            const Vec2 u{std::cos(ang), std::sin(ang)};
            const Vec2 p0 = d.center + u * (R - 8.0 * pxWorld);
            const Vec2 p1 = d.center + u * (R - 2.5 * pxWorld);
            glVertex2d(p0.x, p0.y);
            glVertex2d(p1.x, p1.y);
        }
        glEnd();
    }
    // major ticks: longer and brighter
    if (d.major > 0) {
        setColor(kDialMaj);
        glLineWidth(2.2f);
        glBegin(GL_LINES);
        for (int i = 0; i < d.major; ++i) {
            const double ang = -0.5 * kPi + 2.0 * kPi * (double)i / d.major;
            const Vec2 u{std::cos(ang), std::sin(ang)};
            const Vec2 p0 = d.center + u * (R - 16.0 * pxWorld);
            const Vec2 p1 = d.center + u * (R - 2.5 * pxWorld);
            glVertex2d(p0.x, p0.y);
            glVertex2d(p1.x, p1.y);
        }
        glEnd();
    }
}

// M4: a clock hand — a tapered needle with a counterweight tail, mounted at
// a world point and rotated by the host's angle. Local +x is the pointing
// direction; 12 o'clock reading is whatever 'from:'/'drive' the author chose.
void drawHand(const Model& m, const Hand& h, const std::vector<double>& q,
              double pxWorld) {
    const Body& b = m.bodies[(size_t)h.body];
    const double th = b.qOffset >= 0 ? q[(size_t)b.qOffset + 0] : 0.0;
    const Vec2 mount = m.worldAnchor(h.body, h.mountAnchor, q);
    const double L = h.length;
    const double w1 = 4.0 * pxWorld;   // base half-width
    const double w2 = 1.6 * pxWorld;   // tip half-width
    auto local = [&](double lx, double ly) {
        const Vec2 v = rotate(Vec2{lx, ly}, th) + mount;
        return v;
    };
    setColor(kHand);
    glBegin(GL_POLYGON);
    Vec2 p = local(-0.22 * L, 0.0);                 glVertex2d(p.x, p.y);
    p = local(-0.10 * L,  w1);                      glVertex2d(p.x, p.y);
    p = local( 0.72 * L,  w2);                      glVertex2d(p.x, p.y);
    p = local( L,          0.0);                    glVertex2d(p.x, p.y);
    p = local( 0.72 * L, -w2);                      glVertex2d(p.x, p.y);
    p = local(-0.10 * L, -w1);                      glVertex2d(p.x, p.y);
    glEnd();
    // hub
    setColor(kWhite);
    drawCircle(mount, 5.5 * pxWorld);
    setColor(kDark);
    drawCircle(mount, 2.3 * pxWorld);
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
            simT = 0.0; accum = 0.0; trail.clear();
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
                }
                if (guard >= 4000) accum = 0.0;   // too far behind; drop the debt
            } else {
                simT += speed * dt;
                sys.solve(q, simT, iters);
            }
            if (traceBody >= 0 && (long)trail.size() < 20000) {
                trail.push_back(m.worldAnchor(traceBody, traceAnchor, q));
            }
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

        // M4: dials live just above the grid, under the mechanism
        for (const Dial& d : m.dials) drawDial(d, pxWorld);

        // trail (coupler curve etc.)
        if (trail.size() > 1) {
            setColor(kTrace);
            glLineWidth(2.0f);
            glBegin(GL_LINE_STRIP);
            for (const Vec2& p : trail) glVertex2d(p.x, p.y);
            glEnd();
        }

        // ground: hatched segment through its outermost anchors
        if (m.bodies[0].anchors.size() >= 2) {
            const Body& g = m.bodies[0];
            Vec2 pmin{1e30, 1e30}, pmax{-1e30, -1e30};
            for (const Anchor& a : g.anchors) {
                pmin.x = std::min(pmin.x, a.local.x); pmin.y = std::min(pmin.y, a.local.y);
                pmax.x = std::max(pmax.x, a.local.x); pmax.y = std::max(pmax.y, a.local.y);
            }
            if (vlen(pmax - pmin) > 1e-9) {
                drawHatched(pmin, pmax, pxWorld, 0.30 + 0.5 * vlen(pmax - pmin));
            }
        }
        // directional ground anchors render as rails
        for (const Anchor& a : m.bodies[0].anchors) {
            if (!a.hasDir) continue;
            const double half = std::min(0.45, 0.9 * cam.halfH);   // rail fits small views
            const Vec2 d = a.dirLocal * half;
            drawHatched(a.local - d, a.local + d, pxWorld, half);
        }

        // gears: pitch circles + teeth ticks (under the rods). Internal gears
        // (rings) draw as annuli with inward teeth; ground-hosted gears are
        // fixed and drawn at their anchor.
        for (size_t gi = 0; gi < m.gears.size(); ++gi) {
            const Gear& g = m.gears[gi];
            const Body& b = m.bodies[g.body];
            const double th = b.qOffset >= 0 ? q[b.qOffset + 0] : 0.0;
            const Vec2 c = m.worldAnchor(g.body, g.hostAnchor, q);
            const float* col = kPalette[(size_t)(g.body > 0 ? g.body - 1 : 0) % 5];
            bool internal = false;
            for (const MeshJoint& mm : m.meshes)
                if ((mm.gearA == (int)gi || mm.gearB == (int)gi) && mm.sgn < 0)
                    internal = true;
            const double tooth =
                g.module > 0.0 ? g.module : std::max(0.08 * g.radius, 3.0 * pxWorld);

            if (internal) {
                // ring gear: rim annulus + faint inner band + inward teeth
                setColor(col);
                drawRing(c, g.radius + 1.1 * tooth, g.radius + 3.2 * tooth, 64);
                glColor4f(col[0], col[1], col[2], 0.28f);
                drawRing(c, g.radius - 1.05 * tooth, g.radius + 1.1 * tooth, 64);
            }
            // pitch circle
            glColor4f(col[0], col[1], col[2], internal ? 0.6f : 0.45f);
            glLineWidth(2.0f);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < 64; ++i) {
                double ang = 2.0 * kPi * (double)i / 64.0;
                glVertex2d(c.x + g.radius * std::cos(ang),
                           c.y + g.radius * std::sin(ang));
            }
            glEnd();
            // teeth ticks: outward for external gears, inward for rings
            const int N = g.teeth > 0 ? g.teeth : 16;
            glColor4f(col[0], col[1], col[2], 0.9f);
            glLineWidth(1.5f);
            glBegin(GL_LINES);
            for (int i = 0; i < N; ++i) {
                const double ang = th + 2.0 * kPi * (double)i / N;
                const Vec2 u{std::cos(ang), std::sin(ang)};
                const double r0 = internal ? g.radius + 1.1 * tooth : 0.88 * g.radius;
                const double r1 = internal ? g.radius - 1.05 * tooth : 1.10 * g.radius;
                const Vec2 p0 = c + u * r0;
                const Vec2 p1 = c + u * r1;
                glVertex2d(p0.x, p0.y);
                glVertex2d(p1.x, p1.y);
            }
            glEnd();
        }

        // M6: contact profiles (under the rods): a faint fill for star-shaped
        // kinds, the outline in the host's colour. The curve rides its host.
        for (const Profile& pr : m.profiles) {
            const Body& hb = m.bodies[(size_t)pr.body];
            const double th = hb.qOffset >= 0 ? q[hb.qOffset + 0] : 0.0;
            const Vec2 tr = hb.qOffset >= 0 ? Vec2{q[hb.qOffset + 1], q[hb.qOffset + 2]}
                                            : Vec2{0.0, 0.0};
            const Vec2 origin = hb.anchors[(size_t)pr.hostAnchor].local;
            const float* col = kPalette[(size_t)(pr.body > 0 ? pr.body - 1 : 0) % 5];
            const std::vector<Vec2> pts = pr.curve->outline(240);
            auto toWorld = [&](Vec2 p) { return rotate(origin + p, th) + tr; };
            if (pr.curve->starShaped()) {   // star-shaped: fan from the origin
                glColor4f(col[0], col[1], col[2], 0.16f);
                glBegin(GL_TRIANGLE_FAN);
                const Vec2 c = toWorld(Vec2{0.0, 0.0});
                glVertex2d(c.x, c.y);
                for (size_t i = 0; i <= pts.size(); ++i) {
                    const Vec2 w = toWorld(pts[i % pts.size()]);
                    glVertex2d(w.x, w.y);
                }
                glEnd();
            }
            glColor4f(col[0], col[1], col[2], 0.95f);
            glLineWidth(2.5f);
            glBegin(GL_LINE_LOOP);
            for (const Vec2& p : pts) {
                const Vec2 w = toWorld(p);
                glVertex2d(w.x, w.y);
            }
            glEnd();
        }

        // moving bodies: rods as capsules, shafts as hubs with index line
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const Body& b = m.bodies[bi];
            const float* col = kPalette[(bi - 1) % 5];
            if (b.anchors.size() >= 2) {
                Vec2 pa = m.worldAnchor((int)bi, 0, q);
                Vec2 pb = m.worldAnchor((int)bi, 1, q);
                drawCapsule(pa, pb, 7.0 * pxWorld, col);
            } else if (b.anchors.size() == 1) {
                Vec2 c = m.worldAnchor((int)bi, 0, q);
                setColor(col);
                drawCircle(c, 5.5 * pxWorld);
                setColor(kDark);
                drawCircle(c, 2.4 * pxWorld);
                // rotation index line
                setColor(kWhite);
                glLineWidth(2.0f);
                glBegin(GL_LINES);
                const double th = q[b.qOffset + 0];
                glVertex2d(c.x, c.y);
                glVertex2d(c.x + 11.0 * pxWorld * std::cos(th),
                           c.y + 11.0 * pxWorld * std::sin(th));
                glEnd();
            }
        }

        // springs (over the bodies, under the pins)
        for (const SpringJoint& sp : m.springs) {
            Vec2 pa = m.worldAnchor(sp.bodyA, sp.anchA, q);
            Vec2 pb = m.worldAnchor(sp.bodyB, sp.anchB, q);
            drawSpring(pa, pb, pxWorld);
        }

        // M6: touches — the roller (if any) and the contact point on the curve
        for (const TouchJoint& tj : m.touches) {
            const Vec2 tip = m.worldAnchor(tj.body, tj.anchor, q);
            const ContactState cs = m.touchState(tj, q);
            if (tj.roller > 0.0) {
                setColor(kWhite);
                glLineWidth(2.0f);
                glBegin(GL_LINE_LOOP);
                for (int i = 0; i < 40; ++i) {
                    const double a = 2.0 * kPi * (double)i / 40.0;
                    glVertex2d(tip.x + tj.roller * std::cos(a), tip.y + tj.roller * std::sin(a));
                }
                glEnd();
            }
            setColor(kSpring);
            drawCircle(cs.foot, 3.2 * pxWorld);
        }

        // pin joints: white hub with dark bore
        for (const PinJoint& p : m.pins) {
            Vec2 pos = m.worldAnchor(p.bodyA, p.anchA, q);
            setColor(kWhite);
            drawCircle(pos, 5.0 * pxWorld);
            setColor(kDark);
            drawCircle(pos, 2.2 * pxWorld);
        }

        // M4: hands on top of everything
        for (const Hand& h : m.hands) drawHand(m, h, q, pxWorld);

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
