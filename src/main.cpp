// mech — main.cpp: command line interface
//
//   mech check <file.mech>                    static analysis: parse, DOF, assembly
//   mech run   <file.mech> [options]          animate (window) or --headless <sec>
//   mech test                                 run the unit test suite
//
// run options:
//   --headless <sec>   simulate <sec> (kinematic solve or dynamic integration)
//   --trace b.a        trace body b's anchor a in the viewer (e.g. coupler.B)
//   --speed <x>        initial speed multiplier (default 1)
//   --paused           open the window paused

#include "ast.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "record.hpp"
#include "lexer.hpp"
#include "model.hpp"
#include "parser.hpp"
#include "viewer.hpp"
#include "../tests/tests.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace mech;

namespace {

struct ToolError {
    std::string file;
    int line, col;       // 0 = no position
    std::string msg;
};

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw ToolError{path, 0, 0, "cannot open file"};
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

Program loadAndParse(const std::string& path) {
    std::string src;
    try {
        src = readFile(path);
    } catch (const ToolError& e) { throw; }

    std::vector<Token> toks;
    try {
        toks = lex(src);
    } catch (const LexError& e) {
        throw ToolError{path, e.line, e.col, e.msg};
    }
    try {
        return parse(toks);
    } catch (const ParseError& e) {
        throw ToolError{path, e.line, e.col, e.msg};
    }
}

Model buildOrThrow(const std::string& path, const MechanismDecl& decl) {
    try {
        return buildModel(decl);
    } catch (const SemanticError& e) {
        throw ToolError{path, e.line, 0, e.msg};
    }
}

// print "file:line:col: error: msg" with a caret under the offending column
void printError(const ToolError& e) {
    if (e.line > 0) {
        std::printf("%s:%d:%d: error: %s\n", e.file.c_str(), e.line, e.col, e.msg.c_str());
        std::ifstream f(e.file);
        std::string ln;
        int n = 0;
        while (n < e.line && std::getline(f, ln)) ++n;
        if (n == e.line && !ln.empty()) {
            std::printf("    %s\n    ", ln.c_str());
            for (int i = 1; i < e.col; ++i) std:: putchar(ln[(size_t)(i - 1)] == '\t' ? '\t' : ' ');
            std::printf("^\n");
        }
    } else {
        std::printf("%s: error: %s\n", e.file.c_str(), e.msg.c_str());
    }
}

// " roller 0.006" for a touch with a roller, "" for a knife-edge tip
std::string rollerNote(double r) {
    if (r <= 0.0) return "";
    char buf[48];
    std::snprintf(buf, sizeof buf, " roller %g", r);
    return buf;
}

void printHelp() {
    std::printf(
        "mech — mechanism language, M7: contact, cams, gears, friction; CSV export\n"
        "\n"
        "usage:\n"
        "  mech check <file.mech>              parse + DOF analysis + initial assembly\n"
        "  mech run <file.mech> [options]      animate in an OpenGL window\n"
        "  mech test                           run unit tests\n"
        "\n"
        "run options:\n"
        "  --headless <sec>    simulate <sec> seconds without a window, print residuals\n"
        "                      (dynamic files integrate at their world step; the table\n"
        "                      gains an energy column, drive reaction torques and, for\n"
        "                      hybrid files, the mode state and the energy ledger)\n"
        "  --csv <file>        (with --headless) write the whole run as CSV: every body's\n"
        "                      pose, contacts (gap, force, state), and for dynamic files\n"
        "                      velocities, energies and every term of the energy ledger\n"
        "  --probe <b.a>       (repeatable) also record anchor a of body b in the world\n"
        "  --dt <sec>          kinematic frame step (default 1/60; dynamic files use their\n"
        "                      world step)\n"
        "  --every <n>         record every n-th step\n"
        "  --trace <b.a>       draw the path of body b's anchor a (coupler curve)\n"
        "  --speed <x>         initial speed multiplier (default 1)\n"
        "  --paused            start paused\n"
        "\n"
        "mechanism window: SPACE pause | R reset | F refit | wheel zoom | drag pan | 1-6 speed | Q quit\n");
}

int cmdCheck(const std::string& path) {
    Program prog = loadAndParse(path);
    if (prog.mechanisms.empty()) {
        throw ToolError{path, 0, 0, "file contains no mechanism"};
    }
    bool anyError = false;
    for (const MechanismDecl& decl : prog.mechanisms) {
        Model m = buildOrThrow(path, decl);
        DofReport d = m.dofReport();

        std::printf("-- mechanism %s ", m.name.c_str());
        for (int i = 0; i < std::max(1, 46 - (int)m.name.size()); ++i) std:: putchar('-');
        std::printf("\n");

        std::printf("mode: %s", m.world.dynamic ? "dynamic" : "kinematic");
        if (m.world.dynamic) {
            std::printf(" (gravity %g m/s^2, step %g ms)", m.world.gravity,
                        1000.0 * m.world.step);
        }
        std::printf("\n");

        std::printf("bodies %d (ground 1)  ·  pins %d  ·  slides %d  ·  gears %d  ·  meshes %d",
                    d.nBodies, d.nPins, d.nSlides, d.nGears, d.nMeshes);
        if (d.nTangentMeshes > 0) std::printf(" (%d tangent)", d.nTangentMeshes);
        if (d.nTouches > 0) std::printf("  ·  profiles %d  ·  touches %d", d.nProfiles, d.nTouches);
        else if (d.nPolarMeshes > 0) std::printf("  ·  profiles %d  ·  non-circular meshes %d",
                                                  d.nProfiles, d.nPolarMeshes);
        std::printf("  ·  drives %d", d.nDrives);
        if (d.nHolds > 0) std::printf(" (incl. %d mode hold%s)", d.nHolds,
                                      d.nHolds == 1 ? "" : "s");
        if (d.nTorques > 0) std::printf(" + %d torque", d.nTorques);
        if (d.nWeights > 0) std::printf(" + %d weight", d.nWeights);
        if (d.nSprings > 0) std::printf("  ·  springs %d", d.nSprings);
        if (d.nModes > 0) std::printf("  ·  modes %d", d.nModes);
        std::printf("\n");

        const int j = d.nPins + d.nSlides;
        std::printf("Grubler-Kutzbach (planar): F = 3(n-1) - 2j - k = 3*%d - 2*%d - %d = %d\n",
                    d.nBodies - 1, j,
                    d.nMeshes + d.nTangentMeshes + d.nTouches + d.nPolarMeshes, d.freeDof);
        if (d.residualDof == 0) {
            std::printf("residual DOF after drives: 0  ->  fully determined [ok]\n");
        } else if (d.residualDof > 0) {
            if (m.world.dynamic) {
                std::printf("residual DOF after drives: %d  ->  governed by dynamics "
                            "(masses + forces) [ok]\n", d.residualDof);
            } else {
                std::printf("residual DOF after drives: %d  ->  [warn] motion indeterminate: %d free DOF%s\n",
                            d.residualDof, d.residualDof,
                            d.residualDof == 1 ? "" : "s remain undriven");
            }
        } else {
            std::printf("residual DOF after drives: %d  ->  [warn] over-constrained: redundant joints\n",
                        d.residualDof);
        }

        if (!m.gears.empty()) {
            std::printf("gears:\n");
            for (const Gear& g : m.gears) {
                const Body& host = m.bodies[g.body];
                const std::string where = host.name + "." +
                    host.anchors[g.hostAnchor].name;
                if (g.teeth > 0) {
                    std::printf("  %-8s on %-10s pitch radius %.4f  (%d teeth, module %.4f)\n",
                                g.name.c_str(), where.c_str(),
                                g.radius, g.teeth, g.module);
                } else {
                    std::printf("  %-8s on %-10s pitch radius %.4f\n",
                                g.name.c_str(), where.c_str(), g.radius);
                }
            }
        }

        // M6: contact curves
        if (!m.profiles.empty()) {
            std::printf("profiles:\n");
            for (const Profile& pr : m.profiles) {
                const Body& host = m.bodies[(size_t)pr.body];
                const std::string where = host.name + "." +
                    host.anchors[(size_t)pr.hostAnchor].name;
                const ProfileCurve& cv = *pr.curve;
                if (cv.kind() == ProfileCurve::Kind::Circle) {
                    std::printf("  %-8s on %-10s circle radius %.4f\n",
                                pr.name.c_str(), where.c_str(), cv.circleRadius());
                } else if (cv.kind() == ProfileCurve::Kind::Polygon) {
                    std::printf("  %-8s on %-10s polygon, %zu pieces (edges and fillets), "
                                "perimeter %.4f m, smallest fillet %.4f m\n",
                                pr.name.c_str(), where.c_str(), cv.pieces(), cv.period(),
                                cv.minFillet());
                } else if (cv.kind() == ProfileCurve::Kind::Ellipse) {
                    std::printf("  %-8s on %-10s ellipse %.4f x %.4f%s\n",
                                pr.name.c_str(), where.c_str(), cv.ellipseA(), cv.ellipseB(),
                                cv.ellipseFocus() ? " (pivoted at a focus)" : " (centered)");
                } else {
                    std::printf("  %-8s on %-10s %s, %zu knots, radius %.4f .. %.4f\n",
                                pr.name.c_str(), where.c_str(),
                                cv.kind() == ProfileCurve::Kind::Polar ? "polar" : "points",
                                cv.knots(), cv.minRadius(), cv.maxRadius());
                }
            }
        }

        // M6b: what the cam synthesizer decided, and whether it will work
        for (const CamInfo& ci : m.cams) {
            const Profile& pr = m.profiles[(size_t)ci.profile];
            std::string segs;
            for (const CamSegment& sg : ci.spec.segments) {
                char b[96];
                if (!segs.empty()) segs += " | ";
                switch (sg.kind) {
                case CamSegment::Kind::Rise:
                    std::snprintf(b, sizeof b, "rise %g m over %g deg %s", sg.lift,
                                  rad2deg(sg.angle), lawName(sg.law)); break;
                case CamSegment::Kind::Fall:
                    std::snprintf(b, sizeof b, "fall %g m over %g deg %s", sg.lift,
                                  rad2deg(sg.angle), lawName(sg.law)); break;
                case CamSegment::Kind::Return:
                    std::snprintf(b, sizeof b, "return over %g deg %s", rad2deg(sg.angle),
                                  lawName(sg.law)); break;
                case CamSegment::Kind::Dwell:
                    if (sg.restOfTurn) std::snprintf(b, sizeof b, "dwell (rest of turn)");
                    else std::snprintf(b, sizeof b, "dwell %g deg", rad2deg(sg.angle));
                    break;
                }
                segs += b;
            }
            std::printf("cam %s on %s.%s: base %.4f m, roller %.4f m, lift %.4f m, "
                        "turns %s, rail at %g deg\n",
                        ci.name.c_str(), m.bodies[(size_t)pr.body].name.c_str(),
                        m.bodies[(size_t)pr.body].anchors[(size_t)pr.hostAnchor].name.c_str(),
                        ci.spec.base, ci.spec.roller, ci.maxLift,
                        ci.spec.turn > 0 ? "ccw" : "cw", rad2deg(ci.spec.startRad));
            std::printf("  %s\n", segs.c_str());
            const double pa = rad2deg(ci.maxPressure);
            std::printf("  pressure angle: max %.2f deg at %.1f deg of rotation (rail through "
                        "the axle)%s\n", pa, rad2deg(ci.maxPressureAt),
                        pa > 30.0 ? "  [warn] above the usual 30 deg limit — the follower "
                                    "may bind; enlarge the base circle or lengthen the rise"
                                  : "  [ok]");
            if (ci.minConvexRadius < 1e299) {
                std::printf("  undercut: smallest convex pitch-curve radius %.4f m at %.1f deg "
                            "vs roller %.4f m  [ok]\n", ci.minConvexRadius,
                            rad2deg(ci.minConvexAt), ci.spec.roller);
            }
        }

        // M4: what the intent layer synthesized
        for (const ReduceInfo& ri : m.reduceInfos) {
            std::string stages;
            for (size_t k = 0; k < ri.stages.size(); ++k) {
                if (k) stages += " x ";
                stages += std::to_string(ri.stages[k].second) + ":" +
                          std::to_string(ri.stages[k].first);
            }
            std::string dirNote = ri.opposite ? "direction reversed" : "direction kept";
            if (ri.idlers > 0)
                dirNote += " — idler inserted (" + std::to_string(ri.idlerTeeth) +
                           " teeth) for sense";
            std::printf("reduce %s->%s: %d stage%s %s (module %.4f), %s\n",
                        ri.bodyA.c_str(), ri.bodyB.c_str(),
                        (int)ri.stages.size(), ri.stages.size() == 1 ? "" : "s",
                        stages.c_str(), ri.module, dirNote.c_str());
            if (!ri.placed.empty()) {
                std::printf("  auto-placed:");
                for (const std::string& p : ri.placed)
                    std::printf(" %s", p.c_str());
                std::printf("\n");
            }
        }
        for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
            const Body& b = m.bodies[bi];
            if (b.beatSeconds > 0.0) {
                std::printf("rod %-10s beat %.4g s -> length %.4f m "
                            "(compound period %.4g s)\n",
                            b.name.c_str(), b.beatSeconds, b.length,
                            2.0 * b.beatSeconds);
            }
        }

        // initial assembly first: mesh types are detected from it
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        ConstraintSystem sys(m);
        int iters = 0;
        double res = 0;
        bool ok = sys.solve(q, 0.0, iters, &res);

        for (const MeshJoint& mm : m.meshes) {
            const Gear& ga = m.gears[mm.gearA];
            const Gear& gb = m.gears[mm.gearB];
            const bool internal = mm.sgn < 0;
            const double ratio = ga.radius / gb.radius;   // |wB/wA|
            std::printf("mesh %-8s <-> %-8s %s  ratio %.4g:%.4g  ->  output speed x%.4g, %s\n",
                        ga.name.c_str(), gb.name.c_str(),
                        internal ? "internal" : "external",
                        ga.teeth > 0 ? (double)ga.teeth : ga.radius,
                        gb.teeth > 0 ? (double)gb.teeth : gb.radius,
                        ratio,
                        internal ? "same direction" : "direction reversed (external)");
        }
        for (const TouchJoint& tj : m.touches) {
            const Body& tb = m.bodies[(size_t)tj.body];
            const ContactState cs = m.touchState(tj, q);
            std::printf("touch %s.%s on %s%s  ->  contact (%.4f, %.4f), normal (%.3f, %.3f)"
                        "  [%s]\n",
                        tb.name.c_str(), tb.anchors[(size_t)tj.anchor].name.c_str(),
                        m.profiles[(size_t)tj.profile].name.c_str(),
                        rollerNote(tj.roller).c_str(),
                        cs.foot.x, cs.foot.y, cs.normal.x, cs.normal.y,
                        m.world.dynamic
                            ? (tj.engaged ? "unilateral: starts engaged, lifts off when the force would pull"
                                          : "unilateral: starts in flight")
                            : "bilateral: force closure assumed");
            if (m.world.dynamic) {
                std::printf("  restitution %g%s", tj.restitution,
                            tj.restitution == 0.0 ? " (plastic landing)" : "");
                if (tj.friction > 0.0)
                    std::printf("  ·  friction %g (Coulomb, regularized below 2 mm/s)", tj.friction);
                else
                    std::printf("  ·  frictionless");
                std::printf("\n");
            }
        }
        for (const PolarMesh& pm : m.polarMeshes) {
            std::printf("mesh %-8s <-> %-8s non-circular, centers %.4f m apart  ->  "
                        "output speed x%.4g .. x%.4g, direction reversed (external)\n"
                        "  conjugate: log-slopes of r agree to %.1e over a full turn [ok]\n",
                        m.profiles[(size_t)pm.profA].name.c_str(),
                        m.profiles[(size_t)pm.profB].name.c_str(), pm.d,
                        pm.minRatio, pm.maxRatio, pm.conjDefect);
        }
        for (const SpringJoint& sp : m.springs) {
            std::printf("spring %-8s k=%g rest=%.4f damp=%g\n",
                        sp.name.c_str(), sp.k, sp.rest, sp.damp);
        }
        for (const TorqueDrive& td : m.torques) {
            std::printf("torque drive on %-8s %g N*m\n",
                        m.bodies[td.body].name.c_str(), td.torque);
        }
        // M5: weight drives (energy sources with a budget)
        for (const WeightDrive& w : m.weights) {
            std::printf("weight drive on %-8s %g kg on a %g m barrel -> %g N*m %s",
                        m.bodies[w.body].name.c_str(), w.mass, w.barrelR, w.torque,
                        w.sign < 0 ? "clockwise" : "counterclockwise");
            if (w.drop >= 0.0) {
                std::printf(" (drop %g m = %g J)", w.drop,
                            w.mass * m.world.gravity * w.drop);
            }
            std::printf("\n");
        }
        // M5: hybrid elements — states, holds and the transitions between them
        for (const Mode& mo : m.modes) {
            std::string states;
            for (size_t si = 0; si < mo.states.size(); ++si) {
                if (si) states += " ";
                states += mo.states[si].name;
                if (!mo.states[si].holdBodies.empty()) {
                    states += "(hold:";
                    for (size_t k = 0; k < mo.states[si].holdBodies.size(); ++k) {
                        if (k) states += ",";
                        states += m.bodies[(size_t)mo.states[si].holdBodies[k]].name;
                    }
                    states += ")";
                }
            }
            std::printf("mode %-10s %zu state%s: %s (initial: %s)\n",
                        mo.name.c_str(), mo.states.size(),
                        mo.states.size() == 1 ? "" : "s", states.c_str(),
                        mo.states[(size_t)mo.current].name.c_str());
            for (const ModeTransition& tr : mo.transitions) {
                const char* fld = tr.field == ModeTransition::Field::Theta ? "theta"
                                : tr.field == ModeTransition::Field::Omega ? "omega"
                                : "advance";
                std::printf("  %-12s -> %-12s when %s.%s %s %8.2f deg",
                            mo.states[(size_t)tr.from].name.c_str(),
                            mo.states[(size_t)tr.to].name.c_str(),
                            m.bodies[(size_t)tr.body].name.c_str(), fld,
                            tr.greater ? ">" : "<", rad2deg(tr.threshold));
                for (const ModeReset& rs : tr.resets) {
                    if (rs.maintain) {
                        std::printf(" : impulse %s maintain %g deg",
                                    m.bodies[(size_t)rs.body].name.c_str(),
                                    rad2deg(rs.swingRad));
                    } else {
                        std::printf(" : impulse %s %g mJ",
                                    m.bodies[(size_t)rs.body].name.c_str(),
                                    1000.0 * rs.energy);
                    }
                }
                std::printf("\n");
            }
        }
        if (m.world.dynamic && !m.bodies.empty()) {
            std::printf("masses (auto-derived from geometry where not given):\n");
            for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
                const MassProps& mp = m.mass[bi];
                std::printf("  %-10s m=%8.4f kg  com=(%7.4f, %7.4f)  I=%9.6f kg*m^2\n",
                            m.bodies[bi].name.c_str(), mp.mass, mp.com.x, mp.com.y,
                            mp.inertia);
            }
        }

        std::printf("initial assembly (t = 0):\n");
        for (const Body& b : m.bodies) {
            if (b.isGround) {
                std::printf("  %-10s (ground)", b.name.c_str());
                for (const Anchor& a : b.anchors)
                    std::printf("  %s (%.3f, %.3f)", a.name.c_str(), a.local.x, a.local.y);
                std::printf("\n");
            } else {
                std::printf("  %-10s theta=%8.2f deg", b.name.c_str(),
                            rad2deg(q[b.qOffset + 0]));
                for (size_t ai = 0; ai < b.anchors.size(); ++ai) {
                    Vec2 p = m.worldAnchor((int)(&b - &m.bodies[0]), (int)ai, q);
                    std::printf("  %s (%.3f, %.3f)", b.anchors[ai].name.c_str(), p.x, p.y);
                }
                std::printf("\n");
            }
        }
        std::printf("assembly %s (max residual %.2e, %d iterations)\n",
                    ok ? "converged [ok]" : "DID NOT CONVERGE [warn]", res, iters);

        // M4: hands and dials (dials resolve at assembly)
        for (const Hand& h : m.hands) {
            const Vec2 mount = m.worldAnchor(h.body, h.mountAnchor, q);
            std::printf("hand %-8s on %-8s length %.3f  (mount %.3f, %.3f)\n",
                        h.name.c_str(), m.bodies[h.body].name.c_str(),
                        h.length, mount.x, mount.y);
        }
        for (const Dial& d : m.dials) {
            std::printf("dial at (%.3f, %.3f) radius %.3f  (%d minor, %d major ticks)%s\n",
                        d.center.x, d.center.y, d.radius, d.minor, d.major,
                        d.hasCenter ? "" : "  [auto-centered]");
        }

        std::vector<std::string> geomErrs = m.validateAssembly(q);
        if (geomErrs.empty()) {
            if (!m.meshes.empty()) std::printf("mesh tangency: all %d meshes ok\n", d.nMeshes);
            if (!m.polarMeshes.empty())
                std::printf("non-circular meshes: all %d in rolling contact\n", d.nPolarMeshes);
            if (!m.touches.empty()) std::printf("contacts: all %d touches closed\n", d.nTouches);
        } else {
            for (const std::string& e : geomErrs) {
                std::printf("error: %s\n", e.c_str());
                anyError = true;
            }
        }
        for (const std::string& w : warns) std::printf("warning: %s\n", w.c_str());
        std::printf("\n");
    }
    return anyError ? 1 : 0;
}

// What `mech run --headless` should export (M7).
struct RunExport {
    std::string csv;                  // output path ("" = none)
    std::vector<std::string> probes;  // body.anchor, each recorded as world x, y
    double dt = 0.0;                  // kinematic frame step in seconds (0 = 1/60)
    long every = 1;                   // record every n-th step
};

// A CSV file being written: the Recorder plus the open stream.
struct CsvSink {
    std::FILE* f = nullptr;
    const Recorder* rec = nullptr;
    long rows = 0;
    ~CsvSink() { if (f) std::fclose(f); }
    void row(double t, const std::vector<double>& q, const std::vector<double>& qd, double maxRes) {
        std::fprintf(f, "%s\n", Recorder::csvLine(rec->sample(t, q, qd, maxRes)).c_str());
        ++rows;
    }
};

int cmdRun(const std::string& path, double headlessSecs, bool headless,
           const std::string& traceRef, double speed, bool paused, const RunExport& ex) {
    Program prog = loadAndParse(path);
    if (prog.mechanisms.empty()) {
        throw ToolError{path, 0, 0, "file contains no mechanism"};
    }
    Model m = buildOrThrow(path, prog.mechanisms[0]);

    DofReport d = m.dofReport();
    if (d.residualDof > 0 && !m.world.dynamic) {
        std::fprintf(stderr,
                     "warning: mechanism '%s' has %d undriven DOF; motion is indeterminate "
                     "(the solver will hold them at their initial values)\n",
                     m.name.c_str(), d.residualDof);
    }

    std::vector<double> q;
    std::vector<std::string> warns = m.assemble(q);
    for (const std::string& w : warns) std::fprintf(stderr, "warning: %s\n", w.c_str());
    ConstraintSystem sys(m);
    int iters = 0;
    double res = 0;
    if (!sys.solve(q, 0.0, iters, &res)) {
        std::fprintf(stderr, "warning: initial assembly did not converge (max residual %.3e)\n", res);
    }
    std::vector<std::string> geomErrs = m.validateAssembly(q);
    if (!geomErrs.empty()) {
        for (const std::string& e : geomErrs)
            std::fprintf(stderr, "error: %s\n", e.c_str());
        return 1;
    }

    if (!headless && (!ex.csv.empty() || !ex.probes.empty() || ex.dt > 0.0 || ex.every != 1)) {
        throw ToolError{path, 0, 0, "--csv, --probe, --dt and --every need --headless <sec>"};
    }
    std::vector<Probe> probes;
    for (const std::string& ref : ex.probes) {
        const size_t dot = ref.find('.');
        const int bi = dot == std::string::npos ? -1 : m.bodyIndex(ref.substr(0, dot));
        const int ai = bi < 0 ? -1 : m.anchorIndex(bi, ref.substr(dot + 1));
        if (bi < 0 || ai < 0) {
            throw ToolError{path, 0, 0, "--probe expects an existing body.anchor, got '" + ref + "'"};
        }
        if (m.bodies[(size_t)bi].isGround) {
            throw ToolError{path, 0, 0, "--probe '" + ref + "' is on the ground body (it never moves)"};
        }
        probes.push_back(Probe{bi, ai});
    }
    if (ex.every < 1) throw ToolError{path, 0, 0, "--every must be at least 1"};
    if (m.world.dynamic && ex.dt > 0.0) {
        throw ToolError{path, 0, 0, "--dt applies to kinematic files; a dynamic file integrates at "
                                    "its world step (use --every to thin the output)"};
    }
    if (ex.dt < 0.0) throw ToolError{path, 0, 0, "--dt must be positive"};

    if (headless) {
        if (m.world.dynamic) {
            // fixed-step integration, fully deterministic
            DynamicsSystem dyn(m, sys);
            std::vector<double> qd(m.nq(), 0.0);
            dyn.projectVelocities(q, qd);   // rate-driven bodies start at their prescribed speed
            const double h = m.world.step;
            const long steps = (long)std::llround(headlessSecs / h);
            std::printf("headless dynamic run of '%s' for %.2f s (h = %g s, %ld steps)\n",
                        m.name.c_str(), headlessSecs, h, steps);
            std::printf("%8s  %10s  %10s  ", "t", "maxRes", "E");
            if (!m.modes.empty()) std::printf("%-12s", "mode");
            for (const Body& b : m.bodies)
                if (!b.isGround) std::printf("%-10s%-9s%-9s", b.name.c_str(), ".th", ".pos");
            for (const TouchJoint& tj : m.touches)
                std::printf("%-12s", (m.bodies[(size_t)tj.body].name + ".N").c_str());
            std::printf("\n");
            const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
            long reportEvery = std::max(1L, (long)std::llround(0.5 / h));
            double worstRes = 0.0;
            double dEworst = 0.0;
            Recorder rec(m, &dyn, probes);
            rec.setBaseline(E0);
            CsvSink csv;
            if (!ex.csv.empty()) {
                csv.f = std::fopen(ex.csv.c_str(), "w");
                if (!csv.f) throw ToolError{path, 0, 0, "cannot write '" + ex.csv + "'"};
                csv.rec = &rec;
                std::fprintf(csv.f, "%s\n", rec.headerLine().c_str());
                csv.row(0.0, q, qd, sys.maxResidual(q, 0.0));
            }
            for (long k = 1; k <= steps; ++k) {
                const double t = (k - 1) * h;
                dyn.step(q, qd, t);
                if (csv.f && (k % ex.every == 0 || k == steps))
                    csv.row(k * h, q, qd, sys.maxResidual(q, k * h));
                if (k % reportEvery == 0 || k == steps) {
                    const double r = sys.maxResidual(q, k * h);
                    worstRes = std::max(worstRes, r);
                    const double E = dyn.kinetic(q, qd) + dyn.potential(q);
                    dEworst = std::max(dEworst, std::fabs(E - E0));
                    std::printf("%8.3f  %10.2e  %10.4f  ", k * h, r, E);
                    if (!m.modes.empty()) {
                        std::string st;
                        for (const Mode& mo : m.modes) {
                            if (!st.empty()) st += ",";
                            st += mo.name + ":" + mo.states[(size_t)mo.current].name;
                        }
                        std::printf("%-12s", st.c_str());
                    }
                    for (const Body& b : m.bodies)
                        if (!b.isGround)
                            std::printf("%6.2f    %6.3f,%-6.3f",
                                        rad2deg(q[b.qOffset]),
                                        q[b.qOffset + 1], q[b.qOffset + 2]);
                    for (size_t ti = 0; ti < m.touches.size(); ++ti) {
                        if (m.touches[ti].engaged)
                            std::printf("%-12.3f", dyn.contactForces().size() > ti
                                                       ? dyn.contactForces()[ti] : 0.0);
                        else
                            std::printf("%-12s", "off");
                    }
                    std::printf("\n");
                }
            }
            for (size_t di = 0; di < m.drives.size(); ++di) {
                if (!m.drives[di].enabled) continue;   // a disabled mode hold
                std::printf("drive %-8s reaction torque: %9.4f N*m\n",
                            m.bodies[m.drives[di].body].name.c_str(),
                            dyn.driveReactions()[di]);
            }
            // M5: the hybrid ledger — where the weight's energy went
            if (!m.modes.empty() || !m.weights.empty() || !m.touches.empty()) {
                const HybridStats& hs = dyn.hybridStats();
                long fires = 0;
                for (const Mode& mo : m.modes) fires += mo.fires;
                if (!m.modes.empty()) {
                    std::printf("hybrid: %ld transition%s fired",
                                fires, fires == 1 ? "" : "s");
                    for (const Mode& mo : m.modes)
                        std::printf(" (mode %s: %ld)", mo.name.c_str(), mo.fires);
                    std::printf("\n");
                }
                if (!m.touches.empty()) {
                    std::printf("contact: %ld lift-off%s, %ld impact%s",
                                hs.liftoffs, hs.liftoffs == 1 ? "" : "s",
                                hs.impacts, hs.impacts == 1 ? "" : "s");
                    for (const TouchJoint& tj : m.touches)
                        std::printf("  (%s: %s)", m.bodies[(size_t)tj.body].name.c_str(),
                                    tj.engaged ? "on the curve" : "in flight");
                    std::printf("\n");
                }
                for (const WeightDrive& w : m.weights) {
                    std::printf("weight %-8s paid %8.4f",
                                m.bodies[w.body].name.c_str(), w.paid);
                    if (w.drop >= 0.0) {
                        std::printf("/%g m%s", w.drop,
                                    w.exhausted ? " (exhausted — run down)" : "");
                    }
                    std::printf("\n");
                }
                const double dE = (dyn.kinetic(q, qd) + dyn.potential(q)) - E0;
                const double ledger = dE - (hs.weightDrawn + hs.impulseEnergy +
                                            hs.driveWork - hs.impactLoss - hs.frictionLoss);
                std::printf("energy: drawn %.4f J + impulse %.4f J", hs.weightDrawn,
                            hs.impulseEnergy);
                if (hs.driveWork != 0.0) std::printf(" + drive %.4f J", hs.driveWork);
                std::printf(" - impact %.4f J", hs.impactLoss);
                if (hs.frictionLoss != 0.0) std::printf(" - friction %.4f J", hs.frictionLoss);
                std::printf(" = dE %.4f J (ledger residual %.2e J)\n", dE, ledger);
            } else {
                std::printf("done: worst residual %.2e, worst |E - E0| %.2e J\n",
                            worstRes, dEworst);
            }
            if (csv.f)
                std::printf("wrote %s: %ld rows x %zu columns\n", ex.csv.c_str(), csv.rows,
                            rec.columns().size());
            return 0;
        }

        const double dt = ex.dt > 0.0 ? ex.dt : 1.0 / 60.0;
        const long printEvery = std::max(1L, (long)std::llround(0.5 / dt));
        std::printf("headless run of '%s' for %.2f s (dt = %.6g s, angles in deg)\n",
                    m.name.c_str(), headlessSecs, dt);
        std::printf("%8s  %10s  ", "t", "maxRes");
        for (const Body& b : m.bodies)
            if (!b.isGround) std::printf("%-11s", b.name.c_str());
        for (const TouchJoint& tj : m.touches)
            std::printf("%-20s", (m.bodies[(size_t)tj.body].name + "." +
                m.bodies[(size_t)tj.body].anchors[(size_t)tj.anchor].name + " x,y").c_str());
        std::printf("\n");

        Recorder rec(m, nullptr, probes);
        CsvSink csv;
        const std::vector<double> noVel;
        if (!ex.csv.empty()) {
            csv.f = std::fopen(ex.csv.c_str(), "w");
            if (!csv.f) throw ToolError{path, 0, 0, "cannot write '" + ex.csv + "'"};
            csv.rec = &rec;
            std::fprintf(csv.f, "%s\n", rec.headerLine().c_str());
            csv.row(0.0, q, noVel, sys.maxResidual(q, 0.0));
        }
        int frame = 0;
        double maxSeen = 0;
        for (double t = dt; t <= headlessSecs + 1e-9; t += dt, ++frame) {
            sys.solve(q, t, iters);
            double r = sys.maxResidual(q, t);
            maxSeen = std::max(maxSeen, r);
            if (csv.f && (frame + 1) % ex.every == 0) csv.row(t, q, noVel, r);
            if (frame % printEvery == 0) {
                std::printf("%8.3f  %10.2e  ", t, r);
                for (const Body& b : m.bodies)
                    if (!b.isGround) std::printf("%10.2f    ", rad2deg(q[b.qOffset]));
                for (const TouchJoint& tj : m.touches) {
                    const Vec2 p = m.worldAnchor(tj.body, tj.anchor, q);
                    std::printf("%9.5f,%-9.5f  ", p.x, p.y);
                }
                std::printf("\n");
            }
        }
        std::printf("done: %d frames, worst residual %.2e\n", frame, maxSeen);
        if (csv.f)
            std::printf("wrote %s: %ld rows x %zu columns\n", ex.csv.c_str(), csv.rows,
                        rec.columns().size());
        return 0;
    }

    return viewer::run(m, speed, paused, traceRef);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { printHelp(); return 0; }
    const std::string cmd = argv[1];

    try {
        if (cmd == "-h" || cmd == "--help" || cmd == "help") { printHelp(); return 0; }
        if (cmd == "test") return runAllTests() == 0 ? 0 : 1;

        if (cmd == "check") {
            if (argc < 3) { std::fprintf(stderr, "usage: mech check <file.mech>\n"); return 1; }
            return cmdCheck(argv[2]);
        }
        if (cmd == "run") {
            if (argc < 3) {
                std::fprintf(stderr, "usage: mech run <file.mech> [options]\n");
                return 1;
            }
            double headlessSecs = 2.0;
            bool headless = false;
            std::string traceRef;
            double speed = 1.0;
            bool paused = false;
            RunExport ex;
            for (int i = 3; i < argc; ++i) {
                std::string a = argv[i];
                if (a == "--headless") {
                    headless = true;
                    if (i + 1 < argc) headlessSecs = std::atof(argv[++i]);
                } else if (a == "--trace" && i + 1 < argc) {
                    traceRef = argv[++i];
                } else if (a == "--speed" && i + 1 < argc) {
                    speed = std::atof(argv[++i]);
                } else if (a == "--paused") {
                    paused = true;
                } else if (a == "--csv" && i + 1 < argc) {
                    ex.csv = argv[++i];
                } else if (a == "--probe" && i + 1 < argc) {
                    ex.probes.push_back(argv[++i]);
                } else if (a == "--dt" && i + 1 < argc) {
                    ex.dt = std::atof(argv[++i]);
                    if (!(ex.dt > 0.0)) { std::fprintf(stderr, "--dt must be positive\n"); return 1; }
                } else if (a == "--every" && i + 1 < argc) {
                    ex.every = std::atol(argv[++i]);
                } else {
                    std::fprintf(stderr, "unknown option '%s'\n", a.c_str());
                    return 1;
                }
            }
            return cmdRun(argv[2], headlessSecs, headless, traceRef, speed, paused, ex);
        }

        std::fprintf(stderr, "unknown command '%s' (try: check, run, test)\n", cmd.c_str());
        return 1;
    } catch (const ToolError& e) {
        printError(e);
        return 1;
    }
    return 0;
}
