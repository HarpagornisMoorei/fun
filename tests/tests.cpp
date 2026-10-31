// mech — tests.cpp: unit tests for the M1 + M2 core (no OpenGL required)
#include "tests.hpp"

#include "../src/geom.hpp"
#include "../src/cam.hpp"
#include "../src/dynamics.hpp"
#include "../src/kinematics.hpp"
#include "../src/lexer.hpp"
#include "../src/model.hpp"
#include "../src/parser.hpp"
#include "../src/profile.hpp"
#include "../src/record.hpp"
#include "../src/sweep.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

using namespace mech;

namespace {

int g_checks = 0, g_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++g_checks;                                                            \
        if (!(cond)) {                                                         \
            ++g_failures;                                                      \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                      \
    } while (0)

#define CHECK_APPROX(a, b, tol)                                                \
    do {                                                                       \
        ++g_checks;                                                            \
        double _da = (a), _db = (b);                                           \
        if (std::fabs(_da - _db) > (tol)) {                                    \
            ++g_failures;                                                      \
            std::fprintf(stderr, "FAIL %s:%d: %s = %g vs %s = %g (tol %g)\n",  \
                         __FILE__, __LINE__, #a, _da, #b, _db, (double)(tol)); \
        }                                                                      \
    } while (0)

Program parseText(const char* src) {
    return parse(lex(src));
}

Model buildFromText(const char* src) {
    return buildModel(parseText(src).mechanisms[0]);
}

// ---- shared fixtures --------------------------------------------------------

const char* kPendulum = R"(
mechanism pendulum {
  ground frame {
    anchor pivot at (0, 0)
  }
  rod arm { length: 1.0 }
  pin arm.A -> frame.pivot
  drive arm at 40 deg/s from -60 deg
}
)";

const char* kFourbar = R"(
mechanism fourbar {
  ground base {
    anchor p1 at (0, 0)
    anchor p2 at (3, 0)
  }
  rod crank   { length: 0.8 }
  rod coupler { length: 2.6 }
  rod rocker  { length: 2.2 }
  pin crank.A   -> base.p1
  pin coupler.A -> crank.B
  pin rocker.A  -> base.p2
  pin coupler.B -> rocker.B
  drive crank at 36 deg/s from 60 deg
}
)";

const char* kUndriven = R"(
mechanism undriven {
  ground base {
    anchor p1 at (0, 0)
    anchor p2 at (3, 0)
  }
  rod crank   { length: 0.8 }
  rod coupler { length: 2.6 }
  rod rocker  { length: 2.2 }
  pin crank.A   -> base.p1
  pin coupler.A -> crank.B
  pin rocker.A  -> base.p2
  pin coupler.B -> rocker.B
}
)";

const char* kDoublePend = R"(
mechanism doublepend {
  ground frame { anchor pivot at (0, 0) }
  rod upper { length: 0.9 }
  rod lower { length: 0.7 }
  pin upper.A -> frame.pivot
  pin lower.A -> upper.B
  drive upper at 25 deg/s from 80 deg
  drive lower at -47 deg/s from -40 deg
}
)";

// ---- M2 fixtures --------------------------------------------------------------

const char* kPair = R"(
mechanism pair {
  ground frame {
    anchor p1 at (0, 0)
    anchor p2 at (0.24, 0)
  }
  shaft a at frame.p1
  shaft b at frame.p2
  gear g1 { teeth: 8 } on a
  gear g2 { teeth: 40 } on b
  mesh g1 g2
  drive a at 60 rpm
}
)";

const char* kGeartrain = R"(
mechanism geartrain {
  ground frame {
    anchor s1 at (0, 0)
    anchor s2 at (0.24, 0)
    anchor s3 at (0.36, 0.2078460969)
  }
  shaft a at frame.s1
  shaft b at frame.s2
  shaft c at frame.s3
  gear g1 { teeth: 8 } on a
  gear g2 { teeth: 40 } on b
  gear g3 { teeth: 8 } on b
  gear g4 { teeth: 40 } on c
  mesh g1 g2
  mesh g3 g4
  drive a at 60 rpm
}
)";

const char* kCrankSlider = R"(
mechanism crankslider {
  ground base {
    anchor main at (0, 0)
    anchor rail at (0.9, 0) dir (1, 0)
  }
  rod crank on base.main { length: 0.3 }
  body piston {
    anchor guide at (0, 0) dir (1, 0)
    anchor wrist at (0.22, 0)
  }
  rod conrod { length: 1.0 }
  pin conrod.A -> crank.B
  pin conrod.B -> piston.wrist
  slide piston.guide -> base.rail
  drive crank at 90 deg/s from 45 deg
}
)";

// ---- M3 fixtures --------------------------------------------------------------

const char* kPendulumDyn = R"(
mechanism pendulum_dyn {
  world { gravity: 9.81 }
  ground frame { anchor pivot at (0, 0) }
  rod arm { length: 1.0, from: -85 deg } on frame.pivot
}
)";

const char* kPendulumE = R"(
mechanism pendulum_e {
  world { gravity: 9.81 }
  ground frame { anchor pivot at (0, 0) }
  rod arm { length: 1.0, from: 60 deg } on frame.pivot
}
)";

const char* kDoublePendDyn = R"(
mechanism doublepend_dyn {
  world { gravity: 9.81 }
  ground frame { anchor pivot at (0, 0) }
  rod upper { length: 0.9, from: 80 deg } on frame.pivot
  rod lower { length: 0.7, from: -40 deg }
  pin lower.A -> upper.B
}
)";

const char* kOscillator = R"(
mechanism oscillator {
  world { gravity: 0 }
  ground frame {
    anchor rail at (0, 0) dir (1, 0)
    anchor wall at (-1.2, 0)
  }
  body cart { anchor guide at (0, 0) dir (1, 0), mass: 1 }
  slide cart.guide -> frame.rail
  spring sp frame.wall -> cart.guide { k: 9, rest: 0.7 }
}
)";

const char* kFlywheel = R"(
mechanism flywheel {
  world { gravity: 0 }
  ground frame { anchor axle at (0, 0) }
  shaft rotor at frame.axle
  gear disc { teeth: 40, module: 0.01, mass: 2 } on rotor
  drive rotor torque 0.001
}
)";

const char* kDrivenPend = R"(
mechanism drivenpend {
  world { gravity: 9.81 }
  ground frame { anchor pivot at (0, 0) }
  rod arm { length: 1.0 } on frame.pivot
  drive arm at 5 deg/s from 0 deg
}
)";

const char* kFreeFall = R"(
mechanism freefall {
  world { gravity: 9.81 }
  ground frame { anchor o at (0, 0) }
  body stone { anchor c at (0, 0), mass: 1 }
}
)";

const char* kPlanetary = R"(
mechanism planetary {
  ground frame { anchor o at (0, 0) }
  shaft sun at frame.o
  rod arm on frame.o { length: 0.15 }
  shaft pl at arm.B
  gear gs { teeth: 10, module: 0.01 } on sun
  gear gp { teeth: 20, module: 0.01 } on pl
  gear gr { teeth: 50, module: 0.01 } on frame.o
  mesh gs gp
  mesh gp gr
  drive arm at 30 deg/s
}
)";

const char* kPlanetaryDyn = R"(
mechanism planetary_dyn {
  world { gravity: 0 }
  ground frame { anchor o at (0, 0) }
  shaft sun at frame.o
  rod arm on frame.o { length: 0.15 }
  shaft pl at arm.B
  gear gs { teeth: 10, module: 0.01 } on sun
  gear gp { teeth: 20, module: 0.01 } on pl
  gear gr { teeth: 50, module: 0.01 } on frame.o
  mesh gs gp
  mesh gp gr
  drive arm torque 0.005
}
)";

// carrierless planet: the tangent mesh row IS the carrier
const char* kOrbit = R"(
mechanism orbit {
  ground frame { anchor o at (0, 0) }
  gear gr { teeth: 40, module: 0.01 } on frame.o
  body pl { anchor c at (0.15, 0) }
  gear gp { teeth: 10, module: 0.01 } on pl.c
  mesh gp gr tangent
  drive pl at 60 deg/s
}
)";

// ---- sections ----------------------------------------------------------------

void testLexer() {
    std::printf("[lexer]\n");
    std::vector<Token> t = lex("mechanism m { rod a { length: 1.5 } }");
    CHECK(t[0].type == Tok::Kw && t[0].text == "mechanism");
    CHECK(t[1].type == Tok::Ident && t[1].text == "m");
    CHECK(t[2].type == Tok::Sym && t[2].text == "{");
    CHECK(t[3].type == Tok::Kw && t[3].text == "rod");
    CHECK(t[6].type == Tok::Kw && t[6].text == "length");
    CHECK(t[8].type == Tok::Number && t[8].num == 1.5);
    CHECK(t.back().type == Tok::End);

    // M2 keywords tokenize as keywords (loop stops before the End token)
    std::vector<Token> k = lex("body shaft gear mesh slide on");
    for (size_t i = 0; k[i].type != Tok::End; ++i) {
        CHECK(k[i].type == Tok::Kw);
        CHECK(k[i].text.size() > 0);
    }
    // 'dir' is contextual (an identifier), so anchors may be named dir
    std::vector<Token> d = lex("dir");
    CHECK(d[0].type == Tok::Ident && d[0].text == "dir");

    std::vector<Token> u = lex("drive x at 5 rpm from -30 deg");
    bool sawRpm = false, sawDeg = false, sawMinus = false, saw30 = false;
    for (const Token& k : u) {
        if (k.type == Tok::UnitRpm) sawRpm = true;
        if (k.type == Tok::UnitDeg) sawDeg = true;
        if (k.type == Tok::Sym && k.text == "-") sawMinus = true;
        if (k.type == Tok::Number && k.num == 30.0) saw30 = true;
    }
    CHECK(sawRpm);
    CHECK(sawDeg);
    // unary minus is a parser concern: lexer emits Sym '-' + Number 30
    CHECK(sawMinus && saw30);

    std::vector<Token> v = lex("a -> b // trailing comment\n # hash too");
    CHECK(v[1].type == Tok::Sym && v[1].text == "->");
    CHECK(v[2].type == Tok::Ident && v[2].text == "b");

    bool threw = false;
    try { lex("what !"); } catch (const LexError&) { threw = true; }
    CHECK(threw);
}

void testParser() {
    std::printf("[parser]\n");
    Program p = parseText(kFourbar);
    CHECK(p.mechanisms.size() == 1);
    const MechanismDecl& m = p.mechanisms[0];
    CHECK(m.name == "fourbar");
    CHECK(m.bodies.size() == 4);
    CHECK(m.joints.size() == 4);
    CHECK(m.drives.size() == 1);
    CHECK(m.bodies[0].kind == BodyDecl::Kind::Ground);
    CHECK(m.bodies[0].anchors.size() == 2);
    CHECK_APPROX(m.bodies[0].anchors[1].pos.x, 3.0, 1e-12);
    CHECK_APPROX(m.bodies[1].props.at("length"), 0.8, 1e-12);
    CHECK(m.joints[1].bodyA == "coupler" && m.joints[1].anchorA == "A");
    CHECK(m.joints[1].bodyB == "crank" && m.joints[1].anchorB == "B");
    CHECK_APPROX(m.drives[0].omegaRad, deg2rad(36.0), 1e-12);
    CHECK_APPROX(m.drives[0].theta0Rad, deg2rad(60.0), 1e-12);

    // rpm conversion
    Program q = parseText("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } pin a.A -> g.p drive a at 6 rpm }");
    CHECK_APPROX(q.mechanisms[0].drives[0].omegaRad, 6.0 * kPi / 30.0, 1e-12);

    // error cases
    auto expectParseError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            parseText(src);
        } catch (const ParseError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
            CHECK(e.line >= 1);
        }
        CHECK(threw);
    };
    expectParseError("mechanism m { rod a { length: 1 } fly b {} }", "statement");
    expectParseError("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } pin a.A g.p }", "->");
    expectParseError("mechanism m { ground g { anchor p at (0,0) } rod a { width: 1 } }", "length");
    expectParseError("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } pin a.A -> g.p drive a at 5 furlongs }", "unit");
    // M4: a bare shaft parses fine; the MODEL rejects it as floating
    {
        Program sp = parseText("mechanism m { ground g { anchor p at (0,0) } shaft a }");
        CHECK(sp.mechanisms[0].bodies.size() == 2);
        bool threw = false;
        try {
            buildModel(sp.mechanisms[0]);
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find("floating") != std::string::npos);
        }
        CHECK(threw);
    }
    expectParseError("mechanism m { ground g { anchor p at (0,0) } gear q { teeth: 8 } }", "'on'");
    expectParseError("mechanism m { ground g { anchor p at (0,0) } gear q { cogs: 8 } on g }", "gear property");
}

void testParserM2() {
    std::printf("[parser M2]\n");
    Program p = parseText(kPair);
    const MechanismDecl& m = p.mechanisms[0];
    CHECK(m.name == "pair");
    CHECK(m.bodies.size() == 3);              // ground + 2 shafts
    CHECK(m.bodies[1].kind == BodyDecl::Kind::Shaft);
    CHECK(m.bodies[1].hasAttach && m.bodies[1].attachBody == "frame"
          && m.bodies[1].attachAnchor == "p1");
    CHECK(m.gears.size() == 2);
    CHECK(m.gears[0].host == "a" && m.gears[1].host == "b");
    CHECK(m.gears[0].props.at("teeth") == 8.0);
    CHECK(m.meshes.size() == 1);
    CHECK(m.meshes[0].gearA == "g1" && m.meshes[0].gearB == "g2");

    Program cs = parseText(kCrankSlider);
    const MechanismDecl& c = cs.mechanisms[0];
    CHECK(c.bodies.size() == 4);              // ground, crank(rod), piston(body), conrod
    CHECK(c.bodies[1].kind == BodyDecl::Kind::Rod);
    CHECK(c.bodies[1].hasAttach && c.bodies[1].attachBody == "base");
    CHECK(c.bodies[2].kind == BodyDecl::Kind::Free);
    CHECK(c.bodies[2].anchors.size() == 2);
    CHECK(c.bodies[2].anchors[0].hasDir);
    CHECK_APPROX(c.bodies[2].anchors[0].dir.x, 1.0, 1e-12);
    // ground rail anchor carries a direction
    const AnchorDecl& rail = c.bodies[0].anchors[1];
    CHECK(rail.name == "rail" && rail.hasDir);
    CHECK_APPROX(rail.dir.y, 0.0, 1e-12);
    // slide joint parsed with the right kind
    CHECK(c.joints.size() == 3);
    int slides = 0, pins = 0;
    for (const JointDecl& j : c.joints) {
        if (j.kind == JointDecl::Kind::Slide) ++slides;
        else ++pins;
    }
    CHECK(slides == 1 && pins == 2);

    // direction normalization happens at build, parse keeps raw values
    Program n = parseText("mechanism m { ground g { anchor r at (0,0) dir (2, 0) } }");
    CHECK_APPROX(n.mechanisms[0].bodies[0].anchors[0].dir.x, 2.0, 1e-12);
}

void testModel() {
    std::printf("[model]\n");
    Model m = buildFromText(kFourbar);
    CHECK(m.bodies[0].isGround && m.bodies[0].name == "base");
    CHECK(m.bodies.size() == 4 && m.pins.size() == 4 && m.drives.size() == 1);
    CHECK(m.bodyIndex("coupler") == 2);
    CHECK(m.anchorIndex(1, "A") == 0 && m.anchorIndex(1, "B") == 1);
    // rod anchors desugar: A at origin, B at (L, 0)
    CHECK_APPROX(m.bodies[1].anchors[1].local.x, 0.8, 1e-12);
    CHECK_APPROX(m.bodies[1].anchors[1].local.y, 0.0, 1e-12);
    CHECK(m.nq() == 9);

    DofReport d = m.dofReport();
    CHECK(d.freeDof == 1);
    CHECK(d.residualDof == 0);

    DofReport du = buildFromText(kUndriven).dofReport();
    CHECK(du.freeDof == 1 && du.residualDof == 1);

    DofReport dd = buildFromText(kDoublePend).dofReport();
    CHECK(dd.freeDof == 2 && dd.residualDof == 0);

    // semantic errors
    auto expectModelError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src);
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };
    expectModelError("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } pin a.B -> g.q }",
                     "anchor 'q'");
    expectModelError("mechanism m { ground g { anchor p at (0,0) } rod g { length: 1 } }",
                     "duplicate body name");
    expectModelError("mechanism m { ground g { anchor p at (0,0) } ground h { anchor q at (1,0) } }",
                     "ground body");
    expectModelError("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } pin a.A -> g.p drive g at 1 deg/s }",
                     "cannot drive the ground");
    expectModelError("mechanism m { ground g { anchor p at (0,0) } rod a { length: 0 } pin a.A -> g.p }",
                     "positive 'length'");
    expectModelError("mechanism m { rod a { length: 1 } }", "no ground body");
}

void testModelM2() {
    std::printf("[model M2]\n");

    // pair: shafts, gears, attach sugar desugaring
    {
        Model m = buildFromText(kPair);
        CHECK(m.bodies.size() == 3);
        CHECK(m.bodies[1].name == "a" && !m.bodies[1].isGround);
        CHECK(m.bodies[1].anchors.size() == 1);        // shaft: anchor A only
        CHECK(m.bodies[1].anchors[0].name == "A");
        CHECK(m.pins.size() == 2);                     // both from attach sugar
        CHECK(m.pins[0].bodyA == 1 && m.pins[0].anchA == 0);
        CHECK(m.pins[0].bodyB == 0 && m.pins[0].anchB == 0);
        CHECK(m.gears.size() == 2);
        CHECK(m.gears[0].body == 1 && m.gears[1].body == 2);
        CHECK_APPROX(m.gears[0].radius, 0.5 * 0.01 * 8, 1e-15);   // 0.04
        CHECK_APPROX(m.gears[1].radius, 0.5 * 0.01 * 40, 1e-15);  // 0.20
        CHECK(m.gears[0].teeth == 8 && m.gears[1].teeth == 40);
        CHECK(m.meshes.size() == 1);
        CHECK(m.nq() == 6);

        DofReport d = m.dofReport();
        CHECK(d.freeDof == 1);      // 3*2 - 2*2 - 1
        CHECK(d.residualDof == 0);

        // assembly: shafts at their anchors; tangency valid
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());
        CHECK_APPROX(q[m.bodies[1].qOffset + 1], 0.0, 1e-12);   // a at p1
        CHECK_APPROX(q[m.bodies[2].qOffset + 1], 0.24, 1e-12);  // b at p2
        CHECK(m.validateAssembly(q).empty());

        // roll phase captured from assembled angles
        const double c0 = m.gears[0].radius * q[m.bodies[1].qOffset]
                        + m.gears[1].radius * q[m.bodies[2].qOffset];
        CHECK_APPROX(m.meshes[0].rollConst, c0, 1e-15);
    }

    // geartrain: two meshes, compound gear on shaft b
    {
        Model m = buildFromText(kGeartrain);
        CHECK(m.bodies.size() == 4 && m.gears.size() == 4 && m.meshes.size() == 2);
        CHECK(m.gears[1].body == m.gears[2].body);     // g2 and g3 share shaft b
        DofReport d = m.dofReport();
        CHECK(d.freeDof == 1);      // 3*3 - 2*3 - 2
        CHECK(d.residualDof == 0);

        std::vector<double> q;
        m.assemble(q);
        CHECK(m.validateAssembly(q).empty());
    }

    // crank-slider: body anchors, slide resolution, DOF
    {
        Model m = buildFromText(kCrankSlider);
        CHECK(m.bodies.size() == 4);
        CHECK(m.pins.size() == 3);        // crank attach + conrod/crank + conrod/piston
        CHECK(m.slides.size() == 1);
        CHECK(m.slides[0].guideBody == m.bodyIndex("piston"));
        CHECK_APPROX(m.slides[0].railPos.x, 0.9, 1e-12);
        CHECK_APPROX(m.slides[0].railDir.x, 1.0, 1e-12);

        DofReport d = m.dofReport();
        CHECK(d.freeDof == 1);      // 3*3 - 2*4
        CHECK(d.residualDof == 0);

        // direction normalization
        Model n = buildFromText(
            "mechanism m { ground g { anchor r at (0,0) dir (2, 0) } "
            "body p { anchor gu at (0,0) dir (0, -3) } slide p.gu -> g.r }" );
        CHECK_APPROX(n.bodies[0].anchors[0].dirLocal.x, 1.0, 1e-12);
        CHECK_APPROX(n.bodies[1].anchors[0].dirLocal.y, -1.0, 1e-12);
    }

    // tangency violation is caught with numbers in the message
    {
        Model m = buildFromText(
            "mechanism bad {"
            "  ground f { anchor p1 at (0, 0) anchor p2 at (0.25, 0) }"
            "  shaft a at f.p1 shaft b at f.p2"
            "  gear g1 { teeth: 8 } on a gear g2 { teeth: 40 } on b"
            "  mesh g1 g2 drive a at 60 rpm }" );
        std::vector<double> q;
        m.assemble(q);
        auto errs = m.validateAssembly(q);
        CHECK(errs.size() == 1);
        CHECK(errs[0].find("center distance") != std::string::npos);
        CHECK(errs[0].find("0.24") != std::string::npos);
    }

    // M2 semantic errors
    auto expectModelError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src);
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };
    // slide rail without a direction
    expectModelError(
        "mechanism m { ground g { anchor r at (0,0) } "
        "body p { anchor gu at (0,0) dir (1,0) } slide p.gu -> g.r }",
        "needs a direction");
    // slide guide without a direction
    expectModelError(
        "mechanism m { ground g { anchor r at (0,0) dir (1,0) } "
        "body p { anchor gu at (0,0) } slide p.gu -> g.r }",
        "needs a direction");
    // slide between two moving bodies (no ground rail)
    expectModelError(
        "mechanism m { ground g { anchor r at (0,0) } "
        "body p { anchor gu at (0,0) dir (1,0) } body q { anchor ra at (0,0) dir (1,0) } "
        "slide p.gu -> q.ra }",
        "exactly one moving body");
    // gear on ground is legal in M3 (fixed rings); meshing two of them is not
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } gear q { teeth: 8 } on g.p }");
        CHECK(m.gears[0].body == 0);   // hosted on ground: a fixed ring
        CHECK(m.gears[0].hostAnchor == 0);
    }
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } gear q { teeth: 8 } on g }",
        "needs an explicit anchor");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q1 { teeth: 8 } on g.p gear q2 { teeth: 8 } on g.p mesh q1 q2 }",
        "two gears on ground");
    // teeth and radius together
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q { teeth: 8 radius: 0.2 } on s }",
        "both 'teeth' and 'radius'");
    // neither teeth nor radius
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q { module: 0.01 } on s }",
        "needs 'teeth'");
    // mesh with unknown gear
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q { teeth: 8 } on s mesh q nope }",
        "unknown gear");
    // module mismatch across a mesh
    expectModelError(
        "mechanism m { ground g { anchor p1 at (0,0) anchor p2 at (0.2,0) } "
        "shaft a at g.p1 shaft b at g.p2 "
        "gear g1 { teeth: 8, module: 0.01 } on a gear g2 { teeth: 8, module: 0.02 } on b "
        "mesh g1 g2 }",
        "share a module");
    // driving a gear gets a helpful redirect
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q { teeth: 8 } on s drive q at 60 rpm }",
        "is a gear");
    // duplicate gear name
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q { teeth: 8 } on s gear q { teeth: 9 } on s }",
        "duplicate gear name");
    // zero-length direction
    expectModelError(
        "mechanism m { ground g { anchor r at (0,0) dir (0, 0) } }",
        "zero-length direction");
}

void testKinematics() {
    std::printf("[kinematics]\n");

    // pendulum: single driven body, exact analytic answer
    {
        Model m = buildFromText(kPendulum);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK_APPROX(rad2deg(q[0]), -60.0, 1e-9);
        for (double t : {0.5, 1.0, 2.0, 3.75}) {
            CHECK(sys.solve(q, t, iters, &res));
            CHECK(res < 1e-9);
            double want = deg2rad(-60.0 + 40.0 * t);
            CHECK_APPROX(angDiff(q[0], want), 0.0, 1e-9);
            // anchor A stays pinned at the origin
            Vec2 a = m.worldAnchor(1, 0, q);
            CHECK_APPROX(a.x, 0.0, 1e-9);
            CHECK_APPROX(a.y, 0.0, 1e-9);
        }
    }

    // four-bar: closed loop, verify constraints and rod lengths over time
    {
        Model m = buildFromText(kFourbar);
        ConstraintSystem sys(m);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);

        const double dt = 1.0 / 60.0;
        double worstPin = 0, worstLen = 0;
        for (int frame = 1; frame <= 240; ++frame) {   // 4 seconds
            double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worstPin = std::max(worstPin, res);
            // rod length invariants
            for (size_t bi = 1; bi < m.bodies.size(); ++bi) {
                Vec2 pa = m.worldAnchor((int)bi, 0, q);
                Vec2 pb = m.worldAnchor((int)bi, 1, q);
                worstLen = std::max(worstLen, std::fabs(vlen(pb - pa) - m.bodies[bi].length));
            }
            // drive honored
            double want = deg2rad(60.0 + 36.0 * t);
            double got = q[m.bodies[1].qOffset];
            worstPin = std::max(worstPin, std::fabs(angDiff(got, want)));
        }
        CHECK(worstPin < 1e-8);
        CHECK(worstLen < 1e-8);
    }

    // undriven four-bar: solvable, but frozen (min-norm holds free DOF)
    {
        Model m = buildFromText(kUndriven);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);
        double theta0 = q[m.bodies[1].qOffset];
        std::vector<double> q2 = q;
        CHECK(sys.solve(q2, 5.0, iters, &res));
        CHECK(res < 1e-9);
        CHECK_APPROX(angDiff(q2[m.bodies[1].qOffset], theta0), 0.0, 1e-6);
    }

    // double pendulum: both drives honored simultaneously
    {
        Model m = buildFromText(kDoublePend);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        double t = 1.3;
        CHECK(sys.solve(q, t, iters, &res));
        CHECK(res < 1e-9);
        CHECK_APPROX(angDiff(q[0], deg2rad(80.0 + 25.0 * t)), 0.0, 1e-9);
        CHECK_APPROX(angDiff(q[3], deg2rad(-40.0 - 47.0 * t)), 0.0, 1e-9);
    }

    // cold-start robustness: perturb everything, let LM pull it back
    {
        Model m = buildFromText(kFourbar);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        for (double& v : q) v += 0.21;
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-8);
    }

    // disconnected body produces a warning and stays solvable
    {
        const char* src = R"(
mechanism stray {
  ground g { anchor p at (0, 0) }
  rod a { length: 1.0 }
  rod b { length: 1.0 }
  pin a.A -> g.p
  drive a at 10 deg/s
}
)";
        Model m = buildFromText(src);
        CHECK(m.bodies.size() == 3);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.size() == 1);
        CHECK(warns[0].find("'b'") != std::string::npos);
        ConstraintSystem sys(m);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 1.0, iters, &res));
        CHECK(res < 1e-9);
    }
}

void testKinematicsM2() {
    std::printf("[kinematics M2]\n");

    // gear pair: exact ratio and reversal over time
    {
        Model m = buildFromText(kPair);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);

        const int ia = m.bodyIndex("a");
        const int ib = m.bodyIndex("b");
        const double rA = m.gears[0].radius, rB = m.gears[1].radius;
        const double thA0 = q[m.bodies[ia].qOffset];
        const double thB0 = q[m.bodies[ib].qOffset];

        const double wA = 60.0 * kPi / 30.0;    // 60 rpm -> 2*pi rad/s
        const double dt = 1.0 / 60.0;
        double worst = 0;
        for (int frame = 1; frame <= 180; ++frame) {   // 3 seconds
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worst = std::max(worst, res);
            // drive honored on a
            worst = std::max(worst, std::fabs(angDiff(q[m.bodies[ia].qOffset], wA * t)));
            // mesh: rA*(thA - thA0) + rB*(thB - thB0) = 0
            const double roll = rA * (q[m.bodies[ia].qOffset] - thA0)
                              + rB * (q[m.bodies[ib].qOffset] - thB0);
            worst = std::max(worst, std::fabs(roll));
            // shafts stay on their anchors
            const Vec2 ca{q[m.bodies[ia].qOffset + 1], q[m.bodies[ia].qOffset + 2]};
            const Vec2 cb{q[m.bodies[ib].qOffset + 1], q[m.bodies[ib].qOffset + 2]};
            worst = std::max(worst, std::fabs(ca.x) + std::fabs(cb.x - 0.24));
        }
        CHECK(worst < 1e-9);
        // b turns at exactly rA/rB = 1/5 of a in magnitude: 60 rpm in -> 12 rpm out
        // (reversal is already proven by the roll equation above)
        CHECK_APPROX(wA * rA / rB, 60.0 * kPi / 30.0 / 5.0, 1e-12);
    }

    // geartrain: two stages compound, output = input * (r1/r2) * (r3/r4)
    {
        Model m = buildFromText(kGeartrain);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));

        const int ia = m.bodyIndex("a");
        const int ic = m.bodyIndex("c");
        const double thA0 = q[m.bodies[ia].qOffset];
        const double thC0 = q[m.bodies[ic].qOffset];
        const double wA = 2.0 * kPi;             // 60 rpm
        const double ratio = (0.04 / 0.2) * (0.04 / 0.2);   // 1/25, two ext meshes

        const double dt = 1.0 / 60.0;
        double worst = 0;
        for (int frame = 1; frame <= 240; ++frame) {   // 4 seconds
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worst = std::max(worst, res);
            const double want = thC0 + ratio * (wA * t - thA0);
            worst = std::max(worst, std::fabs(angDiff(q[m.bodies[ic].qOffset], want)));
        }
        CHECK(worst < 1e-9);
        // roll constant is invariant under the solver (to solver precision)
        const int ib = m.bodyIndex("b");
        CHECK_APPROX(m.meshes[0].rollConst,
                     m.gears[0].radius * q[m.bodies[ia].qOffset]
                     + m.gears[1].radius * q[m.bodies[ib].qOffset], 1e-9);
    }

    // crank-slider: slider stays on the rail, orientation locked, lengths held
    {
        Model m = buildFromText(kCrankSlider);
        ConstraintSystem sys(m);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);

        const int icrank = m.bodyIndex("crank");
        const int iconn = m.bodyIndex("conrod");
        const int ipist = m.bodyIndex("piston");

        const double dt = 1.0 / 60.0;
        double worstRail = 0, worstLen = 0, worstDrive = 0;
        for (int frame = 1; frame <= 240; ++frame) {   // 4 seconds = 1.5 cycles
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            // piston orientation parallel to rail (0 or pi; stays near 0)
            worstRail = std::max(worstRail, std::fabs(std::sin(q[m.bodies[ipist].qOffset])));
            // guide anchor rides the rail (y = 0)
            const Vec2 gu = m.worldAnchor(ipist, 0, q);
            worstRail = std::max(worstRail, std::fabs(gu.y));
            // rod length invariants
            for (int bi : {icrank, iconn}) {
                const Vec2 pa = m.worldAnchor(bi, 0, q);
                const Vec2 pb = m.worldAnchor(bi, 1, q);
                worstLen = std::max(worstLen,
                                    std::fabs(vlen(pb - pa) - m.bodies[bi].length));
            }
            // crank pinned at origin and driven
            const Vec2 ca = m.worldAnchor(icrank, 0, q);
            worstDrive = std::max(worstDrive, std::fabs(ca.x) + std::fabs(ca.y));
            const double want = deg2rad(45.0 + 90.0 * t);
            worstDrive = std::max(worstDrive,
                                  std::fabs(angDiff(q[m.bodies[icrank].qOffset], want)));
        }
        CHECK(worstRail < 1e-9);
        CHECK(worstLen < 1e-9);
        CHECK(worstDrive < 1e-9);

        // stroke amplitude: wrist x range = crank length * 2 = 0.6 (+-)
        double minx = 1e30, maxx = -1e30;
        for (int frame = 0; frame <= 240; ++frame) {
            const double t = frame * dt;
            sys.solve(q, t, iters);
            const Vec2 wr = m.worldAnchor(ipist, 1, q);
            minx = std::min(minx, wr.x);
            maxx = std::max(maxx, wr.x);
        }
        CHECK_APPROX(maxx - minx, 0.6, 1e-6);
    }

    // cold-start robustness: perturbed geartrain still converges
    {
        Model m = buildFromText(kGeartrain);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        for (double& v : q) v += 0.17;
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-8);
    }
}

void testParserM3() {
    std::printf("[parser M3]\n");

    auto expectParseError3 = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            parseText(src);
        } catch (const ParseError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };

    // world block
    {
        Program p = parseText(
            "mechanism m { world { gravity: 3.7, step: 0.001 } "
            "ground g { anchor p at (0,0) } }");
        const WorldDecl& w = p.mechanisms[0].world;
        CHECK(w.present);
        CHECK_APPROX(w.gravity, 3.7, 1e-12);
        CHECK_APPROX(w.step, 0.001, 1e-12);
    }
    {
        Program p = parseText("mechanism m { world { } ground g { anchor p at (0,0) } }");
        CHECK(p.mechanisms[0].world.present);
        CHECK_APPROX(p.mechanisms[0].world.gravity, 9.81, 1e-12);
        CHECK_APPROX(p.mechanisms[0].world.step, 0.002, 1e-12);
    }
    expectParseError3(
        "mechanism m { world { gravity: 1, gravity: 2 } ground g { anchor p at (0,0) } }",
        "duplicate world property");
    expectParseError3(
        "mechanism m { world { density: 1 } ground g { anchor p at (0,0) } }",
        "world property");
    expectParseError3(
        "mechanism m { world { step: } ground g { anchor p at (0,0) } }",
        "expected number");

    // torque drive
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
            "drive a torque -0.5 }");
        const DriveDecl& d = p.mechanisms[0].drives[0];
        CHECK(d.isTorque);
        CHECK_APPROX(d.torque, -0.5, 1e-12);
        CHECK(!d.hasFrom);
    }
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
            "drive a at 60 rpm }");
        CHECK(!p.mechanisms[0].drives[0].isTorque);
    }
    expectParseError3(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "drive a torque }",
        "expected number");

    // mesh tangent flag (contextual identifier, not a keyword)
    {
        std::vector<Token> t = lex("tangent gravity step mass k rest damp");
        for (size_t i = 0; i < 7; ++i) CHECK(t[i].type == Tok::Ident);
        std::vector<Token> w = lex("world spring torque");
        for (size_t i = 0; i < 3; ++i) CHECK(w[i].type == Tok::Kw);
    }
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "gear g1 { teeth: 8 } on s gear g2 { teeth: 8 } on s mesh g1 g2 tangent }");
        CHECK(p.mechanisms[0].meshes[0].tangent);
    }
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "gear g1 { teeth: 8 } on s gear g2 { teeth: 8 } on s mesh g1 g2 }");
        CHECK(!p.mechanisms[0].meshes[0].tangent);
    }
    expectParseError3(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear g1 { teeth: 8 } on s gear g2 { teeth: 8 } on s mesh g1 g2 bogus }",
        "tangent");

    // gear host anchor
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "body q { anchor c at (1,2) } "
            "gear a { teeth: 8 } on s gear b { teeth: 8 } on q.c }");
        CHECK(p.mechanisms[0].gears[0].host == "s");
        CHECK(p.mechanisms[0].gears[0].hostAnchor.empty());
        CHECK(p.mechanisms[0].gears[1].host == "q");
        CHECK(p.mechanisms[0].gears[1].hostAnchor == "c");
    }

    // spring
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
            "spring sp a.B -> g.p { k: 9, rest: 1.2, damp: 0.1 } }");
        const SpringDecl& s = p.mechanisms[0].springs[0];
        CHECK(s.name == "sp");
        CHECK(s.bodyA == "a" && s.anchorA == "B");
        CHECK(s.bodyB == "g" && s.anchorB == "p");
        CHECK_APPROX(s.props.at("k"), 9.0, 1e-12);
        CHECK_APPROX(s.props.at("rest"), 1.2, 1e-12);
        CHECK_APPROX(s.props.at("damp"), 0.1, 1e-12);
    }
    expectParseError3(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B -> g.p { k: 9, stiff: 1 } }",
        "unknown spring property");
    expectParseError3(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B g.p { k: 9 } }",
        "'->'");

    // rod props: from + mass
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } "
            "rod a { length: 2, mass: 5, from: 30 deg } }");
        const BodyDecl& b = p.mechanisms[0].bodies[1];
        CHECK_APPROX(b.props.at("length"), 2.0, 1e-12);
        CHECK_APPROX(b.props.at("mass"), 5.0, 1e-12);
        CHECK_APPROX(b.props.at("from"), 30.0, 1e-12);
    }
    expectParseError3(
        "mechanism m { ground g { anchor p at (0,0) } rod a { from: 30 } }",
        "'deg'");

    // free-body mass property
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } "
            "body p { anchor c at (0,0), mass: 1.5 } }");
        CHECK_APPROX(p.mechanisms[0].bodies[1].props.at("mass"), 1.5, 1e-12);
    }
}

void testModelM3() {
    std::printf("[model M3]\n");

    auto expectModelError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src);
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };

    // dynamics activation: world block / spring / torque
    CHECK(!buildFromText(kPair).world.dynamic);
    CHECK(!buildFromText(kFourbar).world.dynamic);
    CHECK(buildFromText(kPendulumDyn).world.dynamic);
    CHECK(buildFromText(kOscillator).world.dynamic);
    CHECK(buildFromText(kFlywheel).world.dynamic);

    // world parameters are stored and validated
    {
        Model m = buildFromText(
            "mechanism m { world { gravity: 3.7, step: 0.001 } "
            "ground g { anchor p at (0,0) } }");
        CHECK_APPROX(m.world.gravity, 3.7, 1e-12);
        CHECK_APPROX(m.world.step, 0.001, 1e-12);
    }
    expectModelError(
        "mechanism m { world { step: 0 } ground g { anchor p at (0,0) } }",
        "'step'");
    expectModelError(
        "mechanism m { world { step: 1 } ground g { anchor p at (0,0) } }",
        "'step'");
    expectModelError(
        "mechanism m { world { gravity: -1 } ground g { anchor p at (0,0) } }",
        "'gravity'");

    // gear host anchors resolve
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "body q { anchor c at (1,2) } "
            "gear a { teeth: 8 } on s gear b { teeth: 8 } on q.c }");
        CHECK(m.gears[0].hostAnchor == 0);
        const int iq = m.bodyIndex("q");
        CHECK(m.gears[1].hostAnchor == m.anchorIndex(iq, "c"));
        CHECK(m.gears[1].body == iq);
    }
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "body q { anchor c at (1,2) } gear b { teeth: 8 } on q.nope }",
        "no anchor");
    // coaxial gears cannot mesh
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear a { teeth: 8 } on s gear b { teeth: 8 } on s mesh a b }",
        "coaxial");

    // springs: defaults, capture, validation
    {
        Model m = buildFromText(kOscillator);
        CHECK(m.springs.size() == 1);
        CHECK_APPROX(m.springs[0].k, 9.0, 1e-12);
        CHECK_APPROX(m.springs[0].rest, 0.7, 1e-12);
        CHECK_APPROX(m.springs[0].damp, 0.0, 1e-12);
        CHECK(m.springs[0].name == "sp");
    }
    {
        // rest captured at assembly when not given (rod points at 90 deg,
        // so B sits at (0, 1) in the world)
        Model m = buildFromText(
            "mechanism m { world { gravity: 0 } "
            "ground g { anchor p at (0, 0) anchor q at (3, 4) } "
            "rod a { length: 1, from: 90 deg } on g.p "
            "spring sp a.B -> g.q { k: 9 } }");
        std::vector<double> q;
        m.assemble(q);
        CHECK_APPROX(m.springs[0].rest, vlen(Vec2{3.0, 4.0} - Vec2{0.0, 1.0}), 1e-9);
    }
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B -> g.p { rest: 1 } }",
        "positive 'k'");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B -> g.nope { k: 1 } }",
        "no anchor");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B -> g.p { k: 1, damp: -1 } }",
        "'damp'");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B -> g.p { k: 1, rest: -1 } }",
        "'rest'");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
        "spring sp a.B -> g.p { k: 1 } spring sp a.B -> g.p { k: 1 } }",
        "duplicate spring");

    // torque drives
    {
        Model m = buildFromText(kFlywheel);
        CHECK(m.torques.size() == 1);
        CHECK(m.drives.empty());
        CHECK(m.torques[0].body == m.bodyIndex("rotor"));
        CHECK_APPROX(m.torques[0].torque, 0.001, 1e-15);
    }
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } on g.p "
        "drive a at 1 deg/s drive a torque 0.1 }",
        "driven more than once");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } drive g torque 0.1 }",
        "ground");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "gear q { teeth: 8 } on s drive q torque 0.1 }",
        "is a gear");

    // mass properties: rod auto-derived
    {
        Model m = buildFromText(kPendulumDyn);
        const MassProps& mp = m.mass[1];
        CHECK_APPROX(mp.mass, 1.0, 1e-12);           // density 1 kg/m, L = 1
        CHECK_APPROX(mp.com.x, 0.5, 1e-12);
        CHECK_APPROX(mp.com.y, 0.0, 1e-12);
        CHECK_APPROX(mp.inertia, 1.0 / 12.0, 1e-12); // m L^2 / 12 about com
    }
    // shaft + gear disc combine
    {
        Model m = buildFromText(kFlywheel);
        const MassProps& mp = m.mass[1];
        const double disc = 2.0;                          // explicit mass
        CHECK_APPROX(mp.mass, 0.05 + disc, 1e-12);
        CHECK_APPROX(mp.com.x, 0.0, 1e-12);
        CHECK_APPROX(mp.inertia, 1.0e-4 + 0.5 * disc * 0.2 * 0.2, 1e-12);
    }
    // compound gears on one shaft
    {
        Model m = buildFromText(
            "mechanism m { world { gravity: 1 } ground g { anchor p at (0,0) } "
            "shaft s at g.p "
            "gear g1 { teeth: 8 } on s gear g2 { teeth: 40 } on s }");
        const MassProps& mp = m.mass[1];
        const double d1 = kPi * 0.04 * 0.04;
        const double d2 = kPi * 0.2 * 0.2;
        CHECK_APPROX(mp.mass, 0.05 + d1 + d2, 1e-12);
        CHECK_APPROX(mp.inertia,
                     1.0e-4 + 0.5 * d1 * 0.04 * 0.04 + 0.5 * d2 * 0.2 * 0.2, 1e-12);
    }
    // explicit rod mass overrides the geometric default
    {
        Model m = buildFromText(
            "mechanism m { world { gravity: 1 } ground g { anchor p at (0,0) } "
            "rod a { length: 2, mass: 5 } on g.p }");
        const MassProps& mp = m.mass[1];
        CHECK_APPROX(mp.mass, 5.0, 1e-12);
        CHECK_APPROX(mp.com.x, 1.0, 1e-12);
        CHECK_APPROX(mp.inertia, 5.0 * 4.0 / 12.0, 1e-12);
    }
    // free bodies default to 1 kg at the frame origin
    {
        Model m = buildFromText(kFreeFall);
        const MassProps& mp = m.mass[1];
        CHECK_APPROX(mp.mass, 1.0, 1e-12);
        CHECK_APPROX(mp.com.x, 0.0, 1e-12);
        CHECK(mp.inertia > 0.0);
    }
    // kinematic models carry no mass at all
    {
        Model m = buildFromText(kPair);
        CHECK(m.mass.size() == m.bodies.size());
        CHECK(m.mass[1].mass == 0.0);
    }
    // zero mass is rejected in dynamic mode
    expectModelError(
        "mechanism m { world { gravity: 1 } ground g { anchor p at (0,0) } "
        "rod a { length: 2, mass: 0 } on g.p }",
        "zero mass");

    // release-angle sugar
    {
        Model m = buildFromText(kPendulumDyn);
        CHECK(m.bodies[1].hasSeed);
        CHECK_APPROX(m.bodies[1].seedAngle, deg2rad(-85.0), 1e-12);
        std::vector<double> q;
        m.assemble(q);
        CHECK_APPROX(q[m.bodies[1].qOffset], deg2rad(-85.0), 1e-12);
    }

    // DOF bookkeeping with tangent meshes
    {
        Model m = buildFromText(kOrbit);
        DofReport d = m.dofReport();
        CHECK(d.nTangentMeshes == 1);
        CHECK(d.freeDof == 1);
        CHECK(d.residualDof == 0);
    }
    {
        Model m = buildFromText(kPlanetary);
        DofReport d = m.dofReport();
        CHECK(d.nTangentMeshes == 0);
        CHECK(d.nMeshes == 2);
        CHECK(d.freeDof == 1);      // 3 bodies, 3 pins, 2 meshes: 9 - 6 - 2
        CHECK(d.residualDof == 0);
    }

    // planetary assembly detects external and internal meshes
    {
        Model m = buildFromText(kPlanetary);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());
        CHECK(m.meshes[0].sgn == 1);    // sun <-> planet: external
        CHECK(m.meshes[1].sgn == -1);   // planet <-> ring: internal
        CHECK(m.validateAssembly(q).empty());
    }
    // internal tangency violation reports the internal numbers
    {
        Model m = buildFromText(
            "mechanism bad { ground f { anchor o at (0, 0) } "
            "gear gr { teeth: 40, module: 0.01 } on f.o "
            "body pl { anchor c at (0.18, 0) } "
            "gear gp { teeth: 10, module: 0.01 } on pl.c "
            "mesh gp gr tangent drive pl at 10 deg/s }");
        std::vector<double> q;
        m.assemble(q);
        CHECK(m.meshes[0].sgn == -1);   // closer to internal than external
        auto errs = m.validateAssembly(q);
        CHECK(errs.size() == 1);
        CHECK(errs[0].find("internal") != std::string::npos);
    }
}

void testKinematicsM3() {
    std::printf("[kinematics M3]\n");

    // planetary: sun turns exactly 6x the carrier, psi winds without branch
    // cuts, and the carrier holds the planet at the meshing distance
    {
        Model m = buildFromText(kPlanetary);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);
        const int iarm = m.bodyIndex("arm");
        const int isun = m.bodyIndex("sun");
        const int ipl = m.bodyIndex("pl");
        const double sun0 = q[m.bodies[isun].qOffset];
        const double wArm = deg2rad(30.0);
        const double dt = 1.0 / 60.0;
        double worst = 0;
        for (int frame = 1; frame <= 2400; ++frame) {   // 40 s: carrier 3.33 turns
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worst = std::max(worst, res);
            // carrier driven exactly
            worst = std::max(worst, std::fabs(q[m.bodies[iarm].qOffset] - wArm * t));
            // sun = 6x carrier (absolute, unwrapped angles)
            worst = std::max(worst,
                std::fabs(q[m.bodies[isun].qOffset] - sun0 - 6.0 * wArm * t));
            // planet rides the carrier arm at pitch distance
            const Vec2 cpl{q[m.bodies[ipl].qOffset + 1], q[m.bodies[ipl].qOffset + 2]};
            worst = std::max(worst, std::fabs(vlen(cpl) - 0.15));
        }
        CHECK(worst < 1e-9);
        // the tracked center-line angle wound through multiple revolutions
        CHECK(std::fabs(m.meshes[0].psiTrack) > 4.0 * kPi);
    }

    // carrierless planet rolling inside a fixed ring: the tangent mesh is
    // the whole joint — no pins, no carrier, pure constraint physics
    {
        Model m = buildFromText(kOrbit);
        ConstraintSystem sys(m);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);
        const int ipl = m.bodyIndex("pl");
        const int igp = m.gearIndex("gp");
        const double wP = deg2rad(60.0);
        const double dt = 1.0 / 60.0;
        double worst = 0;
        for (int frame = 1; frame <= 360; ++frame) {   // 6 s
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worst = std::max(worst, res);
            const double thP = q[m.bodies[ipl].qOffset];
            worst = std::max(worst, std::fabs(thP - wP * t));
            const Vec2 c = m.gearCenter(igp, q);   // planet pitch-circle center
            // stays in contact with the ring: |c| = 0.2 - 0.05
            worst = std::max(worst, std::fabs(vlen(c) - 0.15));
            // rolling inside: psi = -rP/(rR-rP) * theta
            const double psi = std::atan2(c.y, c.x);
            worst = std::max(worst, std::fabs(angDiff(psi, -wP * t / 3.0)));
        }
        CHECK(worst < 1e-9);
    }

    // consistent redundancy: sun + ring + free planet with two tangent
    // meshes is over-constrained per Grubler but geometrically consistent;
    // the damped solver must still hold every invariant
    {
        Model m = buildFromText(
            "mechanism tightsystem {"
            "  ground frame { anchor o at (0, 0) }"
            "  shaft sun at frame.o"
            "  gear gs { teeth: 20, module: 0.01 } on sun"
            "  gear gr { teeth: 40, module: 0.01 } on frame.o"
            "  body pl { anchor c at (0.15, 0) }"
            "  gear gp { teeth: 10, module: 0.01 } on pl.c"
            "  mesh gs gp tangent"
            "  mesh gp gr tangent"
            "  drive sun at 30 deg/s }");
        DofReport d = m.dofReport();
        CHECK(d.residualDof == -1);          // flagged, but consistent
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        const int igp = m.gearIndex("gp");
        const double wS = deg2rad(30.0);
        const double dt = 1.0 / 60.0;
        double worst = 0;
        for (int frame = 1; frame <= 240; ++frame) {   // 4 s
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worst = std::max(worst, res);
            const Vec2 c = m.gearCenter(igp, q);
            worst = std::max(worst, std::fabs(vlen(c) - 0.15));
            // 0.1*thS - 0.3*psi = const -> psi = wS*t/3
            const double psi = std::atan2(c.y, c.x);
            worst = std::max(worst, std::fabs(angDiff(psi, wS * t / 3.0)));
        }
        CHECK(worst < 1e-8);
    }

    // moving-center mesh with radius-specified gears still rolls correctly
    {
        Model m = buildFromText(
            "mechanism rolling {"
            "  ground frame { anchor o at (0, 0) }"
            "  gear gr { radius: 0.2 } on frame.o"
            "  body pl { anchor c at (0.15, 0) }"
            "  gear gp { radius: 0.05 } on pl.c"
            "  mesh gp gr tangent"
            "  drive pl at 60 deg/s }");
        CHECK(m.gears[0].teeth == 0);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        const int igp = m.gearIndex("gp");
        const double wP = deg2rad(60.0);
        const double dt = 1.0 / 60.0;
        double worst = 0;
        for (int frame = 1; frame <= 180; ++frame) {
            const double t = frame * dt;
            CHECK(sys.solve(q, t, iters, &res));
            worst = std::max(worst, res);
            const Vec2 c = m.gearCenter(igp, q);
            worst = std::max(worst, std::fabs(vlen(c) - 0.15));
            const double psi = std::atan2(c.y, c.x);
            worst = std::max(worst, std::fabs(angDiff(psi, -wP * t / 3.0)));
        }
        CHECK(worst < 1e-9);
    }
}

void testDynamics() {
    std::printf("[dynamics]\n");

    // free fall: exact semi-implicit trajectory
    {
        Model m = buildFromText(kFreeFall);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());            // dynamic mode: free bodies are legal
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(2.0 / h);
        for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
        const double t = steps * h;
        const int o = m.bodies[1].qOffset;
        CHECK(std::fabs(qd[o + 1]) < 1e-12);   // no horizontal drift
        CHECK(std::fabs(q[o]) < 1e-12);        // no rotation picked up
        // semi-implicit gravity: |y + g t^2/2| = g t h / 2 exactly
        CHECK(std::fabs(q[o + 2] + 0.5 * 9.81 * t * t) <= 0.5 * 9.81 * h * t + 1e-9);
    }

    // compound pendulum period: T = 2 pi sqrt(2L / (3g))
    {
        Model m = buildFromText(kPendulumDyn);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(4.5 / h);
        const int o = m.bodies[1].qOffset;
        double prev = q[o] + 0.5 * kPi;    // angle about the hanging position
        double prevT = 0.0;
        std::vector<double> crossings;
        for (long k = 1; k <= steps; ++k) {
            dyn.step(q, qd, (k - 1) * h);
            const double cur = q[o] + 0.5 * kPi;
            if ((prev < 0.0) != (cur < 0.0)) {
                const double f = prev / (prev - cur);
                crossings.push_back(prevT + f * h);
            }
            prev = cur;
            prevT = k * h;
        }
        CHECK(crossings.size() >= 5);      // ~2.75 periods in 4.5 s
        double T = 0.0;
        int nT = 0;
        for (size_t i = 2; i + 2 < crossings.size(); ++i) {
            T += crossings[i] - crossings[i - 2];
            ++nT;
        }
        T /= std::max(1, nT);
        const double TTheory = 2.0 * kPi * std::sqrt(2.0 / (3.0 * 9.81));
        CHECK(std::fabs(T - TTheory) < 0.01);
    }

    // pendulum energy conservation: 60-degree release, 10 seconds. RK4 on
    // the exact constrained acceleration keeps energy flat to print precision.
    {
        Model m = buildFromText(kPendulumE);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(10.0 / h);
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        double worst = 0;
        for (long k = 0; k < steps; ++k) {
            dyn.step(q, qd, k * h);
            const double E = dyn.kinetic(q, qd) + dyn.potential(q);
            worst = std::max(worst, std::fabs(E - E0));
            CHECK(sys.maxResidual(q, (k + 1) * h) < 1e-8);
        }
        CHECK(worst < 1e-3);
    }

    // chaotic double pendulum: wild trajectory, machine-flat energy
    {
        Model m = buildFromText(kDoublePendDyn);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(5.0 / h);
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        double worst = 0;
        for (long k = 0; k < steps; ++k) {
            dyn.step(q, qd, k * h);
            const double E = dyn.kinetic(q, qd) + dyn.potential(q);
            worst = std::max(worst, std::fabs(E - E0));
            CHECK(sys.maxResidual(q, (k + 1) * h) < 1e-8);
        }
        CHECK(worst < 1e-3);
    }

    // undriven four-bar under gravity: the closed loop swings freely. The
    // greedy assembly leaves loop residuals, so settle the initial solve
    // first (exactly what `mech run` does) before measuring the energy.
    {
        Model m = buildFromText(
            "mechanism flop { world { gravity: 9.81 } "
            "ground base { anchor p1 at (0, 0) anchor p2 at (3, 0) } "
            "rod crank { length: 0.8, from: 60 deg } on base.p1 "
            "rod coupler { length: 2.6 } "
            "rod rocker { length: 2.2, from: 30 deg } on base.p2 "
            "pin coupler.A -> crank.B pin coupler.B -> rocker.B }");
        CHECK(m.world.dynamic);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        CHECK(sys.solve(q, 0.0, iters));
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(3.0 / h);
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        double worstE = 0, worstRes = 0;
        for (long k = 0; k < steps; ++k) {
            dyn.step(q, qd, k * h);
            const double E = dyn.kinetic(q, qd) + dyn.potential(q);
            worstE = std::max(worstE, std::fabs(E - E0));
            worstRes = std::max(worstRes, sys.maxResidual(q, (k + 1) * h));
        }
        CHECK(worstRes < 1e-8);
        CHECK(worstE < 1e-3);
        // and it actually moved
        CHECK(std::fabs(qd[m.bodies[1].qOffset]) > 0.01);
    }

    // spring oscillator: pure harmonic regime, x(t) = -0.5 + 0.5 cos(3t)
    // (rest chosen so the amplitude never approaches the spring's L = 0
    // point at the wall anchor, where the idealized force flips direction)
    {
        Model m = buildFromText(kOscillator);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        std::vector<std::string> warns = m.assemble(q);
        CHECK(warns.empty());
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const int o = m.bodies[1].qOffset;
        const double w = 3.0;    // sqrt(k/m)
        long total = 0;
        for (double ts : {0.5, 1.0, 1.5, 2.0, 3.0}) {
            const long target = (long)std::llround(ts / h);
            while (total < target) { dyn.step(q, qd, total * h); ++total; }
            CHECK_APPROX(q[o + 1], -0.5 + 0.5 * std::cos(w * ts), 1e-4);
            CHECK_APPROX(qd[o + 1], -0.5 * w * std::sin(w * ts), 1e-3);
        }
        const double E = dyn.kinetic(q, qd) + dyn.potential(q);
        CHECK(std::fabs(E - 1.125) < 1e-3);   // 1/2 k (0.5)^2
    }

    // flywheel: constant torque, analytic spin-up (the rotor's assembled
    // seed angle is arbitrary — only the delta is physical)
    {
        Model m = buildFromText(kFlywheel);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const int o = m.bodies[1].qOffset;
        const double th0 = q[o];
        const long steps = (long)std::llround(4.0 / h);
        for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
        const double I = m.mass[1].inertia;   // com at the origin
        const double alpha = 0.001 / I;
        CHECK_APPROX(qd[o], alpha * 4.0, 1e-6);
        CHECK_APPROX(q[o] - th0, 0.5 * alpha * 16.0, 1e-4);
        CHECK(std::fabs(q[o + 1]) + std::fabs(q[o + 2]) < 1e-12);
    }

    // rate-driven pendulum under gravity: the drive row's reaction torque
    // equals the gravity torque it must transmit
    {
        Model m = buildFromText(kDrivenPend);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const double w = deg2rad(5.0);
        const int o = m.bodies[1].qOffset;
        long sampled = 0;
        double worst = 0;
        const long steps = (long)std::llround(2.0 / h);
        for (long k = 0; k < steps; ++k) {
            dyn.step(q, qd, k * h);
            CHECK(std::fabs(qd[o] - w) < 1e-9);   // velocity projection holds
            const double t = (k + 1) * h;
            if (t > 0.5 && k % 250 == 0) {
                const double want = 9.81 * 0.5 * std::cos(w * t);
                worst = std::max(worst, std::fabs(dyn.driveReactions()[0] - want));
                ++sampled;
            }
        }
        CHECK(sampled >= 3);
        CHECK(worst < 0.05);
    }

    // planetary spun by a torque on the carrier: work-energy identity holds
    // (this exercises the moving-center mesh acceleration bias terms)
    {
        Model m = buildFromText(kPlanetaryDyn);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const int iarm = m.bodyIndex("arm");
        const double theta0 = q[m.bodies[iarm].qOffset];
        const double h = m.world.step;
        const double tau = 0.005;
        const long steps = (long)std::llround(2.0 / h);
        double worstE = 0, worstRes = 0;
        for (long k = 0; k < steps; ++k) {
            dyn.step(q, qd, k * h);
            const double E = dyn.kinetic(q, qd);           // gravity 0
            const double work = tau * (q[m.bodies[iarm].qOffset] - theta0);
            worstE = std::max(worstE, std::fabs(E - work));
            worstRes = std::max(worstRes, sys.maxResidual(q, (k + 1) * h));
        }
        CHECK(worstRes < 1e-8);
        CHECK(worstE < 2e-3);
    }
}

// ---- M4: the intent layer ------------------------------------------------------

const char* kTrainSynth = R"(
mechanism train {
  ground frame { anchor p at (0, 0) }
  shaft s1 at frame.p
  shaft s2
  shaft s3
  shaft s4
  reduce s1->s4 ratio 1:1800 via s2 s3 module: 0.006
  drive s1 at 30 rpm
}
)";

const char* kBeatPend = R"(
mechanism beatpend {
  world { gravity: 9.81 }
  ground frame { anchor pivot at (0, 0) }
  rod pend on frame.pivot { beat: 1s, from: -87 deg }
}
)";

void testParserM4() {
    std::printf("[parser M4]\n");

    // reduce, full option set, clauses in any order
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:1800 module: 0.006 spread: 30 deg same }");
        const ReduceDecl& r = p.mechanisms[0].reduces[0];
        CHECK(r.bodyA == "a" && r.bodyB == "b");
        CHECK(r.ratioNum == 1 && r.ratioDen == 1800);
        CHECK(r.dir == ReduceDecl::Dir::Same);
        CHECK(r.hasModule && r.module == 0.006);
        CHECK(r.hasSpread && r.spreadDeg == 30.0);
        CHECK(r.via.empty());
    }
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p "
            "shaft c shaft d shaft b "
            "reduce a->b ratio 60 via c, d opposite }");
        const ReduceDecl& r = p.mechanisms[0].reduces[0];
        // single-number ratios are kept as exact n/10000 fractions; the
        // model reduces them (60 = 600000/10000)
        CHECK(r.ratioNum == 600000 && r.ratioDen == 10000);
        CHECK(r.via.size() == 2 && r.via[0] == "c" && r.via[1] == "d");
        CHECK(r.dir == ReduceDecl::Dir::Opposite);
        CHECK(!r.hasModule && !r.hasSpread);
    }
    // ratio written the other way round is kept as-written (model normalizes)
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1800:1 }");
        CHECK(p.mechanisms[0].reduces[0].ratioNum == 1800);
        CHECK(p.mechanisms[0].reduces[0].ratioDen == 1);
    }
    // beat: unit forms 1s / 1 s / 0.5 sec / 2
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "rod a { beat: 1s } on g.p rod b { beat: 0.5 sec } on g.p "
            "rod c { beat: 2 s } on g.p rod d { beat: 2 } on g.p }");
        const auto& bs = p.mechanisms[0].bodies;
        CHECK_APPROX(bs[1].props.at("beat"), 1.0, 1e-12);
        CHECK_APPROX(bs[2].props.at("beat"), 0.5, 1e-12);
        CHECK_APPROX(bs[3].props.at("beat"), 2.0, 1e-12);
        CHECK_APPROX(bs[4].props.at("beat"), 2.0, 1e-12);
    }
    // hand: with block, without block, with explicit anchor
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "hand h on s { length: 0.5 } hand bare on s hand anchored on s.A }");
        const auto& hs = p.mechanisms[0].hands;
        CHECK(hs.size() == 3);
        CHECK_APPROX(hs[0].props.at("length"), 0.5, 1e-12);
        CHECK(hs[1].props.empty());
        CHECK(hs[2].hostAnchor == "A");
    }
    // dial: bare, at-form, property block (dials are anonymous, like meshes)
    {
        Program p = parseText(
            "mechanism m { ground g { anchor p at (0,0) anchor c at (1,1) } "
            "dial at g.c { radius: 0.4, major: 4, minor: 60 } dial }");
        const auto& ds = p.mechanisms[0].dials;
        CHECK(ds.size() == 2);
        CHECK(ds[0].hasAt && ds[0].atBody == "g" && ds[0].atAnchor == "c");
        CHECK_APPROX(ds[0].props.at("radius"), 0.4, 1e-12);
        CHECK(ds[0].props.at("major") == 4);
        CHECK(!ds[1].hasAt && ds[1].props.empty());
    }

    auto expectParseError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            parseText(src);
        } catch (const ParseError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
        "reduce a->b 1800 }", "ratio");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
        "reduce a->b ratio 1:2.5 }", "integer");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
        "reduce a->b ratio -5 }", "positive");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "hand h on s { width: 1 } }", "hand property");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
        "hand h s }", "'on'");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } "
        "dial { colour: 1 } }", "dial property");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } "
        "rod a { beat: 0 } on g.p }", "positive");
}

void testModelM4() {
    std::printf("[model M4]\n");

    auto expectModelError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src);
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };

    // ---- beat: the compound-pendulum length formula -------------------------
    {
        Model m = buildFromText(kBeatPend);
        const int ip = m.bodyIndex("pend");
        CHECK(ip > 0);
        CHECK(m.bodies[(size_t)ip].beatSeconds == 1.0);
        // L = 3 g b^2 / (2 pi^2)
        CHECK_APPROX(m.bodies[(size_t)ip].length,
                     3.0 * 9.81 / (2.0 * kPi * kPi), 1e-12);
        // rod anchors follow the derived length
        CHECK_APPROX(m.bodies[(size_t)ip].anchors[1].local.x,
                     m.bodies[(size_t)ip].length, 1e-12);
    }
    {
        // custom gravity participates
        Model m = buildFromText(
            "mechanism m { world { gravity: 3.7 } ground g { anchor p at (0,0) } "
            "rod pend on g.p { beat: 2s } }");
        CHECK_APPROX(m.bodies[(size_t)m.bodyIndex("pend")].length,
                     3.0 * 3.7 * 4.0 / (2.0 * kPi * kPi), 1e-12);
    }
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod a on g.p { beat: 1s } }",
        "kinematic");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } "
        "rod a on g.p { beat: 1s, length: 1 } }", "both");

    // ---- reduce: synthesis of a 1800:1 train --------------------------------
    {
        Model m = buildFromText(kTrainSynth);
        // 3 stages: 12 x 12 x 12.5 = 1800
        CHECK(m.reduceInfos.size() == 1);
        const ReduceInfo& ri = m.reduceInfos[0];
        CHECK(ri.stages.size() == 3);
        CHECK(ri.idlers == 0);
        long long num = 1, den = 1;   // exact rational product of stage ratios
        for (const auto& st : ri.stages) { num *= st.first; den *= st.second; }
        CHECK(num == 1800 * den);
        // each stage within tooth bounds; wheel on the slow side
        for (const auto& st : ri.stages) {
            CHECK(st.first >= 24 && st.first <= 100);   // wheel
            CHECK(st.second >= 8 && st.second <= 16);   // pinion
            CHECK(st.first > st.second);
        }
        CHECK(ri.opposite);   // 3 external meshes -> reversed
        // kernel objects: 6 gears, 3 meshes, 4 pins (3 auto)
        CHECK(m.gears.size() == 6);
        CHECK(m.meshes.size() == 3);
        CHECK(m.pins.size() == 4);
        CHECK(ri.placed.size() == 3);
        // pinion on the driving shaft, wheel on the driven one
        const int is1 = m.bodyIndex("s1"), is2 = m.bodyIndex("s2");
        CHECK(m.gearIndex("s1_p1") >= 0);
        CHECK(m.gearIndex("s2_w1") >= 0);
        CHECK(m.gears[(size_t)m.gearIndex("s1_p1")].body == is1);
        CHECK(m.gears[(size_t)m.gearIndex("s2_w1")].body == is2);
        // compound intermediates
        CHECK(m.gearIndex("s2_p2") >= 0 && m.gearIndex("s3_w2") >= 0);
        // assemble: every synthesized mesh exactly tangent
        std::vector<double> q;
        m.assemble(q);
        for (const MeshJoint& mm : m.meshes) {
            const Gear& ga = m.gears[(size_t)mm.gearA];
            const Gear& gb = m.gears[(size_t)mm.gearB];
            const Vec2 ca = m.gearCenter(mm.gearA, q);
            const Vec2 cb = m.gearCenter(mm.gearB, q);
            CHECK_APPROX(vlen(cb - ca), ga.radius + gb.radius, 1e-9);
        }
        CHECK(m.validateAssembly(q).empty());
        // ground gained auto anchors named after the shafts
        CHECK(m.anchorIndex(0, "s2") >= 0);
        CHECK(m.anchorIndex(0, "s3") >= 0);
        CHECK(m.anchorIndex(0, "s4") >= 0);
    }

    // no-via: intermediates are synthesized with generated names
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:60 }");
        CHECK(m.reduceInfos[0].stages.size() == 2);   // 7.5 x 8
        CHECK(m.bodyIndex("a_1") > 0);
        CHECK(m.reduceInfos[0].placed.size() == 2);   // a_1 and b
        long long num = 1, den = 1;
        for (const auto& st : m.reduceInfos[0].stages) { num *= st.first; den *= st.second; }
        CHECK(num == 60 * den);
        CHECK(!m.reduceInfos[0].opposite);            // 2 meshes -> same sense
    }
    // auto-name collision points at 'via'
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p "
        "shaft a_1 at g.p shaft b "
        "reduce a->b ratio 1:60 }", "'via'");

    // ---- direction control ---------------------------------------------------
    {
        // same: 1800 in an even stage count (4 stages), no idler
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:1800 same }");
        CHECK(m.reduceInfos[0].stages.size() == 4);
        CHECK(m.reduceInfos[0].idlers == 0);
        CHECK(!m.reduceInfos[0].opposite);
    }
    {
        // same with an unreachable parity at R=2: one idler inserted
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:2 same }");
        CHECK(m.reduceInfos[0].stages.size() == 1);
        CHECK(m.reduceInfos[0].idlers == 1);
        CHECK(!m.reduceInfos[0].opposite);   // 2 meshes -> same
        CHECK(m.bodyIndex("a_idler") > 0);
        // the idler's single gear is shared by two meshes
        int idlerGearCount = 0;
        for (const Gear& gg : m.gears)
            if (gg.body == m.bodyIndex("a_idler")) ++idlerGearCount;
        CHECK(idlerGearCount == 1);
        CHECK(m.meshes.size() == 2);
        // idler is pinned (not floating) and adds no DOF
        DofReport d = m.dofReport();
        CHECK(d.freeDof == 1);
    }
    {
        // via fixes the stage count; opposite then needs an idler (60 = 7.5 x 8
        // in exactly two stages; two meshes run same-sense, so an idler flips)
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p "
            "shaft c shaft b "
            "reduce a->b ratio 1:60 via c opposite }");
        CHECK(m.reduceInfos[0].stages.size() == 2);   // via c -> 2 stages
        CHECK(m.reduceInfos[0].idlers == 1);          // even meshes -> idler
        CHECK(m.reduceInfos[0].opposite);
    }

    // ---- ratio bounds and unreachable cases ---------------------------------
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
        "reduce a->b ratio 1:1 }", "smallest synthesizable stage");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
        "reduce a->b ratio 1:999999 }", "exceeds");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
        "reduce a->b ratio 1:1499 }", "cannot be synthesized");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p "
        "shaft c shaft b "
        "reduce a->b ratio 1:1800 via c }", "exactly 2 stages");

    // ---- endpoints and layout -----------------------------------------------
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a shaft b "
        "reduce a->b ratio 1:60 }", "pinned");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p "
        "reduce a->b ratio 1:60 }", "'b'");   // b never declared
    {
        // pinned destination: layout closes by circle intersection, exactly tangent
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) anchor out at (0.5, 0.3) } "
            "shaft a at g.p shaft b at g.out "
            "reduce a->b ratio 1:60 }");
        CHECK(m.reduceInfos[0].placed.size() == 1);   // only the intermediate
        std::vector<double> q;
        m.assemble(q);
        CHECK(m.validateAssembly(q).empty());
        for (const MeshJoint& mm : m.meshes) {
            const Gear& ga = m.gears[(size_t)mm.gearA];
            const Gear& gb = m.gears[(size_t)mm.gearB];
            CHECK_APPROX(vlen(m.gearCenter(mm.gearB, q) - m.gearCenter(mm.gearA, q)),
                         ga.radius + gb.radius, 1e-9);
        }
    }
    {
        // incompatible pinned ends are reported with numbers
        bool threw = false;
        try {
            buildFromText(
                "mechanism m { ground g { anchor p at (0,0) anchor out at (10, 0) } "
                "shaft a at g.p shaft b at g.out "
                "reduce a->b ratio 1:60 }");
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find("tangent") != std::string::npos);
        }
        CHECK(threw);
    }
    // spread controls the first stage's direction
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:60 spread: 90 deg }");
        std::vector<double> q;
        m.assemble(q);
        // the first intermediate sits due +y of the origin
        const int ia1 = m.bodyIndex("a_1");
        const Vec2 p1 = m.worldAnchor(ia1, 0, q);
        CHECK_APPROX(p1.x, 0.0, 1e-9);
        CHECK(p1.y > 0.3);
    }

    // ---- hands and dials -----------------------------------------------------
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "hand h on s { length: 0.5 } hand def on s }");
        CHECK(m.hands.size() == 2);
        CHECK_APPROX(m.hands[0].length, 0.5, 1e-12);
        CHECK_APPROX(m.hands[1].length, 0.3, 1e-12);   // default
        CHECK(m.hands[0].mountAnchor == 0);
    }
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } hand h on g { length: 1 } }",
        "never moves");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } "
        "dial { radius: 1 } }", "no hands");
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p hand h on s "
        "dial { minor: 50, major: 12 } }", "multiple");
    {
        // dial captures center and radius at assembly
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft s at g.p "
            "hand h on s { length: 0.5 } dial }");
        std::vector<double> q;
        m.assemble(q);
        CHECK(m.dials.size() == 1);
        CHECK_APPROX(m.dials[0].center.x, 0.0, 1e-12);
        CHECK_APPROX(m.dials[0].center.y, 0.0, 1e-12);
        CHECK_APPROX(m.dials[0].radius, 1.2 * 0.5, 1e-12);
        CHECK(m.dials[0].major == 12 && m.dials[0].minor == 60);
    }
    {
        // explicit dial position and radius are honored
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) anchor c at (2,1) } "
            "shaft s at g.p hand h on s dial at g.c { radius: 0.75 } }");
        std::vector<double> q;
        m.assemble(q);
        CHECK(m.dials[0].hasCenter);
        CHECK_APPROX(m.dials[0].center.x, 2.0, 1e-12);
        CHECK_APPROX(m.dials[0].center.y, 1.0, 1e-12);
        CHECK_APPROX(m.dials[0].radius, 0.75, 1e-12);
    }

    // ---- the two M4 examples build and assemble ------------------------------
    {
        const char* files[] = { "examples/train.mech", "examples/clockface.mech" };
        for (const char* f : files) {
            std::ifstream in(f);
            if (!in) { std::fprintf(stderr, "note: %s not found (cwd?)\n", f); continue; }
            std::stringstream ss;
            ss << in.rdbuf();
            Program p;
            bool ok = true;
            try {
                p = parse(lex(ss.str()));
            } catch (...) { ok = false; }
            CHECK(ok);
            if (!ok) continue;
            Model m = buildModel(p.mechanisms[0]);
            std::vector<double> q;
            m.assemble(q);
            CHECK(m.validateAssembly(q).empty());
        }
    }
}

void testKinematicsM4() {
    std::printf("[kinematics M4]\n");

    // a synthesized 60:1 train obeys its ratio and direction
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:60 drive a at 60 deg/s }");
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        const double thA0 = q[(size_t)m.bodies[(size_t)m.bodyIndex("a")].qOffset];
        const double thB0 = q[(size_t)m.bodies[(size_t)m.bodyIndex("b")].qOffset];
        for (double t = 0.25; t <= 2.0; t += 0.25) {
            CHECK(sys.solve(q, t, iters, &res));
            CHECK(res < 1e-9);
            const double thA = q[(size_t)m.bodies[(size_t)m.bodyIndex("a")].qOffset];
            const double thB = q[(size_t)m.bodies[(size_t)m.bodyIndex("b")].qOffset];
            // two external meshes: same sense, speed divided by exactly 60
            CHECK_APPROX(thB - thB0, (thA - thA0) / 60.0, 1e-7);
            CHECK_APPROX(thA - thA0, deg2rad(60.0) * t, 1e-9);
        }
    }
    // an idler flips the sense without touching the ratio
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } shaft a at g.p shaft b "
            "reduce a->b ratio 1:2 same drive a at 60 deg/s }");
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        const double thA0 = q[(size_t)m.bodies[(size_t)m.bodyIndex("a")].qOffset];
        const double thB0 = q[(size_t)m.bodies[(size_t)m.bodyIndex("b")].qOffset];
        CHECK(sys.solve(q, 1.0, iters, &res));
        CHECK(res < 1e-9);
        const double thA = q[(size_t)m.bodies[(size_t)m.bodyIndex("a")].qOffset];
        const double thB = q[(size_t)m.bodies[(size_t)m.bodyIndex("b")].qOffset];
        // 2 meshes -> same sense; ratio 2 (the idler contributes 1)
        CHECK_APPROX(thB - thB0, (thA - thA0) / 2.0, 1e-6);
    }
    // the beat pendulum's period really is 2*beat
    {
        Model m = buildFromText(kBeatPend);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        int iters0 = 0;
        sys.solve(q, 0.0, iters0);
        std::vector<double> qd(m.nq(), 0.0);
        const int ip = m.bodyIndex("pend");
        const double h = m.world.step;
        // The pendulum swings about -90 deg (down). Measure the period between
        // successive downward crossings of the vertical — robust against the
        // release angle landing exactly on a sample point.
        const double mid = -0.5 * kPi;
        double prev = q[(size_t)m.bodies[(size_t)ip].qOffset];
        double tPrev = 0.0;
        double tFirst = -1.0, tSecond = -1.0;
        for (long k = 1; k <= (long)(4.5 / h); ++k) {
            dyn.step(q, qd, (k - 1) * h);
            const double th = q[(size_t)m.bodies[(size_t)ip].qOffset];
            if (prev > mid && th <= mid) {
                const double f = (prev - mid) / (prev - th);
                const double t = tPrev + f * h;
                if (tFirst < 0.0) tFirst = t;
                else if (tSecond < 0.0) tSecond = t;
            }
            prev = th;
            tPrev = k * h;
        }
        CHECK(tFirst > 0.4 && tFirst < 0.6);        // ~half a period in
        CHECK(tSecond > 2.4 && tSecond < 2.6);       // one period later
        CHECK_APPROX(tSecond - tFirst, 2.0, 0.01);   // 0.5% at 3 deg amplitude
    }
}

// ---- M5: hybrid modes & weight drives -----------------------------------------

const char* kImpulsePend = R"(
mechanism imp {
  world { gravity: 9.81, step: 0.002 }
  ground g { anchor p at (0, 0) }
  rod pend on g.p { beat: 1s, from: -86 deg }
  mode m {
    state a { }
    state b { }
    a -> b when pend.theta > -88 deg: impulse pend 1 mJ
    b -> a when pend.theta < -92 deg: impulse pend 1 mJ
  }
}
)";

const char* kMaintainPend = R"(
mechanism maint {
  world { gravity: 9.81, step: 0.002 }
  ground g { anchor p at (0, 0) }
  rod pend on g.p { beat: 1s, from: -86 deg }
  mode m {
    state a { }
    state b { }
    a -> b when pend.theta > -87.5 deg: impulse pend maintain 3 deg
    b -> a when pend.theta < -92.5 deg: impulse pend maintain 3 deg
  }
}
)";

const char* kEscapement = R"(
mechanism esc {
  world { gravity: 9.81, step: 0.004 }
  ground g { anchor p at (0, 0) anchor a at (0.42, 0) }
  rod pend on g.p { beat: 1s, from: -87 deg }
  shaft wheel at g.a
  gear w { teeth: 30, module: 0.008, mass: 0.5 } on wheel
  drive wheel weight 4 kg barrel 0.02 drop 1.5 m
  mode esc {
    state tick { hold wheel }
    state run1 { }
    state tock { hold wheel }
    state run2 { }
    tick -> run1 when pend.theta > -87.5 deg
    run1 -> tock when wheel.advance > 6 deg: impulse pend maintain 3 deg
    tock -> run2 when pend.theta < -92.5 deg
    run2 -> tick when wheel.advance > 6 deg: impulse pend maintain 3 deg
  }
}
)";

const char* kRunDown = R"(
mechanism rundown {
  world { gravity: 9.81, step: 0.004 }
  ground g { anchor p at (0, 0) anchor a at (0.42, 0) }
  rod pend on g.p { beat: 1s, from: -87 deg }
  shaft wheel at g.a
  gear w { teeth: 30, module: 0.008, mass: 0.5 } on wheel
  drive wheel weight 4 kg barrel 0.02 drop 0.0052 m
  mode esc {
    state tick { hold wheel }
    state run1 { }
    state tock { hold wheel }
    state run2 { }
    tick -> run1 when pend.theta > -87.5 deg
    run1 -> tock when wheel.advance > 6 deg: impulse pend maintain 3 deg
    tock -> run2 when pend.theta < -92.5 deg
    run2 -> tick when wheel.advance > 6 deg: impulse pend maintain 3 deg
  }
}
)";

const char* kWeightFly = R"(
mechanism wf {
  world { gravity: 9.81, step: 0.002 }
  ground g { anchor p at (0, 0) }
  shaft f at g.p
  gear big { radius: 0.1 } on f
  drive f weight 2 kg
}
)";

// the same escapement, wound the other way: the cord unwinds clockwise,
// so the wheel steps -6 degrees per beat and the ratchet pays on negative
// travel. This is the direction clock hands sweep.
const char* kEscapementCW = R"(
mechanism esccw {
  world { gravity: 9.81, step: 0.004 }
  ground g { anchor p at (0, 0) anchor a at (0.42, 0) }
  rod pend on g.p { beat: 1s, from: -87 deg }
  shaft wheel at g.a
  gear w { teeth: 30, module: 0.008, mass: 0.5 } on wheel
  drive wheel weight 4 kg barrel 0.02 drop 1.5 m cw
  mode esc {
    state tick { hold wheel }
    state run1 { }
    state tock { hold wheel }
    state run2 { }
    tick -> run1 when pend.theta > -87.5 deg
    run1 -> tock when wheel.advance < -6 deg: impulse pend maintain 3 deg
    tock -> run2 when pend.theta < -92.5 deg
    run2 -> tick when wheel.advance < -6 deg: impulse pend maintain 3 deg
  }
}
)";

void testParserM5() {
    std::printf("[parser M5]\n");

    // 'mode' is a keyword; the rest of the M5 vocabulary stays contextual
    {
        std::vector<Token> toks = lex("mode state when hold impulse maintain advance weight");
        CHECK(toks[0].type == Tok::Kw && toks[0].text == "mode");
        for (size_t i = 1; i < toks.size() - 1; ++i)
            CHECK(toks[i].type == Tok::Ident);
        CHECK(toks[toks.size() - 1].type == Tok::End);
    }
    // '<' and '>' lex as symbols
    {
        std::vector<Token> toks = lex("a > b < c");
        CHECK(toks[1].type == Tok::Sym && toks[1].text == ">");
        CHECK(toks[3].type == Tok::Sym && toks[3].text == "<");
    }
    // full mode block: states, holds, guards with units, resets
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "rod w on g.p { length: 1 } "
            "mode esc { state tick { hold w } state run { } "
            "tick -> run when w.theta > 10 deg "
            "run -> tick when w.advance < -0.5: impulse w maintain 3 deg } }");
        const ModeDecl& md = p.mechanisms[0].modes[0];
        CHECK(md.name == "esc");
        CHECK(md.states.size() == 2);
        CHECK(md.states[0].name == "tick" && md.states[0].holds.size() == 1);
        CHECK(md.states[0].holds[0] == "w");
        CHECK(md.states[1].holds.empty());
        CHECK(md.transitions.size() == 2);
        const ModeDecl::Transition& t0 = md.transitions[0];
        CHECK(t0.from == "tick" && t0.to == "run");
        CHECK(t0.body == "w");
        CHECK(t0.field == ModeDecl::Transition::Field::Theta);
        CHECK(t0.greater);
        CHECK_APPROX(t0.threshold, deg2rad(10.0), 1e-12);
        CHECK(t0.resets.empty());
        const ModeDecl::Transition& t1 = md.transitions[1];
        CHECK(t1.field == ModeDecl::Transition::Field::Advance);
        CHECK(!t1.greater);
        CHECK_APPROX(t1.threshold, -0.5, 1e-12);   // raw radians
        CHECK(t1.resets.size() == 1);
        CHECK(t1.resets[0].kind == ModeDecl::Reset::Kind::Maintain);
        CHECK(t1.resets[0].body == "w");
        CHECK_APPROX(t1.resets[0].swingDeg, 3.0, 1e-12);
    }
    // omega guards take deg/s (or raw rad/s); several resets per transition
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "rod w on g.p { length: 1 } "
            "mode m2 { state s { } "
            "s -> s when w.omega > 30 deg/s: impulse w 2 mJ, impulse w -0.5 mJ } }");
        const ModeDecl::Transition& t = p.mechanisms[0].modes[0].transitions[0];
        CHECK(t.field == ModeDecl::Transition::Field::Omega);
        CHECK_APPROX(t.threshold, deg2rad(30.0), 1e-12);
        CHECK(t.resets.size() == 2);
        CHECK_APPROX(t.resets[0].energy, 0.002, 1e-15);
        CHECK_APPROX(t.resets[1].energy, -0.0005, 1e-15);
        CHECK(t.from == t.to);   // self-transition is legal (a ratchet)
    }
    // weight drive: mass [kg] barrel [m] drop [m], all optional but mass
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "shaft s at g.p gear w { radius: 0.1 } on s "
            "drive s weight 4 kg barrel 0.02 drop 1.5 m }");
        const DriveDecl& d = p.mechanisms[0].drives[0];
        CHECK(d.isWeight && !d.isTorque);
        CHECK_APPROX(d.weightMass, 4.0, 1e-12);
        CHECK(d.hasBarrel);
        CHECK_APPROX(d.barrelR, 0.02, 1e-12);
        CHECK(d.hasDrop);
        CHECK_APPROX(d.dropM, 1.5, 1e-12);
    }
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "shaft s at g.p drive s weight 2.5 }");
        const DriveDecl& d = p.mechanisms[0].drives[0];
        CHECK(d.isWeight);
        CHECK_APPROX(d.weightMass, 2.5, 1e-12);
        CHECK(!d.hasBarrel && !d.hasDrop);
        CHECK(d.weightSign == 1);   // default: cord unwinds ccw (+theta)
    }
    // weight sense: 'cw' / 'ccw' (contextual, optional, trailing)
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "shaft s at g.p gear w { radius: 0.1 } on s "
            "drive s weight 4 kg barrel 0.02 drop 1.5 m cw }");
        const DriveDecl& d = p.mechanisms[0].drives[0];
        CHECK(d.isWeight);
        CHECK(d.weightSign == -1);
    }
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "shaft s at g.p gear w { radius: 0.1 } on s "
            "drive s weight 4 kg ccw }");
        CHECK(p.mechanisms[0].drives[0].weightSign == 1);
    }
    // maintain in raw radians converts to degrees
    {
        Program p = parseText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "rod w on g.p { length: 1 } "
            "mode m { state s { } s -> s when w.theta > 0: impulse w maintain 0.1 } }");
        const ModeDecl::Reset& rs = p.mechanisms[0].modes[0].transitions[0].resets[0];
        CHECK_APPROX(rs.swingDeg, rad2deg(0.1), 1e-9);
    }

    auto expectParseError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            parseText(src);
        } catch (const ParseError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s w.theta > 0 } }", "'when'");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s when w.speed > 0 } }", "guard field");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s when w.theta 0 } }", "'>' or '<'");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s when w.omega > 5 deg } }", "'deg/s'");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s when w.theta > 5 deg/s } }", "'deg'");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s when w.theta > 0: kick w 1 } }", "expected reset");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } s -> s when w.theta > 0: impulse w maintain 0 } }", "positive");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { grab w } } }", "expected 'hold");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { state s { } ", "unterminated");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m { 4 } }", "'state' or a transition");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } shaft s at g.p "
        "drive s weight 0 kg } }", "positive mass");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } shaft s at g.p "
        "drive s weight 1 kg barrel -1 } }", "positive radius");
    expectParseError(
        "mechanism m { world { } ground g { anchor p at (0,0) } shaft s at g.p "
        "drive s weight 1 kg drop 0 } }", "positive length");
    expectParseError(
        "mechanism m { ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "modex } }", "'mode'");
}

void testModelM5() {
    std::printf("[model M5]\n");

    auto expectModelError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src);
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };

    // a mode flips the mechanism to dynamic, all on its own
    {
        Model m = buildFromText(
            "mechanism m { ground g { anchor p at (0,0) } "
            "rod w on g.p { length: 1 } "
            "mode m2 { state s { } s -> s when w.theta > 0.1 } }");
        CHECK(m.world.dynamic);
        CHECK(m.modes.size() == 1);
        DofReport d = m.dofReport();
        CHECK(d.nModes == 1 && d.nHolds == 0);
        CHECK(d.residualDof == 1);   // the pendulum, governed by dynamics
    }
    // holds become rate-0 drive rows, one per body, enabled per initial state
    {
        Model m = buildFromText(
            "mechanism m { world { } ground g { anchor p at (0,0) anchor q at (1,0) } "
            "rod w on g.p { length: 1 } rod v on g.q { length: 1 } "
            "mode m2 { state a { hold w } state b { hold v } "
            "a -> b when w.theta > 0.1 b -> a when v.theta < -0.1 } }");
        CHECK(m.modes[0].holdBodies.size() == 2);   // union across states
        CHECK(m.drives.size() == 2);                // one row per held body
        CHECK(m.drives[0].omegaRad == 0.0 && m.drives[1].omegaRad == 0.0);
        CHECK(m.drives[0].enabled && !m.drives[1].enabled);  // initial state a
        DofReport d = m.dofReport();
        CHECK(d.nDrives == 1 && d.nHolds == 1);
        CHECK(d.residualDof == 1);   // 2 free DOF, 1 hold engaged
        // the same body held in two states is still one row
        CHECK(m.modes[0].states[0].holdBodies.size() == 1);
        CHECK(m.modes[0].states[1].holdBodies.size() == 1);
    }
    // capture at assembly: hold targets and advance origins come from q
    {
        Model m = buildFromText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "rod w on g.p { from: 20 deg, length: 1 } "
            "mode m2 { state a { hold w } state b { } "
            "a -> b when w.theta > 3 b -> a when w.theta < -3 } }");
        std::vector<double> q;
        m.assemble(q);
        const int o = m.bodies[1].qOffset;
        CHECK_APPROX(m.drives[0].theta0Rad, q[o], 1e-12);
        CHECK_APPROX(m.modes[0].entryTheta[0], q[o], 1e-12);
        CHECK(m.modes[0].current == 0);
        CHECK(m.modes[0].fires == 0);
        // weights rewind too
    }
    // a hold coexists with a torque or weight drive on the same body
    {
        Model m = buildFromText(
            "mechanism m { world { } ground g { anchor p at (0,0) } "
            "shaft s at g.p gear w { radius: 0.1 } on s "
            "drive s weight 3 kg "
            "mode m2 { state a { hold s } state b { } "
            "a -> b when s.theta > 1 b -> a when s.theta < -1 } }");
        CHECK(m.weights.size() == 1);
        CHECK(m.drives.size() == 1);   // the hold row
        CHECK(m.drives[0].enabled);
        CHECK_APPROX(m.weights[0].torque, 3.0 * 9.81 * 0.1, 1e-9);
    }
    // weight drive: barrel defaults to the host's largest gear; drop to unlimited
    {
        Model m = buildFromText(kWeightFly);
        CHECK(m.weights.size() == 1);
        CHECK(m.drives.empty() && m.torques.empty());
        CHECK_APPROX(m.weights[0].barrelR, 0.1, 1e-12);
        CHECK_APPROX(m.weights[0].torque, 2.0 * 9.81 * 0.1, 1e-12);
        CHECK(m.weights[0].drop < 0.0);   // no drop: unlimited cord
        DofReport d = m.dofReport();
        CHECK(d.nWeights == 1);
        CHECK(d.residualDof == 1);        // free rotor under torque
    }

    // --- error classes -------------------------------------------------------
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { } }", "no states");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } state s { } } }", "more than once");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } s -> x when w.theta > 1 } }", "unknown state");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } x -> s when w.theta > 1 } }", "unknown state");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { hold nope } } }", "unknown body");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { hold g } } }", "ground body");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { hold w hold w } } }", "more than once");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } s -> s when nope.theta > 1 } }", "unknown body");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } s -> s when g.theta > 1 } }", "ground body");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } s -> s when w.theta > 1: impulse nope 1 mJ } }", "unknown body");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode m2 { state s { } s -> s when w.theta > 1: impulse g 1 mJ } }", "ground body");
    // a hold may not pin a rate-driven body
    expectModelError(
        "mechanism m { ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "drive w at 30 deg/s "
        "mode m2 { state s { hold w } } }", "rate drive");
    // one hold row per body, across modes too
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } rod w on g.p { length: 1 } "
        "mode a { state s { hold w } } mode b { state s { hold w } } }",
        "more than one mode");
    // weights: no gear to default the barrel to
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } shaft s at g.p "
        "drive s weight 1 kg }", "'barrel'");
    // weights collide with other drives on the same body
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } shaft s at g.p "
        "gear w { radius: 0.1 } on s drive s weight 1 kg drive s torque 2 }",
        "driven more than once");
    expectModelError(
        "mechanism m { world { } ground g { anchor p at (0,0) } shaft s at g.p "
        "drive s weight 1 kg }", "'barrel'");

    // --- the M5 flagship file builds, assembles and validates ------------------
    {
        std::ifstream in("examples/clock.mech");
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            Model m = buildModel(parse(lex(ss.str())).mechanisms[0]);
            std::vector<double> q;
            m.assemble(q);
            CHECK(m.validateAssembly(q).empty());
            // three hands: second on the escape arbor, minute + hour coaxial
            CHECK(m.hands.size() == 3);
            CHECK(m.dials.size() == 2);
            // the weight unwinds clockwise (hands sweep forward)
            CHECK(m.weights.size() == 1 && m.weights[0].sign == -1);
            // coaxial stack: minute and hour pin to the same ground anchor
            const int imin = m.bodyIndex("minute"), ihou = m.bodyIndex("hour");
            CHECK(imin >= 0 && ihou >= 0);
            int amin = -1, ahou = -1;
            for (const PinJoint& pj : m.pins) {
                if (pj.bodyA == imin && pj.bodyB == 0) amin = pj.anchB;
                if (pj.bodyA == ihou && pj.bodyB == 0) ahou = pj.anchB;
            }
            CHECK(amin >= 0 && amin == ahou);
        } else {
            std::fprintf(stderr, "note: examples/clock.mech not found (cwd?)\n");
        }
    }
}

void testDynamicsM5() {
    std::printf("[dynamics M5]\n");

    // --- impulse resets deliver their energy exactly ------------------------
    // Each beat injects 1 mJ along the pendulum's motion; over six seconds
    // the mechanical energy must rise by exactly fires * 1 mJ.
    {
        Model m = buildFromText(kImpulsePend);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(6.0 / h);
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
        const double E1 = dyn.kinetic(q, qd) + dyn.potential(q);
        const long fires = m.modes[0].fires;
        CHECK(fires >= 4);                       // ~1 event per second
        CHECK(std::fabs(dyn.hybridStats().impulseEnergy - fires * 1e-3) < 1e-9);
        CHECK(std::fabs((E1 - E0) - fires * 1e-3) < 2e-5);
        // no impact losses: nothing is held or released here
        CHECK(std::fabs(dyn.hybridStats().impactLoss) < 1e-12);
        // the amplitude grew (energy in, none out)
        CHECK(E1 - E0 > 3e-3);
    }

    // --- 'maintain' holds the swing at its target ---------------------------
    // Released at a 4 degree swing with a 3 degree maintain: one event brakes
    // the pendulum onto the limit cycle, and it stays there.
    {
        Model m = buildFromText(kMaintainPend);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(8.0 / h);
        const int o = m.bodies[1].qOffset;
        double hi = -1e9, lo = 1e9;
        for (long k = 1; k <= steps; ++k) {
            dyn.step(q, qd, (k - 1) * h);
            if (k * h >= 4.0) {                  // measure the last four seconds
                hi = std::max(hi, q[o]);
                lo = std::min(lo, q[o]);
            }
        }
        CHECK(m.modes[0].fires >= 6);
        CHECK_APPROX(hi, deg2rad(-87.0), deg2rad(0.3));
        CHECK_APPROX(lo, deg2rad(-93.0), deg2rad(0.3));
        // net impulse energy ~ 0: it braked the excess away on the first beat
        CHECK(dyn.hybridStats().impulseEnergy < 0.0);
    }

    // --- the escapement: half a tooth per beat, exactly ----------------------
    // Locked states hold the wheel; each unlock lets the weight run it 6 deg;
    // the maintain impulse holds the pendulum at 3 degrees. The ledger closes.
    {
        Model m = buildFromText(kEscapement);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(10.0 / h);
        const int op = m.bodies[1].qOffset;      // pendulum
        const int ow = m.bodies[2].qOffset;      // wheel
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        double hi = -1e9, lo = 1e9;
        for (long k = 1; k <= steps; ++k) {
            dyn.step(q, qd, (k - 1) * h);
            if (k * h >= 6.0) {
                hi = std::max(hi, q[op]);
                lo = std::min(lo, q[op]);
            }
        }
        const double E1 = dyn.kinetic(q, qd) + dyn.potential(q);
        const long fires = m.modes[0].fires;
        const long beats = fires / 2;
        CHECK(beats >= 8 && beats <= 11);
        // the wheel advanced 6 degrees per beat, to within the event
        // localization overshoot (<= h/1024 of wheel speed)
        const double perBeat = rad2deg(q[ow]) / (double)beats;
        CHECK(perBeat > 6.0 && perBeat < 6.15);
        // the pendulum is held at its 3 degree swing
        CHECK_APPROX(hi, deg2rad(-87.0), deg2rad(0.5));
        CHECK_APPROX(lo, deg2rad(-93.0), deg2rad(0.5));
        // constraints hold through all the discrete switching
        CHECK(sys.maxResidual(q, steps * h) < 1e-8);
        // the hold transmits the weight's torque while engaged
        CHECK(std::fabs(dyn.driveReactions()[0]) > 0.5 * m.weights[0].torque);
        // the ledger closes: dE = drawn + impulse - impact
        const HybridStats& hs = dyn.hybridStats();
        const double resid = (E1 - E0) - (hs.weightDrawn + hs.impulseEnergy - hs.impactLoss);
        CHECK(std::fabs(resid) < 5e-3);
        // the ratchet paid out exactly half a pitch of cord per beat
        CHECK_APPROX(m.weights[0].paid, beats * 0.02 * deg2rad(6.0), 2e-4);
        // and (almost) everything the weight paid went to the pallet impact
        CHECK(hs.impactLoss > 0.9 * hs.weightDrawn);
    }

    // --- determinism: same file, same numbers, always -------------------------
    {
        std::vector<double> qA, qdA;
        for (int run = 0; run < 2; ++run) {
            Model m = buildFromText(kEscapement);
            ConstraintSystem sys(m);
            DynamicsSystem dyn(m, sys);
            std::vector<double> q;
            m.assemble(q);
            std::vector<double> qd(m.nq(), 0.0);
            const double h = m.world.step;
            const long steps = (long)std::llround(3.0 / h);
            for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
            if (run == 0) { qA = q; qdA = qd; }
            else {
                CHECK(q == qA);
                CHECK(qd == qdA);
            }
        }
    }

    // --- the run-down: a finite cord stops the clock --------------------------
    // drop = 0.0052 m = r * half-pitch * 2.5 -> about 2.5 beats of energy.
    // The wheel freezes mid-state, the weight pays out its whole budget, and
    // exactly m*g*drop of energy was drawn.
    {
        Model m = buildFromText(kRunDown);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long half = (long)std::llround(6.0 / h);
        const long full = (long)std::llround(12.0 / h);
        const int ow = m.bodies[2].qOffset;
        for (long k = 0; k < half; ++k) dyn.step(q, qd, k * h);
        const long fires6 = m.modes[0].fires;
        const double th6 = q[ow];
        double hi = -1e9, lo = 1e9;
        for (long k = half; k < full; ++k) {
            dyn.step(q, qd, k * h);
            hi = std::max(hi, q[m.bodies[1].qOffset]);
            lo = std::min(lo, q[m.bodies[1].qOffset]);
        }
        CHECK(m.modes[0].fires == fires6);       // no more beats after run-down
        CHECK(std::fabs(q[ow] - th6) < 1e-12);   // the wheel is frozen
        CHECK(std::fabs(qd[ow]) < 1e-12);
        CHECK(m.modes[0].current == 1 || m.modes[0].current == 3);  // stuck mid-beat
        CHECK(m.weights[0].exhausted);
        CHECK_APPROX(m.weights[0].paid, 0.0052, 1e-12);   // the whole cord
        CHECK_APPROX(dyn.hybridStats().weightDrawn,
                     4.0 * 9.81 * 0.0052, 1e-6);          // the whole budget
        // the pendulum itself keeps swinging freely at the maintained swing
        CHECK(lo < deg2rad(-89.0) && hi > deg2rad(-91.0));
    }

    // --- a weight is a torque, until it is not --------------------------------
    // No drop: the flywheel spins up exactly as a torque drive would.
    {
        Model m = buildFromText(kWeightFly);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(0.1 / h);
        const int o = m.bodies[1].qOffset;
        const double th0 = q[o];   // assembled seed angle
        for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
        // I = shaft 1e-4 + disc 0.5 * (pi r^2) * r^2
        const double I = 1e-4 + 0.5 * (kPi * 0.01) * 0.01;
        const double tau = 2.0 * 9.81 * 0.1;
        CHECK_APPROX(qd[o], tau / I * 0.1, 1e-6 * std::fabs(tau / I * 0.1));
        // the cord paid out from the assembled angle, exactly theta - theta0
        CHECK_APPROX(dyn.hybridStats().weightDrawn, tau * (q[o] - th0), 1e-9);
        CHECK(!m.weights[0].exhausted);
    }

    // --- a clockwise weight: same physics, mirrored sense ---------------------
    // The torque acts along -theta, the ratchet pays cord for negative
    // travel, the ledger still closes, and the wheel spins clockwise — the
    // direction a clock's hands must sweep.
    {
        Model m = buildFromText(
            "mechanism wf { world { gravity: 9.81, step: 0.002 } "
            "ground g { anchor p at (0, 0) } shaft f at g.p "
            "gear big { radius: 0.1 } on f drive f weight 2 kg barrel 0.1 cw }");
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(0.1 / h);
        const int o = m.bodies[1].qOffset;
        const double th0 = q[o];
        for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
        const double I = 1e-4 + 0.5 * (kPi * 0.01) * 0.01;
        const double tau = 2.0 * 9.81 * 0.1;
        CHECK(qd[o] < 0.0);                                  // spins clockwise
        CHECK_APPROX(qd[o], -tau / I * 0.1, 1e-6 * (tau / I * 0.1));
        CHECK(q[o] < th0);                                   // travelled -theta
        // cord paid for |theta - theta0|, energy = tau * |travel|
        CHECK_APPROX(dyn.hybridStats().weightDrawn, tau * (th0 - q[o]), 1e-9);
    }

    // --- the escapement wound clockwise: -6 degrees per beat ------------------
    {
        Model m = buildFromText(kEscapementCW);
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q;
        m.assemble(q);
        std::vector<double> qd(m.nq(), 0.0);
        const double h = m.world.step;
        const long steps = (long)std::llround(10.0 / h);
        const int op = m.bodies[1].qOffset;      // pendulum
        const int ow = m.bodies[2].qOffset;      // wheel
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        double hi = -1e9, lo = 1e9;
        for (long k = 1; k <= steps; ++k) {
            dyn.step(q, qd, (k - 1) * h);
            if (k * h >= 6.0) {
                hi = std::max(hi, q[op]);
                lo = std::min(lo, q[op]);
            }
        }
        const double E1 = dyn.kinetic(q, qd) + dyn.potential(q);
        const long beats = m.modes[0].fires / 2;
        CHECK(beats >= 8 && beats <= 11);
        // the wheel stepped -6 degrees per beat (clockwise)
        const double perBeat = rad2deg(q[ow]) / (double)beats;
        CHECK(perBeat < -6.0 && perBeat > -6.15);
        // the pendulum still swings its maintained 3 degrees
        CHECK_APPROX(hi, deg2rad(-87.0), deg2rad(0.5));
        CHECK_APPROX(lo, deg2rad(-93.0), deg2rad(0.5));
        CHECK(sys.maxResidual(q, steps * h) < 1e-8);
        // the ratchet paid half a pitch of cord per beat, negative travel
        CHECK_APPROX(m.weights[0].paid, beats * 0.02 * deg2rad(6.0), 2e-4);
        // and the ledger closes as before
        const HybridStats& hs = dyn.hybridStats();
        const double resid = (E1 - E0) - (hs.weightDrawn + hs.impulseEnergy - hs.impactLoss);
        CHECK(std::fabs(resid) < 5e-3);
        CHECK(hs.impactLoss > 0.9 * hs.weightDrawn);
    }

    // --- motion work: coaxial minute and hour hands at exactly 1:12 -----------
    // The horologist's equal-tooth-sum stack: a 10-leaf cannon on the minute
    // arbor meshes a 30-tooth wheel on the stud, whose 8-leaf pinion meshes
    // the 32-tooth hour wheel back on the same axis (10+30 = 8+32, so both
    // meshes sit at the same centre distance). 10/30 * 8/32 = 1/12 exactly.
    {
        Model m = buildFromText(
            "mechanism mw { ground g { anchor c at (0,0) anchor i at (0.12,0) } "
            "shaft minute at g.c shaft hour at g.c shaft inter at g.i "
            "gear cannon { teeth: 10, module: 0.006 } on minute "
            "gear studw { teeth: 30, module: 0.006 } on inter "
            "gear studp { teeth: 8, module: 0.006 } on inter "
            "gear hourw { teeth: 32, module: 0.006 } on hour "
            "mesh cannon studw mesh studp hourw "
            "drive minute at 30 deg/s }");
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        const int omin = m.bodies[(size_t)m.bodyIndex("minute")].qOffset;
        const int oint = m.bodies[(size_t)m.bodyIndex("inter")].qOffset;
        const int ohou = m.bodies[(size_t)m.bodyIndex("hour")].qOffset;
        const double m0 = q[(size_t)omin], i0 = q[(size_t)oint], h0 = q[(size_t)ohou];
        for (double t = 0.5; t <= 3.0; t += 0.5) {
            CHECK(sys.solve(q, t, iters, &res));
            CHECK(res < 1e-9);
            const double dm = q[(size_t)omin] - m0;
            const double di = q[(size_t)oint] - i0;
            const double dh = q[(size_t)ohou] - h0;
            CHECK_APPROX(dm, deg2rad(30.0) * t, 1e-9);
            // two external meshes: the hour wheel keeps the minute arbor's
            // sense at exactly 1/12 the speed
            CHECK(dh > 0.0);
            CHECK_APPROX(dh, dm / 12.0, 1e-7);
            // the stud runs backwards at 1/3 the minute speed
            CHECK(di < 0.0);
            CHECK_APPROX(di, -dm / 3.0, 1e-7);
        }
    }

    // --- the flagship: a weight-driven clock with three hands -----------------
    // Escape arbor 1 rev/min clockwise; the going train divides by 60 to the
    // minute arbor (same sense), the motion work by another 12 to the hour
    // arbor (same sense) — every hand sweeps clockwise, in exact ratio.
    {
        std::ifstream in("examples/clock.mech");
        if (!in) {
            std::fprintf(stderr, "note: examples/clock.mech not found (cwd?)\n");
        } else {
            std::stringstream ss;
            ss << in.rdbuf();
            Model m = buildModel(parse(lex(ss.str())).mechanisms[0]);
            ConstraintSystem sys(m);
            DynamicsSystem dyn(m, sys);
            std::vector<double> q;
            m.assemble(q);
            std::vector<double> qd(m.nq(), 0.0);
            const int oesc = m.bodies[(size_t)m.bodyIndex("escape")].qOffset;
            const int omin = m.bodies[(size_t)m.bodyIndex("minute")].qOffset;
            const int ohou = m.bodies[(size_t)m.bodyIndex("hour")].qOffset;
            const double e0 = q[(size_t)oesc];
            const double m0 = q[(size_t)omin];
            const double h0 = q[(size_t)ohou];
            const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
            const double h = m.world.step;
            const long steps = (long)std::llround(8.0 / h);
            for (long k = 0; k < steps; ++k) dyn.step(q, qd, k * h);
            const double de = q[(size_t)oesc] - e0;
            const double dm = q[(size_t)omin] - m0;
            const double dh = q[(size_t)ohou] - h0;
            CHECK(m.modes[0].fires >= 12);   // ~7 beats in 8 s (first at ~1 s)
            // the escape wheel ran clockwise, half a tooth per beat
            CHECK(de < -deg2rad(36.0));
            const long beats = m.modes[0].fires / 2;
            CHECK_APPROX(de, -(double)beats * deg2rad(6.0), deg2rad(0.2));
            // minute = escape / 60, same sense (both negative)
            CHECK(dm < 0.0);
            CHECK_APPROX(dm, de / 60.0, 1e-6);
            // hour = minute / 12, same sense
            CHECK(dh < 0.0);
            CHECK_APPROX(dh, dm / 12.0, 1e-7);
            // constraints survived all the discrete switching
            CHECK(sys.maxResidual(q, steps * h) < 1e-8);
            // and the ledger closes
            const double E1 = dyn.kinetic(q, qd) + dyn.potential(q);
            const HybridStats& hs = dyn.hybridStats();
            const double resid = (E1 - E0) - (hs.weightDrawn + hs.impulseEnergy - hs.impactLoss);
            CHECK(std::fabs(resid) < 5e-3);
            CHECK_APPROX(m.weights[0].paid, beats * 0.02 * deg2rad(6.0), 2e-4);
        }
    }
}


// ---- M6a: profiles and bilateral contact -------------------------------------

// eccentric disc (R = 0.03, rim centre e = 0.01 off the axle) under a roller
// follower on a rail through the axle; the profile line is spliced in.
std::string eccentricSrc(const std::string& profileLine, const char* roller) {
    std::string s =
        "mechanism ecc {\n"
        "  ground frame { anchor axle at (0, 0)  anchor rail at (0, 0.09) dir (0, 1) }\n"
        "  body cam { anchor axle at (0, 0)  anchor rim at (0.010, 0) }\n"
        "  pin cam.axle -> frame.axle\n";
    s += "  " + profileLine + "\n";
    s += "  body follower { anchor tip at (0, 0) dir (1, 0) }\n"
         "  slide follower.tip -> frame.rail\n"
         "  touch follower.tip on disc ";
    s += roller;
    s += "\n  drive cam at 30 deg/s\n}\n";
    return s;
}

// y(theta) for the eccentric: e sin(theta) + sqrt((R+rho)^2 - e^2 cos^2(theta))
double eccentricY(double th, double rho) {
    const double e = 0.010, R = 0.030 + rho;
    return e * std::sin(th) + std::sqrt(R * R - e * e * std::cos(th) * std::cos(th));
}

// Run the eccentric over two turns (5 deg steps, warm-started) and return the
// worst deviation of the follower height from the closed form.
double eccentricWorstError(const std::string& src, double rho, double* worstRes) {
    Model m = buildFromText(src.c_str());
    ConstraintSystem sys(m);
    std::vector<double> q;
    m.assemble(q);
    int iters = 0;
    double res = 0.0;
    sys.solve(q, 0.0, iters, &res);
    const int ifol = m.bodyIndex("follower");
    double worst = 0.0;
    *worstRes = 0.0;
    for (int k = 0; k <= 144; ++k) {
        const double t = k * (5.0 / 30.0);
        sys.solve(q, t, iters, &res);
        *worstRes = std::max(*worstRes, res);
        worst = std::max(worst, std::fabs(q[(size_t)m.bodies[ifol].qOffset + 2] -
                                          eccentricY(deg2rad(30.0) * t, rho)));
    }
    return worst;
}

void testProfileCurve() {
    std::printf("[profile M6a]\n");

    // a polar table of constant radius is exactly a circle (spline reproduces
    // constants), values and both derivatives
    {
        std::vector<double> ang, rad;
        for (int i = 0; i < 7; ++i) { ang.push_back(0.1 + i * 2.0 * kPi / 7.0); rad.push_back(0.03); }
        const ProfileCurve pc = ProfileCurve::polar(ang, rad);
        const ProfileCurve cc = ProfileCurve::circle(0.03);
        double worst = 0.0;
        for (int k = 0; k < 200; ++k) {
            const double s = k * 0.0317;
            Vec2 p, d1, d2, q, e1, e2;
            pc.eval(s, p, d1, d2);
            cc.eval(s, q, e1, e2);
            worst = std::max({worst, vlen(p - q), vlen(d1 - e1), vlen(d2 - e2)});
        }
        CHECK(worst < 1e-12);
    }

    // spline accuracy on a smooth oval, and periodicity / C2 at the seam
    {
        std::vector<double> ang, rad;
        for (int i = 0; i < 36; ++i) {
            const double a = i * 2.0 * kPi / 36.0;
            ang.push_back(a);
            rad.push_back(0.03 + 0.005 * std::cos(2.0 * a));
        }
        const ProfileCurve oval = ProfileCurve::polar(ang, rad);
        double worstPos = 0.0;
        for (int k = 0; k < 500; ++k) {
            const double s = 0.0123 + k * 2.0 * kPi / 500.0;
            Vec2 p, d1, d2;
            oval.eval(s, p, d1, d2);
            const double r = 0.03 + 0.005 * std::cos(2.0 * s);
            worstPos = std::max(worstPos, vlen(p - Vec2{r * std::cos(s), r * std::sin(s)}));
        }
        CHECK(worstPos < 1e-6);
        Vec2 p0, a0, b0, p1, a1, b1;
        oval.eval(0.7, p0, a0, b0);
        oval.eval(0.7 + 2.0 * kPi, p1, a1, b1);
        CHECK(vlen(p0 - p1) < 1e-12 && vlen(a0 - a1) < 1e-12 && vlen(b0 - b1) < 1e-12);
        // C2 across a knot: second derivative continuous
        const double knot = ang[5];
        Vec2 pl, al, bl, pr, ar, br;
        oval.eval(knot - 1e-9, pl, al, bl);
        oval.eval(knot + 1e-9, pr, ar, br);
        CHECK(vlen(bl - br) < 1e-5);
        // first derivative really is the derivative of the position
        Vec2 pa, aa, ba, pb, ab, bb;
        oval.eval(1.234 - 1e-6, pa, aa, ba);
        oval.eval(1.234 + 1e-6, pb, ab, bb);
        Vec2 pm, am, bm;
        oval.eval(1.234, pm, am, bm);
        CHECK(vlen((pb - pa) * (0.5e6) - am) < 1e-6);
    }

    // nearest(): matches brute force on a non-circular curve, signs are right,
    // and a clockwise point list is normalised (outside stays positive)
    {
        std::vector<Vec2> ccw, cw;
        for (int i = 0; i < 40; ++i) {
            const double a = i * 2.0 * kPi / 40.0;
            ccw.push_back({0.05 * std::cos(a), 0.03 * std::sin(a)});
            cw.push_back({0.05 * std::cos(-a), 0.03 * std::sin(-a)});
        }
        const ProfileCurve a = ProfileCurve::points(ccw);
        const ProfileCurve b = ProfileCurve::points(cw);
        unsigned seed = 12345;
        auto rnd = [&]() {
            seed = seed * 1664525u + 1013904223u;
            return ((seed >> 8) & 0xFFFF) / 65535.0 * 0.18 - 0.09;
        };
        double worstDist = 0.0;
        bool signsOk = true, orientOk = true;
        for (int t = 0; t < 300; ++t) {
            const Vec2 u{rnd(), rnd()};
            const ProfileCurve::Foot f = a.nearest(u);
            // reference: dense scan, then ternary-search the best bracket
            double best = 1e9, sBest = 0.0;
            const int N = 20000;
            auto dist = [&](double s) {
                Vec2 p, d1, d2;
                a.eval(s, p, d1, d2);
                return vlen(u - p);
            };
            for (int k = 0; k < N; ++k) {
                const double d = dist(a.period() * k / N);
                if (d < best) { best = d; sBest = a.period() * k / N; }
            }
            double lo = sBest - a.period() / N, hi = sBest + a.period() / N;
            for (int it = 0; it < 80; ++it) {
                const double m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
                if (dist(m1) < dist(m2)) hi = m2; else lo = m1;
            }
            best = std::min(best, dist(0.5 * (lo + hi)));
            worstDist = std::max(worstDist, std::fabs(std::fabs(f.signedDist) - best));
            const bool outside = (u.x / 0.05) * (u.x / 0.05) + (u.y / 0.03) * (u.y / 0.03) > 1.0;
            if ((f.signedDist > 0.0) != outside) signsOk = false;
            if (std::fabs(a.nearest(u).signedDist - b.nearest(u).signedDist) > 1e-9)
                orientOk = false;
        }
        CHECK(worstDist < 1e-9);
        CHECK(signsOk);
        CHECK(orientOk);
        const ProfileCurve::Foot f = a.nearest({0.08, 0.0});
        CHECK_APPROX(f.normal.x, 1.0, 1e-9);
        CHECK_APPROX(f.signedDist, 0.03, 1e-9);
    }

    // circle fast path agrees with the generic search on a fine polygonal circle
    {
        std::vector<Vec2> pts;
        for (int i = 0; i < 96; ++i) {
            const double a = i * 2.0 * kPi / 96.0;
            pts.push_back({0.03 * std::cos(a), 0.03 * std::sin(a)});
        }
        const ProfileCurve gen = ProfileCurve::points(pts);
        const ProfileCurve cir = ProfileCurve::circle(0.03);
        CHECK_APPROX(gen.nearest({0.05, 0.02}).signedDist, cir.nearest({0.05, 0.02}).signedDist, 1e-7);
        CHECK_APPROX(gen.nearest({-0.01, 0.015}).signedDist, cir.nearest({-0.01, 0.015}).signedDist, 1e-7);
    }
}

void testParserM6() {
    std::printf("[parser M6]\n");

    // keywords
    std::vector<Token> k = lex("profile touch");
    CHECK(k[0].type == Tok::Kw && k[1].type == Tok::Kw);

    {
        const Program p = parseText(eccentricSrc("profile disc on cam.rim { circle: 0.03 }",
                                                 "roller 0.006").c_str());
        const MechanismDecl& m = p.mechanisms[0];
        CHECK(m.profiles.size() == 1 && m.touches.size() == 1);
        CHECK(m.profiles[0].name == "disc");
        CHECK(m.profiles[0].kind == ProfileDecl::Kind::Circle);
        CHECK_APPROX(m.profiles[0].radius, 0.03, 1e-12);
        CHECK(m.profiles[0].host == "cam" && m.profiles[0].hostAnchor == "rim");
        CHECK(m.touches[0].body == "follower" && m.touches[0].anchor == "tip");
        CHECK(m.touches[0].profile == "disc");
        CHECK(m.touches[0].hasRoller);
        CHECK_APPROX(m.touches[0].roller, 0.006, 1e-12);
    }

    // host after the block, polar with and without degree units, separators
    {
        const Program p = parseText(
            "mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
            "profile p { polar: (0 deg, 0.03), (90 deg, 0.04) (3.5, 0.03) } on s }");
        const ProfileDecl& pd = p.mechanisms[0].profiles[0];
        CHECK(pd.kind == ProfileDecl::Kind::Polar);
        CHECK(pd.polar.size() == 3);
        CHECK_APPROX(pd.polar[1].first, kPi / 2.0, 1e-12);
        CHECK_APPROX(pd.polar[2].first, 3.5, 1e-12);   // bare number = radians
        CHECK(pd.host == "s" && pd.hostAnchor.empty());
    }
    {
        const Program p = parseText(
            "mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
            "profile p on s { points: (0.03, 0) (0, 0.03) (-0.03, 0) (0, -0.03) } "
            "touch g.a on p }");
        CHECK(p.mechanisms[0].profiles[0].points.size() == 4);
        CHECK(!p.mechanisms[0].touches[0].hasRoller);
    }

    // errors carry a line number and a useful hint
    auto expectParseError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            parseText(src);
        } catch (const ParseError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
            CHECK(e.line >= 1);
        }
        CHECK(threw);
    };
    expectParseError("mechanism m { profile p { } on s }", "no shape");
    expectParseError("mechanism m { profile p { circle: 1 } }", "needs a host");
    expectParseError("mechanism m { profile p on s { circle: 1 circle: 2 } }", "already has a shape");
    expectParseError("mechanism m { profile p on s { square: 1 } }", "'ellipse' or 'polygon'");
    expectParseError("mechanism m { profile p on s { polar: } }", "needs (angle, radius)");
    expectParseError("mechanism m { touch f.tip p }", "expected 'on'");
}

void testModelM6() {
    std::printf("[model M6]\n");

    auto expectModelError = [&](const std::string& src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src.c_str());
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
            CHECK(e.line >= 1);
        }
        CHECK(threw);
    };
    const std::string head =
        "mechanism m { ground g { anchor a at (0,0) anchor r at (0,1) dir (0,1) } "
        "shaft s at g.a  body f { anchor tip at (0,0) dir (1,0) } slide f.tip -> g.r ";

    // good model + DOF bookkeeping: touch removes one DOF
    {
        Model m = buildFromText(eccentricSrc("profile disc on cam.rim { circle: 0.03 }",
                                             "roller 0.006").c_str());
        CHECK(m.profiles.size() == 1 && m.touches.size() == 1);
        CHECK(m.profileIndex("disc") == 0);
        const DofReport d = m.dofReport();
        CHECK(d.nProfiles == 1 && d.nTouches == 1);
        CHECK(d.freeDof == 1);
        CHECK(d.residualDof == 0);
    }

    expectModelError(head + "profile p on s { circle: 0.03 } profile p on s { circle: 0.02 } }",
                     "duplicate profile");
    expectModelError(head + "profile p on nope { circle: 0.03 } }", "unknown host body");
    expectModelError(head + "profile p on s.zzz { circle: 0.03 } }", "no anchor 'zzz'");
    expectModelError(head + "profile p on g { circle: 0.03 } }", "explicit anchor");
    expectModelError(head + "profile p on s { circle: 0 } }", "positive radius");
    expectModelError(head + "profile p on s { polar: (0 deg, 1) (90 deg, 1) } }", "at least 3");
    expectModelError(head + "profile p on s { polar: (0 deg, 1) (90 deg, 1) (80 deg, 1) } }",
                     "strictly increasing");
    expectModelError(head + "profile p on s { polar: (0 deg, 1) (90 deg, 1) (400 deg, 1) } }",
                     "[0, 360)");
    expectModelError(head + "profile p on s { polar: (0 deg, 1) (90 deg, -1) (180 deg, 1) } }",
                     "positive");
    expectModelError(head + "profile p on s { points: (0,0) (1,0) } }", "at least 3");
    expectModelError(head + "profile p on s { points: (0,0) (1,0) (2,0) } }", "enclose no area");
    expectModelError(head + "profile p on s { points: (0,0) (0,0) (1,1) } }", "coincident");

    const std::string withP = head + "profile p on s { circle: 0.03 } ";
    expectModelError(withP + "touch nope.tip on p }", "unknown body");
    expectModelError(withP + "touch f.zzz on p }", "no anchor 'zzz'");
    expectModelError(withP + "touch f.tip on nope }", "unknown profile");
    expectModelError(withP + "touch s.A on p }", "its own profile");
    // a ground point may ride a moving profile; ground cannot touch its own
    buildFromText((withP + "touch g.a on p }").c_str());
    expectModelError(head + "profile p on g.a { circle: 0.03 } touch g.r on p }", "its own profile");
    expectModelError(withP + "touch f.tip on p roller 0 }", "positive radius");
    // restitution belongs to dynamic mechanisms: it is meaningless where contact is bilateral
    expectModelError(withP + "touch f.tip on p restitution 0.5 }", "no effect in a kinematic");
    // ... and in a dynamic one it is range-checked and recorded; touch itself is now legal there
    expectModelError(withP + "touch f.tip on p restitution 1.5 world { gravity: 9.81 } }", "[0, 1]");
    {
        Model m = buildFromText((withP + "touch f.tip on p roller 0.01 restitution 0.4 "
                                         "world { gravity: 9.81 } }").c_str());
        CHECK(m.world.dynamic);
        CHECK_APPROX(m.touches[0].restitution, 0.4, 1e-15);
        CHECK_APPROX(m.touches[0].roller, 0.01, 1e-15);
        Model m2 = buildFromText((withP + "touch f.tip on p restitution 0.4 roller 0.01 "
                                          "world { gravity: 9.81 } }").c_str());
        CHECK_APPROX(m2.touches[0].restitution, 0.4, 1e-15);   // either order
    }
}

void testKinematicsM6() {
    std::printf("[kinematics M6]\n");

    // ---- acceptance: the eccentric disc matches its closed form -----------------
    {
        double res = 0.0;
        const double e0 = eccentricWorstError(
            eccentricSrc("profile disc on cam.rim { circle: 0.03 }", ""), 0.0, &res);
        CHECK(e0 < 1e-9);
        CHECK(res < 1e-9);
        const double e1 = eccentricWorstError(
            eccentricSrc("profile disc on cam.rim { circle: 0.03 }", "roller 0.006"), 0.006, &res);
        CHECK(e1 < 1e-9);
        CHECK(res < 1e-9);
    }

    // a constant-radius polar table goes through the generic spline path and
    // must land on the same closed form; so must a 64-point spline circle and
    // the same points listed clockwise
    {
        double res = 0.0;
        std::string polar = "profile disc on cam.rim { polar:";
        for (int i = 0; i < 9; ++i) {
            char b[64];
            std::snprintf(b, sizeof b, " (%d deg, 0.03)", i * 40);
            polar += b;
        }
        polar += " }";
        CHECK(eccentricWorstError(eccentricSrc(polar, "roller 0.006"), 0.006, &res) < 1e-9);

        for (int cw = 0; cw < 2; ++cw) {
            std::string pts = "profile disc on cam.rim { points:";
            for (int i = 0; i < 64; ++i) {
                const double a = (cw ? -1.0 : 1.0) * i * 2.0 * kPi / 64.0;
                char b[96];
                std::snprintf(b, sizeof b, " (%.12f, %.12f)", 0.03 * std::cos(a), 0.03 * std::sin(a));
                pts += b;
            }
            pts += " }";
            CHECK(eccentricWorstError(eccentricSrc(pts, "roller 0.006"), 0.006, &res) < 2e-6);
            CHECK(res < 1e-9);
        }
    }

    // ---- analytic Jacobian vs central differences ---------------------------------
    // moving cam with a non-circular spline profile, swing-arm roller follower:
    // both sides of the touch row move, and the foot sweeps the curve
    {
        std::string src =
            "mechanism swing {\n"
            "  ground frame { anchor axle at (0, 0)  anchor pivot at (0.12, 0.06) }\n"
            "  body cam { anchor axle at (0, 0)  anchor c at (0.008, 0.004) }\n"
            "  pin cam.axle -> frame.axle\n"
            "  profile oval on cam.c { polar:";
        for (int i = 0; i < 36; ++i) {
            const double a = i * 10.0;
            char b[80];
            std::snprintf(b, sizeof b, " (%g deg, %.8f)", a,
                          0.03 + 0.006 * std::cos(2.0 * deg2rad(a)) + 0.002 * std::sin(deg2rad(a)));
            src += b;
        }
        src += " }\n"
               "  rod arm on frame.pivot { length: 0.12 }\n"
               "  touch arm.B on oval roller 0.005\n"
               "  drive cam at 45 deg/s\n}\n";
        Model m = buildFromText(src.c_str());
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);

        unsigned seed = 7;
        auto rnd = [&]() {
            seed = seed * 1664525u + 1013904223u;
            return ((seed >> 8) & 0xFFFF) / 65535.0 - 0.5;
        };
        double worst = 0.0;
        for (int trial = 0; trial < 12; ++trial) {
            std::vector<double> qq = q;
            for (double& v : qq) v += 0.004 * rnd();     // off the manifold on purpose
            std::vector<double> J;
            sys.jacobian(qq, J);
            const int rows = sys.rowCount();
            for (int j = 0; j < sys.nq; ++j) {
                std::vector<double> qp = qq, qm = qq, cp, cm;
                const double h = 1e-6;
                qp[(size_t)j] += h;
                qm[(size_t)j] -= h;
                sys.residuals(qp, 0.0, cp);
                sys.residuals(qm, 0.0, cm);
                for (int i = 0; i < rows; ++i) {
                    const double fd = (cp[(size_t)i] - cm[(size_t)i]) / (2.0 * h);
                    worst = std::max(worst, std::fabs(fd - J[(size_t)i * sys.nq + j]));
                }
            }
        }
        CHECK(worst < 1e-6);
    }

    // ---- a profile on ground: a rod tip held on a fixed circle ---------------------
    {
        Model m = buildFromText(
            "mechanism gp { ground frame { anchor pivot at (0, 0)  anchor c at (0.1, 0) }\n"
            "  profile ring on frame.c { circle: 0.04 }\n"
            "  rod arm on frame.pivot { length: 0.12 }\n"
            "  touch arm.B on ring }");
        CHECK(m.dofReport().freeDof == 0);
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(res < 1e-9);
        const Vec2 tip = m.worldAnchor(m.bodyIndex("arm"), 1, q);
        CHECK_APPROX(vlen(tip), 0.12, 1e-9);
        CHECK_APPROX(vlen(tip - Vec2{0.1, 0.0}), 0.04, 1e-9);
        CHECK(m.validateAssembly(q).empty());
    }

    // ---- an open contact is reported, not hidden -------------------------------------
    {
        // rod too short to ever reach the circle: assembly cannot close the gap
        Model m = buildFromText(
            "mechanism far { ground frame { anchor pivot at (0, 0)  anchor c at (1.0, 0) }\n"
            "  profile ring on frame.c { circle: 0.04 }\n"
            "  rod arm on frame.pivot { length: 0.1 }\n"
            "  touch arm.B on ring }");
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        CHECK(!sys.solve(q, 0.0, iters, &res));
        CHECK(res > 0.1);     // the 0.86 m shortfall is shared between pin and contact
        CHECK(!m.validateAssembly(q).empty());
    }

    // ---- the shipped examples ----------------------------------------------------------
    {
        const char* files[] = { "examples/eccentric.mech", "examples/lobecam.mech" };
        for (const char* f : files) {
            std::ifstream in(f);
            if (!in) { std::fprintf(stderr, "note: %s not found (cwd?)\n", f); continue; }
            std::stringstream ss;
            ss << in.rdbuf();
            Model m = buildModel(parse(lex(ss.str())).mechanisms[0]);
            CHECK(m.dofReport().residualDof == 0);
            ConstraintSystem sys(m);
            std::vector<double> q;
            CHECK(m.assemble(q).empty());
            int iters = 0;
            double res = 0.0;
            CHECK(sys.solve(q, 0.0, iters, &res));
            CHECK(m.validateAssembly(q).empty());
            const int ifol = m.bodyIndex("follower");
            double worst = 0.0, lo = 1e9, hi = -1e9;
            for (int k = 1; k <= 720; ++k) {          // two turns of the cam, 3 deg/step
                const double t = k * (3.0 / m.drives[0].omegaRad * kPi / 180.0);
                sys.solve(q, t, iters, &res);
                worst = std::max(worst, res);
                const double y = q[(size_t)m.bodies[ifol].qOffset + 2];
                lo = std::min(lo, y);
                hi = std::max(hi, y);
            }
            CHECK(worst < 1e-8);
            if (std::string(f).find("eccentric") != std::string::npos) {
                // R + rho -/+ e at the rail: sqrt(0.036^2 - 0.01^2 cos^2) + 0.01 sin
                CHECK_APPROX(hi, eccentricY(kPi / 2.0, 0.006), 1e-6);
                CHECK(lo < 0.0346 && lo > 0.0245);
            } else {
                // dwells: roller centre 36 mm up on the base circle, 46 mm on the lobe top
                CHECK_APPROX(lo, 0.036, 2e-5);
                CHECK_APPROX(hi, 0.046, 2e-5);
            }
        }
    }
}


// ---- M6b: cam synthesis from displacement diagrams -----------------------------

// A plate cam on a shaft, a roller follower on a rail through the axle. `camBody`
// is spliced into the cam block; `touchTail` after the touch.
std::string plateCamSrc(const std::string& camBody, const char* rate, const char* touchTail = "") {
    std::string s =
        "mechanism pc {\n"
        "  ground frame { anchor axle at (0, 0)  anchor rail at (0, 0.09) dir (0, 1) }\n"
        "  shaft cam at frame.axle\n"
        "  cam lobe on cam.A { " + camBody + " }\n"
        "  body follower { anchor tip at (0, 0) dir (1, 0) }\n"
        "  slide follower.tip -> frame.rail\n"
        "  touch follower.tip on lobe ";
    s += touchTail;
    s += "\n  drive cam at ";
    s += rate;
    s += " deg/s\n}\n";
    return s;
}

const char* kCamBody =
    "base 0.025 roller 0.006 rise 0.012 over 100 deg cycloidal dwell 60 deg "
    "return over 140 deg modtrap dwell";

void testCamLaws() {
    std::printf("[cam M6b]\n");

    const MotionLaw laws[] = {MotionLaw::Cycloidal, MotionLaw::Harmonic, MotionLaw::ModTrap};
    const double peakA[] = {2.0 * kPi, 0.5 * kPi * kPi, 4.8881};   // textbook coefficients
    for (int li = 0; li < 3; ++li) {
        const MotionLaw L = laws[li];
        // rest-to-rest, 0 -> 1
        CHECK_APPROX(evalLaw(L, 0.0).s, 0.0, 1e-15);
        CHECK_APPROX(evalLaw(L, 1.0).s, 1.0, 1e-12);
        CHECK_APPROX(evalLaw(L, 0.0).v, 0.0, 1e-12);
        CHECK_APPROX(evalLaw(L, 1.0).v, 0.0, 1e-12);
        double worstV = 0.0, worstA = 0.0, peak = 0.0, minV = 1e9, symm = 0.0;
        for (int k = 1; k < 1000; ++k) {
            const double x = k / 1000.0, h = 1e-6;
            const LawValue f = evalLaw(L, x), p = evalLaw(L, x + h), m = evalLaw(L, x - h);
            worstV = std::max(worstV, std::fabs((p.s - m.s) / (2 * h) - f.v));
            worstA = std::max(worstA, std::fabs((p.v - m.v) / (2 * h) - f.a));
            peak = std::max(peak, std::fabs(f.a));
            minV = std::min(minV, f.v);
            symm = std::max(symm, std::fabs(f.s + evalLaw(L, 1.0 - x).s - 1.0));
        }
        CHECK(worstV < 1e-8);     // v is really ds/dx
        CHECK(worstA < 1e-7);     // a is really dv/dx (continuous: no jumps hiding in the grid)
        CHECK(minV >= -1e-12);    // monotone: the follower never backs up
        CHECK(symm < 1e-12);      // s(x) + s(1-x) = 1
        CHECK_APPROX(peak, peakA[li], 2e-4);
    }
    CHECK(std::string(lawName(MotionLaw::ModTrap)) == "modtrap");
}

void testCamDiagram() {
    std::printf("[cam diagram M6b]\n");

    auto seg = [](CamSegment::Kind k, double lift, double deg, bool rest = false) {
        CamSegment c;
        c.kind = k;
        c.lift = lift;
        c.angle = deg2rad(deg);
        c.restOfTurn = rest;
        return c;
    };
    using K = CamSegment::Kind;
    auto build = [&](std::vector<CamSegment> segs, CamError& err) {
        CamSpec sp;
        sp.base = 0.03;
        sp.segments = std::move(segs);
        CamDiagram d;
        const bool ok = d.build(sp, err);
        return ok;
    };
    CamError err;

    // a good diagram: values at known angles; dwells are exactly flat
    {
        CamSpec sp;
        sp.base = 0.03;
        sp.segments = {seg(K::Rise, 0.012, 100), seg(K::Dwell, 0, 60),
                       seg(K::Return, 0, 140), seg(K::Dwell, 0, 0, true)};
        CamDiagram d;
        CHECK(d.build(sp, err));
        CHECK_APPROX(d.maxLift(), 0.012, 1e-15);
        double s, ds, dds;
        d.eval(deg2rad(50.0), s, ds, dds);               // cycloidal midpoint
        CHECK_APPROX(s, 0.006, 1e-12);
        CHECK_APPROX(ds, 2.0 * 0.012 / deg2rad(100.0), 1e-9);   // peak velocity 2h/beta
        CHECK_APPROX(dds, 0.0, 1e-9);
        for (double deg : {101.0, 130.0, 159.9}) {        // dwell at the top
            d.eval(deg2rad(deg), s, ds, dds);
            CHECK(s == 0.012 && ds == 0.0 && dds == 0.0);
        }
        for (double deg : {301.0, 330.0, 359.9}) {        // closing dwell at the base
            d.eval(deg2rad(deg), s, ds, dds);
            CHECK(s == 0.0 && ds == 0.0 && dds == 0.0);
        }
        d.eval(deg2rad(10.0 + 360.0), s, ds, dds);        // periodic
        double s2, a2, b2;
        d.eval(deg2rad(10.0), s2, a2, b2);
        CHECK_APPROX(s, s2, 1e-15);
        d.eval(deg2rad(-350.0), s, ds, dds);
        CHECK_APPROX(s, s2, 1e-12);
    }

    // errors name the problem
    auto fails = [&](std::vector<CamSegment> segs, const char* needle, int segIdx = -2) {
        CamError e;
        CHECK(!build(std::move(segs), e));
        CHECK(e.msg.find(needle) != std::string::npos);
        if (segIdx != -2) CHECK(e.segment == segIdx);
    };
    fails({}, "no motion segments");
    fails({seg(K::Rise, 0.01, 180), seg(K::Dwell, 0, 0, true)}, "does not close");
    fails({seg(K::Rise, 0.01, 100), seg(K::Return, 0, 100)}, "200 of 360");   // not a full turn
    fails({seg(K::Rise, 0.01, 200), seg(K::Return, 0, 200)}, "more than one turn");
    fails({seg(K::Rise, 0.01, 100), seg(K::Fall, 0.02, 100), seg(K::Dwell, 0, 0, true)},
          "below the base circle", 1);
    fails({seg(K::Return, 0, 100), seg(K::Dwell, 0, 0, true)}, "nothing to return from", 0);
    fails({seg(K::Rise, 0.0, 100), seg(K::Dwell, 0, 0, true)}, "positive lift", 0);
    fails({seg(K::Dwell, 0, 0, true), seg(K::Rise, 0.01, 100)}, "must be the last", 0);
    fails({seg(K::Rise, 0.01, 180), seg(K::Return, 0, 180), seg(K::Dwell, 0, 0, true)},
          "no angle left", 2);
    fails({seg(K::Rise, 0.01, -5), seg(K::Dwell, 0, 0, true)}, "positive angle", 0);
}

void testCamDesign() {
    std::printf("[cam design M6b]\n");

    using K = CamSegment::Kind;
    auto seg = [](K k, double lift, double deg, bool rest = false) {
        CamSegment c;
        c.kind = k;
        c.lift = lift;
        c.angle = deg2rad(deg);
        c.restOfTurn = rest;
        return c;
    };

    // zero lift: the pitch curve is the prime circle, so its radius of
    // curvature is exactly base + roller and the pressure angle is zero
    {
        CamSpec sp;
        sp.base = 0.03;
        sp.roller = 0.006;
        sp.segments = {seg(K::Dwell, 0, 0, true)};
        CamDiagram d;
        CamError e;
        CHECK(d.build(sp, e));
        const CamDesign cd = designCam(sp, d);
        CHECK_APPROX(cd.minConvexRadius, 0.036, 1e-9);
        CHECK_APPROX(cd.maxPressure, 0.0, 1e-15);
        CHECK(!cd.undercut);
        // the surface is the base circle (offset in by the roller)
        double worst = 0.0;
        for (const Vec2& p : cd.surface) worst = std::max(worst, std::fabs(vlen(p) - 0.03));
        CHECK(worst < 1e-12);
    }

    // pressure angle against a hand calculation: a lone cycloidal rise, then
    // maximize atan((h/beta)(1 - cos 2 pi x) / (r0 + h(x - sin(2 pi x)/2pi))) over x
    {
        const double h = 0.012, beta = deg2rad(100.0), r0 = 0.031;
        CamSpec sp;
        sp.base = 0.025;
        sp.roller = 0.006;
        sp.segments = {seg(K::Rise, h, 100), seg(K::Dwell, 0, 60), seg(K::Return, 0, 100),
                       seg(K::Dwell, 0, 0, true)};
        sp.segments[2].law = MotionLaw::Cycloidal;
        CamDiagram d;
        CamError e;
        CHECK(d.build(sp, e));
        const CamDesign cd = designCam(sp, d);
        double hand = 0.0, handX = 0.0;
        for (int k = 0; k <= 2000000; ++k) {
            const double x = k / 2000000.0;
            const double a = std::atan((h / beta) * (1.0 - std::cos(2.0 * kPi * x)) /
                                       (r0 + h * (x - std::sin(2.0 * kPi * x) / (2.0 * kPi))));
            if (a > hand) { hand = a; handX = x; }
        }
        CHECK_APPROX(cd.maxPressure, hand, 1e-6);
        // rise and return mirror each other, so the maximum occurs twice: on the
        // rise's early half (smaller radius) and the return's late half; either
        // may be reported. Return starts at 160 deg and is 100 deg long.
        const double at = rad2deg(cd.maxPressureAt);
        const double riseAt = rad2deg(handX * beta);
        const double returnAt = 160.0 + 100.0 - riseAt;
        CHECK(std::fabs(at - riseAt) < 0.06 || std::fabs(at - returnAt) < 0.06);   // 0.05 deg grid
    }

    // radius of curvature against the polar formula
    //   kappa = (r^2 + 2 r'^2 - r r'') / (r^2 + r'^2)^(3/2),  r = r0 + s(theta)
    // (an independent route from the vector formula used by the synthesizer)
    {
        CamSpec sp;
        sp.base = 0.025;
        sp.roller = 0.006;
        sp.segments = {seg(K::Rise, 0.012, 100), seg(K::Dwell, 0, 60),
                       seg(K::Return, 0, 140), seg(K::Dwell, 0, 0, true)};
        sp.segments[2].law = MotionLaw::ModTrap;
        CamDiagram d;
        CamError e;
        CHECK(d.build(sp, e));
        const CamDesign cd = designCam(sp, d);
        double minR = 1e9;
        for (int k = 0; k < 7200; ++k) {
            double s, ds, dds;
            d.eval(2.0 * kPi * k / 7200.0, s, ds, dds);
            const double r = 0.031 + s;
            const double kappa = (r * r + 2.0 * ds * ds - r * dds) /
                                 std::pow(r * r + ds * ds, 1.5);
            if (kappa > 1e-12) minR = std::min(minR, 1.0 / kappa);
        }
        CHECK_APPROX(cd.minConvexRadius, minR, 1e-9);
        CHECK(cd.minConvexRadius > 0.006 && !cd.undercut);
    }

    // a tight lobe under a big roller undercuts
    {
        CamSpec sp;
        sp.base = 0.02;
        sp.roller = 0.015;
        sp.segments = {seg(K::Rise, 0.02, 40), seg(K::Return, 0, 40), seg(K::Dwell, 0, 0, true)};
        CamDiagram d;
        CamError e;
        CHECK(d.build(sp, e));
        CHECK(designCam(sp, d).undercut);
    }
}

void testParserModelM6b() {
    std::printf("[parser+model M6b]\n");

    // parse: every item, host either side, 'cam' still a fine body name
    {
        const Program p = parseText(
            "mechanism m { ground g { anchor a at (0,0) } shaft cam at g.a "
            "cam lobe { base 0.03 roller 0.005 start 100 deg turn cw "
            "rise 0.01 over 90 deg harmonic fall 0.004 over 45 deg dwell 10 deg "
            "return over 100 deg modtrap dwell } on cam }");
        const CamDecl& c = p.mechanisms[0].cams[0];
        CHECK(c.name == "lobe" && c.host == "cam" && c.hostAnchor.empty());
        CHECK(c.hasBase);
        CHECK_APPROX(c.spec.base, 0.03, 1e-15);
        CHECK_APPROX(c.spec.roller, 0.005, 1e-15);
        CHECK_APPROX(c.spec.startRad, deg2rad(100.0), 1e-12);
        CHECK(c.spec.turn == -1);
        CHECK(c.spec.segments.size() == 5 && c.segLines.size() == 5);
        CHECK(c.spec.segments[0].kind == CamSegment::Kind::Rise);
        CHECK(c.spec.segments[0].law == MotionLaw::Harmonic);
        CHECK(c.spec.segments[1].kind == CamSegment::Kind::Fall);
        CHECK(c.spec.segments[1].law == MotionLaw::Cycloidal);       // the default
        CHECK(!c.spec.segments[2].restOfTurn);
        CHECK_APPROX(c.spec.segments[2].angle, deg2rad(10.0), 1e-12);
        CHECK(c.spec.segments[3].kind == CamSegment::Kind::Return);
        CHECK(c.spec.segments[3].law == MotionLaw::ModTrap);
        CHECK(c.spec.segments[4].restOfTurn);
    }

    auto expectParseError = [&](const char* src, const char* needle) {
        bool threw = false;
        try {
            parseText(src);
        } catch (const ParseError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
            CHECK(e.line >= 1);
        }
        CHECK(threw);
    };
    expectParseError("mechanism m { cam c on s { rise 0.01 over 90 deg bouncy } }", "unknown motion law 'bouncy'");
    expectParseError("mechanism m { cam c on s { rise 0.01 over 90 } }", "expected 'deg'");
    expectParseError("mechanism m { cam c on s { rise 0.01 90 deg } }", "expected 'over'");
    expectParseError("mechanism m { cam c on s { turn sideways } }", "'ccw' or 'cw'");
    expectParseError("mechanism m { cam c on s { jump 3 } }", "expected cam item");
    expectParseError("mechanism m { cam c { base 1 } }", "needs a host");
    expectParseError("mechanism m { cam c on s { start 90 } }", "expected 'deg'");

    // model
    auto expectModelError = [&](const std::string& body, const char* needle, int line = -1) {
        bool threw = false;
        try {
            buildFromText(plateCamSrc(body, "30").c_str());
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
            if (line >= 0) CHECK(e.line == line);
        }
        CHECK(threw);
    };
    expectModelError("rise 0.01 over 100 deg dwell", "needs a 'base'");
    expectModelError("base 0 rise 0.01 over 100 deg return over 100 deg dwell", "'base' must be positive");
    expectModelError("base 0.03 roller -1 rise 0.01 over 100 deg return over 100 deg dwell", "cannot be negative");
    expectModelError("base 0.03 rise 0.01 over 100 deg", "100 of 360");
    expectModelError("base 0.03 rise 0.01 over 100 deg dwell", "does not close");
    expectModelError("base 0.02 roller 0.015 rise 0.02 over 40 deg return over 40 deg dwell", "undercut");

    // the segment's own source line is reported (line 3 of the spliced source)
    {
        bool threw = false;
        try {
            buildFromText(
                "mechanism m {\n  ground g { anchor a at (0,0) }\n  shaft cam at g.a\n"
                "  cam lobe on cam.A {\n    base 0.03\n    return over 100 deg\n    dwell\n  }\n}");
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(e.line == 6);
            CHECK(std::string(e.msg).find("cam 'lobe'") != std::string::npos);
        }
        CHECK(threw);
    }

    // shared namespace, host resolution
    {
        bool threw = false;
        try {
            buildFromText(
                "mechanism m { ground g { anchor a at (0,0) } shaft cam at g.a "
                "profile lobe on cam { circle: 0.03 } "
                "cam lobe on cam { base 0.03 rise 0.01 over 90 deg return over 90 deg dwell } }");
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find("duplicate profile name") != std::string::npos);
        }
        CHECK(threw);
        threw = false;
        try {
            buildFromText("mechanism m { ground g { anchor a at (0,0) } "
                          "cam lobe on nope { base 0.03 rise 0.01 over 90 deg return over 90 deg dwell } }");
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find("cam 'lobe' refers to unknown host body") != std::string::npos);
        }
        CHECK(threw);
    }

    // what the model records; a touch follows the cam's roller by default
    {
        Model m = buildFromText(plateCamSrc(kCamBody, "30").c_str());
        CHECK(m.cams.size() == 1 && m.profiles.size() == 1);
        CHECK(m.profiles[0].fromCam);
        CHECK_APPROX(m.profiles[0].designRoller, 0.006, 1e-15);
        CHECK_APPROX(m.touches[0].roller, 0.006, 1e-15);
        CHECK_APPROX(m.cams[0].maxLift, 0.012, 1e-15);
        CHECK(m.cams[0].maxPressure > 0.0 && m.cams[0].minConvexRadius > 0.006);
        CHECK(m.dofReport().residualDof == 0);
        std::vector<double> q;
        CHECK(m.assemble(q).empty());        // no roller-mismatch warning

        // an explicit, different roller still works but is flagged at assembly
        Model mm = buildFromText(plateCamSrc(kCamBody, "30", "roller 0.01").c_str());
        CHECK_APPROX(mm.touches[0].roller, 0.01, 1e-15);
        const std::vector<std::string> w = mm.assemble(q);
        CHECK(w.size() == 1 && w[0].find("was cut for") != std::string::npos);
        // an explicit matching roller is fine
        Model m3 = buildFromText(plateCamSrc(kCamBody, "30", "roller 0.006").c_str());
        CHECK(m3.assemble(q).empty());
    }
}

// y of the follower centre over two turns vs r0 + s(delta), delta the diagram
// angle the follower currently sits on. Returns the worst deviation; also the
// worst pressure-angle error against atan(|s'|/r) read from the real contact.
void camFollowsDiagram(const std::string& src, double rateDegPerSec, double* worstY,
                       double* worstPressure, double* worstRes) {
    Model m = buildFromText(src.c_str());
    const CamSpec spec = m.cams[0].spec;
    CamDiagram dia;
    CamError ce;
    dia.build(spec, ce);
    ConstraintSystem sys(m);
    std::vector<double> q;
    m.assemble(q);
    int iters = 0;
    double res = 0.0;
    sys.solve(q, 0.0, iters, &res);
    const int ifol = m.bodyIndex("follower"), icam = m.bodyIndex("cam");
    const double r0 = spec.base + spec.roller;
    const double sg = (double)spec.turn;
    *worstY = *worstPressure = *worstRes = 0.0;
    const double dt = 5.0 / std::fabs(rateDegPerSec);
    for (int k = 0; k <= 144; ++k) {
        const double t = k * dt;
        sys.solve(q, t, iters, &res);
        *worstRes = std::max(*worstRes, res);
        const double thCam = q[(size_t)m.bodies[icam].qOffset];
        // the follower sits on rail angle 90 deg; in the cam frame that is 90 - thCam
        const double delta = sg * (spec.startRad - (kPi / 2.0 - thCam));
        double s, ds, dds;
        dia.eval(delta, s, ds, dds);
        *worstY = std::max(*worstY, std::fabs(q[(size_t)m.bodies[ifol].qOffset + 2] - (r0 + s)));
        // pressure angle from the actual contact normal vs the rail (+y)
        const ContactState cs = m.touchState(m.touches[0], q);
        const double alphaGeom = std::fabs(std::atan2(cs.normal.x, cs.normal.y));
        const double alphaDiag = std::atan2(std::fabs(ds), r0 + s);
        *worstPressure = std::max(*worstPressure, std::fabs(alphaGeom - alphaDiag));
    }
}

void testKinematicsM6b() {
    std::printf("[kinematics M6b]\n");

    // ---- the follower follows the diagram, for every law -------------------------
    const char* laws[] = {"cycloidal", "harmonic", "modtrap"};
    for (const char* law : laws) {
        const std::string body = std::string(
            "base 0.025 roller 0.006 rise 0.012 over 100 deg ") + law +
            " dwell 60 deg return over 140 deg " + law + " dwell";
        double wy, wp, wr;
        camFollowsDiagram(plateCamSrc(body, "30"), 30.0, &wy, &wp, &wr);
        CHECK(wy < 1e-8);       // the surface is a 720-point spline: error ~1e-11
        // The contact normal agrees with tan(alpha) = s'/r. Harmonic motion has
        // a jump in acceleration at each segment end, so the true surface has a
        // curvature discontinuity there that a C2 spline cannot hold: the normal
        // is off by ~0.045 deg at those two junctions (positions are not).
        CHECK(wp < (std::string(law) == "harmonic" ? 2e-3 : 1e-6));
        CHECK(wr < 1e-9);
    }

    // knife-edge follower (no roller): the surface IS the pitch curve
    {
        double wy, wp, wr;
        camFollowsDiagram(plateCamSrc(
            "base 0.03 rise 0.01 over 120 deg dwell 60 deg return over 120 deg dwell", "30"),
            30.0, &wy, &wp, &wr);
        CHECK(wy < 1e-8 && wp < 1e-6 && wr < 1e-9);
    }

    // a cam driven the other way ('turn cw', negative rate) and a rotated start
    {
        double wy, wp, wr;
        camFollowsDiagram(plateCamSrc(
            "base 0.025 roller 0.006 turn cw rise 0.012 over 100 deg dwell 60 deg "
            "return over 140 deg dwell", "-30"), -30.0, &wy, &wp, &wr);
        CHECK(wy < 1e-8 && wp < 1e-6 && wr < 1e-9);
        camFollowsDiagram(plateCamSrc(
            "base 0.025 roller 0.006 start 150 deg rise 0.012 over 100 deg dwell 60 deg "
            "return over 140 deg dwell", "30"), 30.0, &wy, &wp, &wr);
        CHECK(wy < 1e-8 && wp < 1e-6 && wr < 1e-9);
    }

    // the dwell is exactly flat: 60 degrees of cam rotation, the follower does not move
    {
        Model m = buildFromText(plateCamSrc(kCamBody, "30").c_str());
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        const int ifol = m.bodyIndex("follower");
        double lo = 1e9, hi = -1e9;
        for (int k = 0; k <= 50; ++k) {                         // 105 .. 155 degrees
            sys.solve(q, (105.0 + k) / 30.0, iters, &res);
            const double y = q[(size_t)m.bodies[ifol].qOffset + 2];
            lo = std::min(lo, y);
            hi = std::max(hi, y);
        }
        CHECK(hi - lo < 1e-9);
        CHECK_APPROX(hi, 0.025 + 0.006 + 0.012, 1e-8);
    }

    // the shipped example
    {
        std::ifstream in("examples/platecam.mech");
        if (!in) {
            std::fprintf(stderr, "note: examples/platecam.mech not found (cwd?)\n");
        } else {
            std::stringstream ss;
            ss << in.rdbuf();
            Model m = buildModel(parse(lex(ss.str())).mechanisms[0]);
            CHECK(m.cams.size() == 1);
            CHECK(m.dofReport().residualDof == 0);
            CHECK(m.cams[0].maxPressure < deg2rad(30.0));
            ConstraintSystem sys(m);
            std::vector<double> q;
            CHECK(m.assemble(q).empty());
            int iters = 0;
            double res = 0.0;
            CHECK(sys.solve(q, 0.0, iters, &res));
            CHECK(m.validateAssembly(q).empty());
            const int ifol = m.bodyIndex("follower");
            double lo = 1e9, hi = -1e9, worst = 0.0;
            for (int k = 1; k <= 480; ++k) {
                sys.solve(q, k * 0.0125, iters, &res);
                worst = std::max(worst, res);
                const double y = q[(size_t)m.bodies[ifol].qOffset + 2];
                lo = std::min(lo, y);
                hi = std::max(hi, y);
            }
            CHECK(worst < 1e-8);
            CHECK_APPROX(lo, 0.031, 1e-8);
            CHECK_APPROX(hi, 0.043, 1e-8);
        }
    }
}


// ---- M6c: unilateral contact in the dynamics ------------------------------------

// The second-order contact term, checked the hard way: from a state on the
// constraint manifold with a consistent velocity, advance
//   q(h) = q + h qd + h^2/2 qdd          (qdd from the dynamics)
// and look at the constraint residual with its exact first-order part removed
// (that part is J qd - nu, which the damped velocity projection only satisfies
// to ~1e-5 in ill-conditioned poses; it is not what is under test). If every
// row's bias is right the true trajectory stays on the manifold to second
// order, so what remains is O(h^3), ~1e-10 here; a bias wrong by delta leaves
// delta h^2 / 2 = 2e-6 delta at h = 2e-3 — thousands of times more.
// Returns the worst remainder over all rows and trials.
double biasDrift(const char* src, int* trials) {
    Model m = buildFromText(src);
    ConstraintSystem sys(m);
    DynamicsSystem dyn(m, sys);
    std::vector<double> q;
    m.assemble(q);
    for (const TouchJoint& tj : m.touches) tj.engaged = true;
    int iters = 0;
    double res = 0.0;
    unsigned seed = 99;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return ((seed >> 8) & 0xFFFF) / 65535.0 - 0.5;
    };
    double worst = 0.0;
    *trials = 0;
    for (int k = 0; k < 14; ++k) {
        const double t = 0.045 * k;
        if (!sys.solve(q, t, iters, &res)) continue;
        std::vector<double> qd((size_t)m.nq());
        for (double& v : qd) v = 6.0 * rnd();
        dyn.projectVelocities(q, qd);
        std::vector<double> qdd;
        dyn.accelerations(q, qd, qdd);
        // exact first-order part of the residual along the velocity path
        std::vector<double> c0, cp, cm, c1;
        sys.residuals(q, t, c0);
        {
            const double e = 1e-6;
            std::vector<double> qp(q.size()), qm(q.size());
            for (size_t i = 0; i < q.size(); ++i) { qp[i] = q[i] + e * qd[i]; qm[i] = q[i] - e * qd[i]; }
            sys.residuals(qp, t + e, cp);
            sys.residuals(qm, t - e, cm);
            c1.resize(cp.size());
            for (size_t i = 0; i < cp.size(); ++i) c1[i] = (cp[i] - cm[i]) / (2.0 * e);
        }
        auto drift = [&](double h) {
            std::vector<double> qh(q.size()), c;
            for (size_t i = 0; i < q.size(); ++i) qh[i] = q[i] + h * qd[i] + 0.5 * h * h * qdd[i];
            sys.residuals(qh, t + h, c);
            double mx = 0.0;
            for (size_t i = 0; i < c.size(); ++i)
                mx = std::max(mx, std::fabs(c[i] - c0[i] - h * c1[i]));
            return mx;
        };
        double speed = 0.0;
        for (double v : qd) speed = std::max(speed, std::fabs(v));
        if (speed < 0.5) continue;                       // a state at rest proves nothing
        worst = std::max(worst, drift(2e-3));
        ++*trials;
    }
    return worst;
}

// The same trajectory test as biasDrift, for a mechanism whose third derivatives are
// large (the polar mesh's ratio swings 9:1): there the O(h^3) truncation term is far
// above the solver's noise floor, so an absolute bound is the wrong instrument. What
// tells a correct bias from a wrong one is how the drift scales: a correct bias leaves
// only the h^3 term (halving h cuts the drift 8x), a wrong one leaves an h^2 term (4x).
// Returns the smallest 2h/h drift ratio over the trials (expect ~8; < ~5 means a bias error).
double biasScalingRatio(const char* src, int* trials) {
    Model m = buildFromText(src);
    ConstraintSystem sys(m);
    DynamicsSystem dyn(m, sys);
    std::vector<double> q;
    m.assemble(q);
    int iters = 0;
    double res = 0.0;
    unsigned seed = 99;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return ((seed >> 8) & 0xFFFF) / 65535.0 - 0.5;
    };
    double minRatio = 1e9;
    *trials = 0;
    for (int k = 0; k < 14; ++k) {
        const double t = 0.045 * k;
        if (!sys.solve(q, t, iters, &res)) continue;
        std::vector<double> qd((size_t)m.nq());
        for (double& v : qd) v = 6.0 * rnd();
        dyn.projectVelocities(q, qd);
        std::vector<double> qdd;
        dyn.accelerations(q, qd, qdd);
        std::vector<double> c0, cp, cm, c1;
        sys.residuals(q, t, c0);
        {
            const double e = 1e-6;
            std::vector<double> qp(q.size()), qm(q.size());
            for (size_t i = 0; i < q.size(); ++i) { qp[i] = q[i] + e * qd[i]; qm[i] = q[i] - e * qd[i]; }
            sys.residuals(qp, t + e, cp);
            sys.residuals(qm, t - e, cm);
            c1.resize(cp.size());
            for (size_t i = 0; i < cp.size(); ++i) c1[i] = (cp[i] - cm[i]) / (2.0 * e);
        }
        auto drift = [&](double h) {
            std::vector<double> qh(q.size()), c;
            for (size_t i = 0; i < q.size(); ++i) qh[i] = q[i] + h * qd[i] + 0.5 * h * h * qdd[i];
            sys.residuals(qh, t + h, c);
            double mx = 0.0;
            for (size_t i = 0; i < c.size(); ++i)
                mx = std::max(mx, std::fabs(c[i] - c0[i] - h * c1[i]));
            return mx;
        };
        const double big = drift(2e-3), small = drift(1e-3);
        if (big < 1e-8) continue;                    // too near the noise floor to read a scaling
        minRatio = std::min(minRatio, big / small);
        ++*trials;
    }
    return minRatio;
}

void testContactDynamics() {
    std::printf("[dynamics M6c]\n");

    // ---- the bias term -----------------------------------------------------------
    {
        std::string oval =
            "mechanism swing {\n"
            "  world { gravity: 9.81 }\n"
            "  ground frame { anchor axle at (0, 0)  anchor pivot at (0.12, 0.06) }\n"
            "  body cam { anchor axle at (0, 0)  anchor c at (0.008, 0.004) }\n"
            "  pin cam.axle -> frame.axle\n"
            "  profile oval on cam.c { polar:";
        for (int i = 0; i < 36; ++i) {
            const double a = i * 10.0;
            char b[80];
            std::snprintf(b, sizeof b, " (%g deg, %.8f)", a,
                          0.03 + 0.006 * std::cos(2.0 * deg2rad(a)) + 0.002 * std::sin(deg2rad(a)));
            oval += b;
        }
        oval += " }\n"
                "  rod arm on frame.pivot { length: 0.12 }\n"
                "  touch arm.B on oval roller 0.005\n"
                "  drive cam at 45 deg/s\n}\n";
        int n = 0;
        const double drift1 = biasDrift(oval.c_str(), &n);
        CHECK(n >= 5);
        CHECK(drift1 < 1e-8);

        // the profile rides a swinging arm, touched by a driven crank tip: the
        // foot slides while both bodies turn about different pivots
        std::string flip =
            "mechanism flip {\n"
            "  world { gravity: 9.81 }\n"
            "  ground frame { anchor p1 at (0, 0)  anchor p2 at (0.07, 0.02) }\n"
            "  rod crank on frame.p1 { length: 0.05 }\n"
            "  body arm { anchor piv at (0, 0)  anchor c at (0.02, 0.005) }\n"
            "  pin arm.piv -> frame.p2\n"
            "  profile blob on arm.c { points:";
        for (int i = 0; i < 40; ++i) {
            const double a = i * 2.0 * kPi / 40.0;
            char b[80];
            std::snprintf(b, sizeof b, " (%.8f, %.8f)", 0.035 * std::cos(a), 0.022 * std::sin(a));
            flip += b;
        }
        flip += " }\n"
                "  touch crank.B on blob roller 0.004\n"
                "  drive crank at 60 deg/s\n}\n";
        const double drift2 = biasDrift(flip.c_str(), &n);
        CHECK(n >= 5);
        CHECK(drift2 < 1e-8);
    }

    // ---- a ball dropped on a fixed circle: Newton restitution, in closed form --------
    // The ball (1 kg) rides a vertical rail through the top of a ground circle and
    // starts h0 = 0.1 m above it, at rest. It lands at v = sqrt(2 g h0) and leaves at
    // e v: the k-th rebound peaks at e^(2k) h0, and each impact dissipates
    // (1 - e^2) v^2 / 2.
    auto dropSrc = [](double e) {
        char b[600];
        std::snprintf(b, sizeof b,
            "mechanism drop {\n"
            "  world { gravity: 9.81  step: 0.0005 }\n"
            "  ground frame { anchor c at (0, 0)  anchor rail at (0, 0.30) dir (0, 1) }\n"
            "  profile floor on frame.c { circle: 0.2 }\n"
            "  body ball { anchor tip at (0, 0) dir (1, 0)  mass: 1 }\n"
            "  slide ball.tip -> frame.rail\n"
            "  touch ball.tip on floor restitution %g\n}\n", e);
        return std::string(b);
    };
    struct DropResult {
        double apex[8] = {0, 0, 0, 0, 0, 0, 0, 0};   // peak height above the floor between impacts
        double lossAfterFirst = 0.0, ledger = 0.0, totalLoss = 0.0, finalForce = 0.0, finalY = 0.0;
        double firstImpactT = 0.0;
        bool engagedAtEnd = false;
        bool startedInFlight = false;
        long impacts = 0;
    };
    auto drop = [&](double e, double T) {
        DropResult r;
        Model m = buildFromText(dropSrc(e).c_str());
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        r.startedInFlight = !m.touches[0].engaged;
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        const int ib = m.bodyIndex("ball");
        const double h = m.world.step;
        r.apex[0] = q[(size_t)m.bodies[ib].qOffset + 2] - 0.2;   // where it starts
        long seen = 0;
        for (long k = 0; k < (long)std::llround(T / h); ++k) {
            dyn.step(q, qd, k * h);
            const long n = dyn.hybridStats().impacts;
            if (n >= 1 && seen == 0) {
                seen = 1;
                r.lossAfterFirst = dyn.hybridStats().impactLoss;
                r.firstImpactT = (k + 1) * h;
            }
            if (n < 8) {
                const double y = q[(size_t)m.bodies[ib].qOffset + 2] - 0.2;
                r.apex[n] = std::max(r.apex[n], y);
            }
        }
        const HybridStats& hs = dyn.hybridStats();
        const double dE = dyn.kinetic(q, qd) + dyn.potential(q) - E0;
        r.ledger = dE - (hs.driveWork - hs.impactLoss);
        r.totalLoss = hs.impactLoss;
        r.impacts = hs.impacts;
        r.engagedAtEnd = m.touches[0].engaged;
        std::vector<double> qdd;
        dyn.accelerations(q, qd, qdd);
        r.finalForce = dyn.contactForces()[0];
        r.finalY = q[(size_t)m.bodies[ib].qOffset + 2];
        return r;
    };
    {
        const double e = 0.5, h0 = 0.1;
        const DropResult r = drop(e, 1.0);
        CHECK(r.startedInFlight);                                  // placed 0.1 m above: starts in flight
        CHECK_APPROX(r.firstImpactT, std::sqrt(2.0 * h0 / 9.81), 6e-4);
        CHECK_APPROX(r.apex[0], h0, 1e-12);
        CHECK_APPROX(r.apex[1], e * e * h0, 2e-5);                 // 0.025
        CHECK_APPROX(r.apex[2], std::pow(e, 4) * h0, 2e-5);        // 0.00625
        CHECK_APPROX(r.apex[3], std::pow(e, 6) * h0, 2e-5);        // 0.0015625
        CHECK_APPROX(r.lossAfterFirst, 9.81 * h0 * (1.0 - e * e), 2e-3);   // (1 - e^2) m g h0
        // it comes to rest on the circle, held by exactly its weight
        CHECK(r.engagedAtEnd);
        CHECK_APPROX(r.finalForce, 9.81, 1e-4);
        CHECK_APPROX(r.finalY, 0.2, 1e-8);
        // every joule it started with above the floor went into impacts
        CHECK_APPROX(r.totalLoss, 9.81 * h0, 2e-3);
        CHECK(std::fabs(r.ledger) < 2e-4);
        CHECK(r.impacts >= 9 && r.impacts <= 13);                  // a finite Zeno sequence: 11
    }
    {
        // perfectly elastic: it climbs back to where it started, nothing is lost
        const DropResult r = drop(1.0, 0.62);      // long enough to see the second peak (t = 0.571)
        CHECK_APPROX(r.apex[1], 0.1, 3e-4);
        CHECK_APPROX(r.apex[2], 0.1, 6e-4);
        CHECK(r.totalLoss < 2e-3);
        CHECK(std::fabs(r.ledger) < 2e-3);
        // perfectly plastic: no bounce at all, the whole drop is dissipated
        const DropResult p = drop(0.0, 0.6);
        CHECK(p.impacts == 1);
        CHECK_APPROX(p.apex[1], 0.0, 2e-6);
        CHECK(p.engagedAtEnd);
        CHECK_APPROX(p.finalForce, 9.81, 1e-4);
        CHECK_APPROX(p.totalLoss, 9.81 * 0.1, 2e-3);
    }

    // ---- a spring-loaded follower on a plate cam ---------------------------------------
    // 100 g follower, spring k to the axle (rest 0.020 m), gravity, the platecam
    // diagram. While the contact holds, the follower is exactly on the diagram
    // y = r0 + s(theta), so Newton fixes the normal force in closed form:
    //   N cos(alpha) = m (omega^2 s'' + g) + k (y - rest),   cos(alpha) = y / sqrt(y^2 + s'^2)
    // and the cam lets go where that N reaches zero.
    auto jumpSrc = [](double rpm, double railY) {
        char b[1200];
        std::snprintf(b, sizeof b,
            "mechanism jump {\n"
            "  world { gravity: 9.81  step: 0.0005 }\n"
            "  ground frame { anchor axle at (0, 0)  anchor rail at (0, %g) dir (0, 1) }\n"
            "  shaft cam at frame.axle\n"
            "  cam lobe on cam.A { base 0.025 roller 0.006 rise 0.012 over 100 deg cycloidal\n"
            "    dwell 60 deg return over 140 deg modtrap dwell }\n"
            "  body follower { anchor tip at (0, 0) dir (1, 0)  mass: 0.1 }\n"
            "  slide follower.tip -> frame.rail\n"
            "  spring preload follower.tip -> frame.axle { k: 200  rest: 0.020 }\n"
            "  touch follower.tip on lobe restitution 0.3\n"
            "  drive cam at %g rpm\n}\n", railY, rpm);
        return std::string(b);
    };
    // closed-form contact force on the diagram at cam angle theta (rad), speed omega
    auto diagramForce = [](const CamDiagram& dia, double theta, double omega) {
        double s, ds, dds;
        dia.eval(theta, s, ds, dds);
        const double y = 0.031 + s;
        const double cosA = y / std::sqrt(y * y + ds * ds);
        return (0.1 * (omega * omega * dds + 9.81) + 200.0 * (y - 0.020)) / cosA;
    };

    {   // slow: the contact never lets go, and its force is the closed form
        Model m = buildFromText(jumpSrc(300.0, 0.031).c_str());
        CHECK(m.touches[0].engaged);                       // placed on the base circle: engaged
        CamDiagram dia;
        CamError ce;
        CHECK(dia.build(m.cams[0].spec, ce));
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const double omega = 300.0 * kPi / 30.0;
        const int ifol = m.bodyIndex("follower"), icam = m.bodyIndex("cam");
        const double h = m.world.step;
        double worstY = 0.0, worstF = 0.0, minF = 1e9;
        bool alwaysEngaged = true;
        for (long k = 0; k < 800; ++k) {                   // 0.4 s = two turns
            dyn.step(q, qd, k * h);
            alwaysEngaged = alwaysEngaged && m.touches[0].engaged;
            const double th = q[(size_t)m.bodies[icam].qOffset];
            double s, ds, dds;
            dia.eval(th, s, ds, dds);
            worstY = std::max(worstY, std::fabs(q[(size_t)m.bodies[ifol].qOffset + 2] - (0.031 + s)));
            if (k % 10 == 0) {
                std::vector<double> qdd;
                dyn.accelerations(q, qd, qdd);
                worstF = std::max(worstF, std::fabs(dyn.contactForces()[0] - diagramForce(dia, th, omega)));
                minF = std::min(minF, dyn.contactForces()[0]);
            }
        }
        CHECK(alwaysEngaged);
        CHECK(dyn.hybridStats().liftoffs == 0 && dyn.hybridStats().impacts == 0);
        CHECK(worstY < 1e-8);                              // exactly on the diagram
        CHECK(worstF < 2e-3);                              // Newton's law, to 2 mN of ~5 N
        CHECK(minF > 0.0);
    }

    {   // fast: it lifts off where the closed form says, flies, lands, repeats
        const double rpm = 600.0, omega = rpm * kPi / 30.0;
        Model m = buildFromText(jumpSrc(rpm, 0.031).c_str());
        CamDiagram dia;
        CamError ce;
        dia.build(m.cams[0].spec, ce);

        // predicted lift-off: first angle in the rise where the engaged force reaches zero
        double thStar = -1.0;
        for (int k = 0; k <= 100000; ++k) {
            const double th = deg2rad(100.0) * k / 100000.0;
            if (diagramForce(dia, th, omega) < 0.0) { thStar = th; break; }
        }
        CHECK(thStar > deg2rad(50.0) && thStar < deg2rad(100.0));

        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        const int ifol = m.bodyIndex("follower"), icam = m.bodyIndex("cam");
        const double h = m.world.step;
        double liftAngle = -1.0, worstEngaged = 0.0, deepest = 0.0, worstForce = 1e9, flyMax = 0.0;
        long impactsAfterTurn1 = 0;
        for (long k = 0; k < 400; ++k) {                   // 0.2 s = two turns
            dyn.step(q, qd, k * h);
            const TouchJoint& tj = m.touches[0];
            const double th = q[(size_t)m.bodies[icam].qOffset];
            const double gap = m.touchState(tj, q).gap - tj.roller;
            if (tj.engaged) {
                worstEngaged = std::max(worstEngaged, std::fabs(gap));
                std::vector<double> qdd;
                dyn.accelerations(q, qd, qdd);
                worstForce = std::min(worstForce, dyn.contactForces()[0]);
            } else {
                deepest = std::min(deepest, gap);          // flight never digs into the cam
                flyMax = std::max(flyMax, gap);
                if (liftAngle < 0.0) liftAngle = th;
            }
            if (k == 199) impactsAfterTurn1 = dyn.hybridStats().impacts;
        }
        const HybridStats& hs = dyn.hybridStats();
        // lifted off where Newton said (the step ends up to omega*h = 1.8 deg later)
        CHECK(liftAngle >= thStar - deg2rad(0.05) && liftAngle <= thStar + omega * h + deg2rad(0.05));
        CHECK(hs.liftoffs == 2);                           // once per turn
        CHECK(hs.impacts >= 6);                            // each flight ends in a bounce sequence
        CHECK(hs.impacts == 2 * impactsAfterTurn1);        // and turn 2 repeats turn 1 exactly
        CHECK(worstEngaged < 1e-8);                        // on the curve while engaged
        CHECK(deepest > -2e-6);                            // never through it while flying
        CHECK(flyMax > 1e-3 && flyMax < 5e-3);             // a real flight: ~2 mm clear
        CHECK(worstForce > -1e-6);                         // engaged means pushing, never pulling
        // the ledger: the cam's work equals what the impacts discarded, and the
        // state is back where it began after two whole turns
        const double dE = dyn.kinetic(q, qd) + dyn.potential(q) - E0;
        CHECK(std::fabs(dE - (hs.driveWork - hs.impactLoss)) < 5e-5);
        CHECK(hs.driveWork > 0.02);
        CHECK_APPROX(hs.impactLoss, hs.driveWork, 5e-5);
        CHECK(std::fabs(dE) < 1e-4);
        (void)ifol;
    }

    {   // determinism: two identical runs agree to the last bit
        auto run = [&](std::vector<double>& qOut, HybridStats& stOut) {
            Model m = buildFromText(jumpSrc(600.0, 0.031).c_str());
            ConstraintSystem sys(m);
            DynamicsSystem dyn(m, sys);
            std::vector<double> q, qd((size_t)m.nq(), 0.0);
            m.assemble(q);
            int iters = 0;
            double res = 0.0;
            sys.solve(q, 0.0, iters, &res);
            dyn.projectVelocities(q, qd);
            for (long k = 0; k < 300; ++k) dyn.step(q, qd, k * m.world.step);
            qOut = q;
            stOut = dyn.hybridStats();
        };
        std::vector<double> qa, qb;
        HybridStats sa, sb;
        run(qa, sa);
        run(qb, sb);
        CHECK(qa == qb);
        CHECK(sa.liftoffs == sb.liftoffs && sa.impacts == sb.impacts);
        CHECK(sa.impactLoss == sb.impactLoss && sa.driveWork == sb.driveWork);
    }

    {   // initial state: on the curve -> engaged; placed in the air -> in flight; kinematic -> bilateral
        Model on = buildFromText(jumpSrc(600.0, 0.031).c_str());
        std::vector<double> q;
        on.assemble(q);
        CHECK(on.touches[0].engaged);
        Model air = buildFromText(jumpSrc(600.0, 0.05).c_str());
        air.assemble(q);
        CHECK(!air.touches[0].engaged);
        CHECK(air.validateAssembly(q).empty());            // starting in flight is not an error
        Model kin = buildFromText(plateCamSrc(kCamBody, "30").c_str());
        kin.assemble(q);
        CHECK(kin.touches[0].engaged);
    }
}


// ---- M6d: non-circular gear pairs ---------------------------------------------------

// Two identical ellipses (a = 50 mm, e = 0.5) pivoted at their foci, shafts 2a apart.
const char* kEllipB = "0.04330127018922193";     // a sqrt(1 - e^2)
std::string ellipSrc(const char* driveLine, const char* extra = "") {
    std::string s =
        "mechanism ep {\n"
        "  ground frame { anchor pa at (0, 0)  anchor pb at (0.1, 0) }\n"
        "  shaft a at frame.pa\n"
        "  shaft b at frame.pb\n";
    s += std::string("  profile ga on a.A { ellipse: 0.05 ") + kEllipB + " focus }\n";
    s += std::string("  profile gb on b.A { ellipse: 0.05 ") + kEllipB + " focus }\n";
    s += "  mesh ga gb\n  ";
    s += driveLine;
    s += "\n  ";
    s += extra;
    s += "\n}\n";
    return s;
}

// Where profile `pi` crosses the line from its own centre toward `toward`,
// found by brute force on the curve itself (nothing from the mesh row is used).
struct EdgeHit { double s; Vec2 P, tangent; };
EdgeHit hitOnLine(const Model& m, int pi, double th, Vec2 center, Vec2 toward) {
    const ProfileCurve& cv = *m.profiles[(size_t)pi].curve;
    const Vec2 u = normalize(toward - center);
    auto f = [&](double s, Vec2& P, Vec2& T) {
        Vec2 p, d1, d2;
        cv.eval(s, p, d1, d2);
        const Vec2 w = rotate(p, th);
        P = center + w;
        T = rotate(d1, th);
        return std::make_pair(crossz(u, w), dot(u, w));
    };
    // The scan starts off s = 0: at some poses the crossing lies exactly on s = 0,
    // where "sign change" never fires (f is exactly zero there).
    const int N = 20000;
    const double s0 = 0.1234567;
    double prevS = s0, prevF;
    Vec2 P, T;
    prevF = f(s0, P, T).first;
    for (int k = 1; k <= N; ++k) {
        const double sk = s0 + 2.0 * kPi * k / N;
        const auto fv = f(sk, P, T);
        if ((prevF < 0.0) != (fv.first < 0.0) && fv.second > 0.0) {
            double lo = prevS, hi = sk, flo = prevF;
            for (int it = 0; it < 60; ++it) {
                const double mid = 0.5 * (lo + hi);
                const double fm = f(mid, P, T).first;
                if ((fm < 0.0) == (flo < 0.0)) { lo = mid; flo = fm; } else hi = mid;
            }
            EdgeHit h;
            h.s = 0.5 * (lo + hi);
            f(h.s, h.P, h.tangent);
            return h;
        }
        prevS = sk;
        prevF = fv.first;
    }
    return EdgeHit{0.0, Vec2{}, Vec2{}};
}

// signed arc length of a profile between two curve parameters (the short way round)
double arcBetween(const Model& m, int pi, double s0, double s1) {
    double ds = s1 - s0;
    while (ds > kPi) ds -= 2.0 * kPi;
    while (ds < -kPi) ds += 2.0 * kPi;
    const ProfileCurve& cv = *m.profiles[(size_t)pi].curve;
    const int n = 2000;
    double sum = 0.0;
    for (int k = 0; k <= n; ++k) {
        Vec2 p, d1, d2;
        cv.eval(s0 + ds * k / n, p, d1, d2);
        const double w = (k == 0 || k == n) ? 1.0 : (k % 2 ? 4.0 : 2.0);
        sum += w * vlen(d1);
    }
    return sum * (ds / n) / 3.0;
}

void testNonCircularMesh() {
    std::printf("[mesh M6d]\n");

    // ---- the analytic polar form of the ellipse: exact on its own equation ---------
    for (int focus = 0; focus < 2; ++focus) {
        const ProfileCurve e = ProfileCurve::ellipse(0.05, 0.03, focus != 0);
        const double cx = focus ? 0.04 : 0.0;          // sqrt(0.05^2 - 0.03^2)
        double worst = 0.0;
        for (int k = 0; k < 720; ++k) {
            const double phi = 2.0 * kPi * k / 720.0 + 0.0031;
            double r, dr, d2r;
            e.polarEval(phi, r, dr, d2r);
            const double x = r * std::cos(phi) - cx, y = r * std::sin(phi);
            worst = std::max(worst, std::fabs(x * x / (0.05 * 0.05) + y * y / (0.03 * 0.03) - 1.0));
        }
        CHECK(worst < 1e-12);                          // the point (r cos, r sin) lies ON the ellipse
    }
    {
        // derivatives by central differences
        for (int focus = 0; focus < 2; ++focus) {
            const ProfileCurve e = ProfileCurve::ellipse(0.05, 0.03, focus != 0);
            double w1 = 0.0, w2 = 0.0;
            for (int k = 0; k < 360; ++k) {
                const double phi = 2.0 * kPi * k / 360.0 + 0.0071, h = 1e-5;
                double r, dr, d2r, rp, drp, x, rm, drm;
                e.polarEval(phi, r, dr, d2r);
                e.polarEval(phi + h, rp, drp, x);
                e.polarEval(phi - h, rm, drm, x);
                w1 = std::max(w1, std::fabs((rp - rm) / (2 * h) - dr));
                w2 = std::max(w2, std::fabs((drp - drm) / (2 * h) - d2r));
            }
            CHECK(w1 < 1e-8 && w2 < 1e-7);
        }
    }

    // ---- grammar and diagnostics -----------------------------------------------------
    {
        const Program p = parseText("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
                                    "profile p on s { ellipse: 0.05 0.03 focus } "
                                    "profile q on s { ellipse: 0.05 0.03 } }");
        CHECK(p.mechanisms[0].profiles[0].kind == ProfileDecl::Kind::Ellipse);
        CHECK(p.mechanisms[0].profiles[0].ellipseFocus && !p.mechanisms[0].profiles[1].ellipseFocus);
        CHECK_APPROX(p.mechanisms[0].profiles[0].ellipseA, 0.05, 1e-15);
        CHECK_APPROX(p.mechanisms[0].profiles[0].ellipseB, 0.03, 1e-15);
    }
    auto expectModelError = [&](const std::string& src, const char* needle) {
        bool threw = false;
        try {
            buildFromText(src.c_str());
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
            CHECK(e.line >= 1);
        }
        CHECK(threw);
    };
    const std::string two =
        "mechanism m { ground g { anchor pa at (0,0) anchor pb at (0.1,0) } "
        "shaft a at g.pa shaft b at g.pb ";
    expectModelError(two + "profile p on a { ellipse: 0 0.03 } }", "two positive semi-axes");
    expectModelError(two + "profile p on a { ellipse: 0.03 0.05 focus } }", "major semi-axis first");
    {
        // a centered ellipse may have either axis longer
        buildFromText((two + "profile p on a { ellipse: 0.03 0.05 } }").c_str());
    }

    const std::string pair = std::string("mechanism m { ground g { anchor pa at (0,0) anchor pb at (0.1,0) } "
        "shaft a at g.pa shaft b at g.pb ")
        + "profile ga on a.A { ellipse: 0.05 " + kEllipB + " focus } ";
    expectModelError(pair + "profile gb on b.A { ellipse: 0.05 " + kEllipB + " focus } "
                     "mesh ga gb tangent }", "applies to gear meshes");
    expectModelError(pair + "gear x { radius: 0.05 } on b mesh ga x }", "not a profile");
    expectModelError(pair + "profile gb on b.A { points: (0.05,0) (0,0.05) (-0.05,0) (0,-0.05) } "
                     "mesh ga gb }", "point list");
    expectModelError(pair + "mesh ga ga }", "cannot mesh with itself");
    expectModelError(std::string("mechanism m { ground g { anchor pa at (0,0) anchor pb at (0.1,0) } "
                     "shaft a at g.pa shaft b at g.pb "
                     "profile ga on a { circle: 0.05 } profile gb on a { circle: 0.05 } mesh ga gb }"),
                     "same body");
    expectModelError(std::string("mechanism m { ground g { anchor pa at (0,0) anchor pb at (0.1,0) } "
                     "shaft a at g.pa  body b { anchor c at (0,0) } "
                     "profile ga on a { circle: 0.05 } profile gb on b.c { circle: 0.05 } mesh ga gb }"),
                     "not pinned to ground");
    expectModelError(std::string("mechanism m { ground g { anchor pa at (0,0) anchor pb at (0.1,0) } "
                     "shaft a at g.pa profile ga on a { circle: 0.05 } "
                     "profile gb on g.pb { circle: 0.05 } mesh ga gb }"), "is on ground");
    // not a conjugate pair: the right centre distance for the wrong curves
    expectModelError(pair + "profile gb on b.A { ellipse: 0.05 0.04 focus } mesh ga gb }",
                     "not a conjugate pair");
    // the right curves at the wrong distance (ellipses need 2a): the profiles cannot reach each other
    expectModelError(std::string("mechanism m { ground g { anchor pa at (0,0) anchor pb at (0.2,0) } "
                     "shaft a at g.pa shaft b at g.pb profile ga on a.A { ellipse: 0.05 ") + kEllipB +
                     " focus } profile gb on b.A { ellipse: 0.05 " + kEllipB + " focus } mesh ga gb }",
                     "cannot roll on each other");
    // an ellipse against a circle is not a pair either
    expectModelError(pair + "profile gb on b.A { circle: 0.05 } mesh ga gb }", "cannot roll on each other");

    // ---- circles reproduce `mesh` exactly -------------------------------------------------
    {
        const char* gearsSrc =
            "mechanism g { ground frame { anchor pa at (0,0) anchor pb at (0.1,0) }\n"
            "  shaft a at frame.pa  shaft b at frame.pb\n"
            "  gear ga { radius: 0.04 } on a  gear gb { radius: 0.06 } on b\n"
            "  mesh ga gb  drive a at 36 deg/s }";
        const char* circSrc =
            "mechanism g { ground frame { anchor pa at (0,0) anchor pb at (0.1,0) }\n"
            "  shaft a at frame.pa  shaft b at frame.pb\n"
            "  profile ga on a { circle: 0.04 } profile gb on b { circle: 0.06 }\n"
            "  mesh ga gb  drive a at 36 deg/s }";
        Model mg = buildFromText(gearsSrc), mc = buildFromText(circSrc);
        CHECK(mc.dofReport().freeDof == mg.dofReport().freeDof);
        CHECK_APPROX(mc.polarMeshes[0].minRatio, 0.04 / 0.06, 1e-12);       // |wB/wA| = rA/rB, constant
        CHECK_APPROX(mc.polarMeshes[0].maxRatio, 0.04 / 0.06, 1e-12);
        ConstraintSystem sg(mg), sc(mc);
        std::vector<double> qg, qc;
        mg.assemble(qg);
        mc.assemble(qc);
        int iters = 0;
        double res = 0.0;
        sg.solve(qg, 0.0, iters, &res);
        sc.solve(qc, 0.0, iters, &res);
        CHECK(mc.validateAssembly(qc).empty());
        const int bg = mg.bodyIndex("b"), bc = mc.bodyIndex("b");
        const double b0g = qg[(size_t)mg.bodies[bg].qOffset], b0c = qc[(size_t)mc.bodies[bc].qOffset];
        double worst = 0.0, worstExact = 0.0;
        for (int k = 1; k <= 144; ++k) {                 // two turns of A
            const double t = k / 6.0;
            sg.solve(qg, t, iters, &res);
            sc.solve(qc, t, iters, &res);
            const double dg = qg[(size_t)mg.bodies[bg].qOffset] - b0g;
            const double dc = qc[(size_t)mc.bodies[bc].qOffset] - b0c;
            worst = std::max(worst, std::fabs(dg - dc));
            worstExact = std::max(worstExact, std::fabs(dc - (-(0.04 / 0.06) * deg2rad(36.0) * t)));
        }
        CHECK(worst < 1e-9);          // same motion as the gear mesh
        CHECK(worstExact < 1e-9);     // and it is the exact ratio -rA/rB
    }

    // ---- an elliptical pair ------------------------------------------------------------------
    {
        Model m = buildFromText(ellipSrc("drive a at 30 deg/s").c_str());
        const PolarMesh& pm = m.polarMeshes[0];
        CHECK_APPROX(pm.minRatio, 1.0 / 3.0, 1e-9);       // (1 - e)/(1 + e)
        CHECK_APPROX(pm.maxRatio, 3.0, 1e-9);             // (1 + e)/(1 - e)
        CHECK(pm.conjDefect < 1e-9);
        CHECK(m.dofReport().freeDof == 1 && m.dofReport().residualDof == 0);
        CHECK_APPROX(pm.Ftot, 2.0 * kPi, 1e-9);           // B turns once per turn of A

        ConstraintSystem sys(m);
        std::vector<double> q;
        CHECK(m.assemble(q).empty());
        int iters = 0;
        double res = 0.0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(m.validateAssembly(q).empty());
        const int ia = m.bodyIndex("a"), ib = m.bodyIndex("b");
        const int pa = m.profileIndex("ga"), pb = m.profileIndex("gb");
        const Vec2 cA = pm.cA, cB = pm.cB;
        const double a0 = q[(size_t)m.bodies[ia].qOffset], b0 = q[(size_t)m.bodies[ib].qOffset];
        double worstRes = 0.0, worstTouch = 0.0, worstTangent = 0.0, worstArc = 0.0;
        EdgeHit prevA{}, prevB{};
        double bAtTurn = 0.0;
        for (int k = 0; k <= 144; ++k) {                  // two turns, 5 deg of A per step
            const double t = k / 6.0;
            sys.solve(q, t, iters, &res);
            worstRes = std::max(worstRes, res);
            if (k == 72) bAtTurn = q[(size_t)m.bodies[ib].qOffset];
            if (k % 3 != 0) continue;
            // rolling, judged on the curves alone: the contact point on A and on B
            // coincide, the tangents there are parallel, and the arcs rolled agree
            const double thA = q[(size_t)m.bodies[ia].qOffset], thB = q[(size_t)m.bodies[ib].qOffset];
            const EdgeHit ha = hitOnLine(m, pa, thA, cA, cB), hb = hitOnLine(m, pb, thB, cB, cA);
            worstTouch = std::max(worstTouch, vlen(ha.P - hb.P));
            worstTangent = std::max(worstTangent, std::fabs(crossz(normalize(ha.tangent), normalize(hb.tangent))));
            if (k > 0) {
                const double dLA = arcBetween(m, pa, prevA.s, ha.s);
                const double dLB = arcBetween(m, pb, prevB.s, hb.s);
                worstArc = std::max(worstArc, std::fabs(dLA + dLB));    // external: opposite senses
                CHECK(std::fabs(dLA) > 1e-3);                            // a real distance was rolled
            }
            prevA = ha;
            prevB = hb;
        }
        CHECK(worstRes < 1e-9);
        CHECK_APPROX(worstTouch, 0.0, 1e-8);       // the two curves meet at the contact point
        CHECK_APPROX(worstTangent, 0.0, 1e-6);     // ... tangentially
        CHECK_APPROX(worstArc, 0.0, 1e-6);         // ... and have rolled equal lengths: no slip
        CHECK_APPROX(bAtTurn - b0, -2.0 * kPi, 1e-9);     // one turn of A is exactly one (reversed) turn of B

        // the speed surges between 3x and 1/3x: read off the solved motion
        auto ratioAt = [&](double t) {
            std::vector<double> qa = q, qb = q;
            const double e = 1e-4;
            sys.solve(qa, t + e, iters, &res);
            sys.solve(qb, t - e, iters, &res);
            return (qa[(size_t)m.bodies[ib].qOffset] - qb[(size_t)m.bodies[ib].qOffset]) /
                   (qa[(size_t)m.bodies[ia].qOffset] - qb[(size_t)m.bodies[ia].qOffset]);
        };
        std::vector<double> q0;
        m.assemble(q0);
        sys.solve(q0, 0.0, iters, &res);
        q = q0;
        CHECK_APPROX(ratioAt(0.0), -3.0, 1e-5);             // far vertex of A meets near vertex of B
        q = q0;
        sys.solve(q, 6.0, iters, &res);                     // half a turn later
        CHECK_APPROX(ratioAt(6.0), -1.0 / 3.0, 1e-5);
        (void)a0;
    }

    // ---- non-causal: drive the OTHER shaft, and the pair still rolls ---------------------------------
    {
        Model m = buildFromText(ellipSrc("drive b at -30 deg/s").c_str());
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        CHECK(sys.solve(q, 0.0, iters, &res));
        CHECK(m.validateAssembly(q).empty());
        const int ia = m.bodyIndex("a");
        const double a0 = q[(size_t)m.bodies[ia].qOffset];
        double worstRes = 0.0, worstTouch = 0.0, aAtTurn = 0.0;
        for (int k = 1; k <= 72; ++k) {                   // one turn of B
            sys.solve(q, k / 6.0, iters, &res);
            worstRes = std::max(worstRes, res);
            if (k == 72) aAtTurn = q[(size_t)m.bodies[ia].qOffset];
            if (k % 6 == 0) {
                const PolarMesh& pm = m.polarMeshes[0];
                const double thA = q[(size_t)m.bodies[ia].qOffset];
                const double thB = q[(size_t)m.bodies[m.bodyIndex("b")].qOffset];
                double rA, rAp, rA2, rB, rBp, rB2;
                m.profiles[0].curve->polarEval(pm.psi - thA, rA, rAp, rA2);
                m.profiles[1].curve->polarEval(pm.psi + kPi - thB, rB, rBp, rB2);
                worstTouch = std::max(worstTouch, std::fabs(rA + rB - pm.d));   // contact on the line of centers
            }
        }
        CHECK(worstRes < 1e-9);
        CHECK(worstTouch < 1e-8);
        CHECK_APPROX(aAtTurn - a0, 2.0 * kPi, 1e-9);
    }

    // ---- assembly picks the tangent branch whatever the starting angle ------------------------------------
    {
        for (const char* from : {"from 70 deg", "from 130 deg", "from -40 deg"}) {
            const std::string line = std::string("drive a at 30 deg/s ") + from;
            Model m = buildFromText(ellipSrc(line.c_str()).c_str());
            ConstraintSystem sys(m);
            std::vector<double> q;
            m.assemble(q);
            int iters = 0;
            double res = 0.0;
            CHECK(sys.solve(q, 0.0, iters, &res));
            CHECK(m.validateAssembly(q).empty());
        }
        // two drives that disagree about the pose cannot be assembled into rolling contact
        Model bad = buildFromText(ellipSrc("drive a at 30 deg/s", "drive b at -30 deg/s from 20 deg").c_str());
        std::vector<double> q;
        bad.assemble(q);
        const std::vector<std::string> errs = bad.validateAssembly(q);
        CHECK(!errs.empty());
        CHECK(!errs.empty() && errs[0].find("not in rolling contact") != std::string::npos);
    }

    // ---- dynamics -----------------------------------------------------------------------------------------
    {
        // J qdd + bias = 0 along the true motion: a rate-driven pair, both shafts moving
        const std::string dyn = ellipSrc("drive a at 400 deg/s", "world { gravity: 9.81 }");
        int n = 0;
        const double ratio = biasScalingRatio(dyn.c_str(), &n);
        CHECK(n >= 8);
        CHECK(ratio > 6.0);        // ~8: only the h^3 term remains; a wrong bias leaves h^2 (~4)
    }
    {
        // work-energy: a constant torque on A from rest delivers tau * dtheta_A, all of it as
        // kinetic energy of the two shafts, however the ratio surges
        Model m = buildFromText(ellipSrc("drive a torque 0.002", "world { gravity: 0 }").c_str());
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const int ia = m.bodyIndex("a"), ib = m.bodyIndex("b");
        const double thA0 = q[(size_t)m.bodies[ia].qOffset];
        double worstWork = 0.0, worstRatio = 0.0, maxRatio = 0.0, minRatio = 1e9;
        const PolarMesh& pm = m.polarMeshes[0];
        for (long k = 0; k < 600; ++k) {                   // 1.2 s
            dyn.step(q, qd, k * m.world.step);
            const double thA = q[(size_t)m.bodies[ia].qOffset];
            const double wA = qd[(size_t)m.bodies[ia].qOffset], wB = qd[(size_t)m.bodies[ib].qOffset];
            const double ke = dyn.kinetic(q, qd);
            worstWork = std::max(worstWork, std::fabs(ke - 0.002 * (thA - thA0)));
            double rho, drho;
            pm.phase(*m.profiles[0].curve, pm.psi - thA, rho, drho);
            if (std::fabs(wA) > 1e-3) {
                worstRatio = std::max(worstRatio, std::fabs(wB / wA + rho));
                maxRatio = std::max(maxRatio, std::fabs(wB / wA));
                minRatio = std::min(minRatio, std::fabs(wB / wA));
            }
        }
        CHECK(worstWork < 1e-6);                           // KE = tau dtheta to a micro-joule (of ~0.03 J)
        CHECK_APPROX(dyn.hybridStats().driveWork, 0.002 * (q[(size_t)m.bodies[ia].qOffset] - thA0), 1e-6);  // the ledger counts it
        CHECK(worstRatio < 1e-6);                          // wB / wA = -rho(phi_A) all along
        CHECK(maxRatio > 2.5 && minRatio < 0.4);           // and it did surge, through most of the range
        CHECK(q[(size_t)m.bodies[ia].qOffset] - thA0 > 1.0);      // it really spun up
        CHECK(sys.maxResidual(q, 1.2) < 1e-8);
    }

    // ---- the shipped example --------------------------------------------------------------------------------
    {
        std::ifstream in("examples/ellipgears.mech");
        if (!in) {
            std::fprintf(stderr, "note: examples/ellipgears.mech not found (cwd?)\n");
        } else {
            std::stringstream ss;
            ss << in.rdbuf();
            Model m = buildModel(parse(lex(ss.str())).mechanisms[0]);
            CHECK(m.polarMeshes.size() == 1);
            CHECK(m.dofReport().residualDof == 0);
            CHECK_APPROX(m.polarMeshes[0].maxRatio, 3.0, 1e-6);
            CHECK_APPROX(m.polarMeshes[0].minRatio, 1.0 / 3.0, 1e-6);
            ConstraintSystem sys(m);
            std::vector<double> q;
            CHECK(m.assemble(q).empty());
            int iters = 0;
            double res = 0.0;
            CHECK(sys.solve(q, 0.0, iters, &res));
            CHECK(m.validateAssembly(q).empty());
            double worst = 0.0;
            for (int k = 1; k <= 240; ++k) {               // 4 s = two turns at 30 rpm
                sys.solve(q, k / 60.0, iters, &res);
                worst = std::max(worst, res);
            }
            CHECK(worst < 1e-9);
        }
    }
}


// ---- M6e: polygon profiles and the ratchet -------------------------------------------

void testPolygonRatchet() {
    std::printf("[polygon M6e]\n");

    // ---- exact geometry on a rounded square (given clockwise, to prove it is normalized) ----
    {
        const std::vector<Vec2> sq = {{-0.02, 0.02}, {0.02, 0.02}, {0.02, -0.02}, {-0.02, -0.02}};
        const ProfileCurve c = ProfileCurve::polygon(sq, 0.005);
        CHECK(c.pieces() == 8);                                    // 4 edges + 4 fillets
        CHECK_APPROX(c.period(), 4.0 * (0.04 - 0.01) + 2.0 * kPi * 0.005, 1e-12);
        CHECK_APPROX(c.minFillet(), 0.005, 1e-15);
        CHECK(!c.starShaped());
        CHECK_APPROX(c.nearest({0.0, 0.0}).signedDist, -0.02, 1e-12);          // inside: negative
        CHECK_APPROX(c.nearest({0.05, 0.0}).signedDist, 0.03, 1e-12);          // outside: positive
        // off a corner: the distance to the fillet circle's center (0.015, 0.015), less its radius
        CHECK_APPROX(c.nearest({0.05, 0.05}).signedDist, std::hypot(0.035, 0.035) - 0.005, 1e-12);
        const ProfileCurve::Foot top = c.nearest({0.0, 0.03});
        CHECK_APPROX(top.point.y, 0.02, 1e-12);
        CHECK_APPROX(top.normal.y, 1.0, 1e-12);                                // outward, even though the input was clockwise
        CHECK_APPROX(top.curvature, 0.0, 1e-15);                               // a straight edge
        CHECK_APPROX(c.nearest({0.05, 0.05}).curvature, 200.0, 1e-9);          // 1 / fillet radius

        // against brute force over the curve itself; signs against an independent inside test
        unsigned seed = 3;
        auto rnd = [&]() {
            seed = seed * 1664525u + 1013904223u;
            return ((seed >> 8) & 0xFFFF) / 65535.0 * 0.1 - 0.05;
        };
        double worst = 0.0;
        int signBad = 0;
        for (int t = 0; t < 400; ++t) {
            const Vec2 u{rnd(), rnd()};
            const ProfileCurve::Foot f = c.nearest(u);
            double best = 1e9;
            for (int k = 0; k < 30000; ++k) {
                Vec2 p, a, b;
                c.eval(c.period() * k / 30000.0, p, a, b);
                best = std::min(best, vlen(u - p));
            }
            worst = std::max(worst, std::fabs(std::fabs(f.signedDist) - best));
            const double ax = std::fabs(u.x), ay = std::fabs(u.y);
            const bool inside = (ax <= 0.015 || ay <= 0.015) ? (ax < 0.02 && ay < 0.02)
                                                              : std::hypot(ax - 0.015, ay - 0.015) < 0.005;
            if ((f.signedDist < 0.0) != inside) ++signBad;
        }
        CHECK(worst < 2e-7);       // brute-force grid resolution is ~5e-6 m
        CHECK(signBad == 0);
        // the curve is C1: the unit tangent is continuous across every junction
        double jump = 0.0;
        for (int k = 0; k < 400; ++k) {
            Vec2 p, a, b, p2, a2, b2;
            const double sk = c.period() * k / 400.0;
            c.eval(sk - 1e-9, p, a, b);
            c.eval(sk + 1e-9, p2, a2, b2);
            jump = std::max(jump, vlen(a - a2));
        }
        CHECK(jump < 1e-6);
    }
    {   // a reflex corner: the L-shape's fillet curves the other way (curvature -1/r), and a
        // point in the notch is OUTSIDE (positive) at distance r - |p - center| from the arc
        const std::vector<Vec2> L = {{0, 0}, {0.04, 0}, {0.04, 0.01}, {0.01, 0.01}, {0.01, 0.04}, {0, 0.04}};
        const ProfileCurve c = ProfileCurve::polygon(L, 0.002);
        // (0.0115, 0.0115) is down-left of the fillet circle's center (0.012, 0.012), so the
        // nearest point of that circle lies on the arc (the arc is its lower-left quarter)
        const ProfileCurve::Foot f = c.nearest({0.0115, 0.0115});
        CHECK_APPROX(f.signedDist, 0.002 - std::hypot(0.0005, 0.0005), 1e-12);
        CHECK_APPROX(f.curvature, -500.0, 1e-9);
    }
    {   // a fillet that does not fit is clamped to half the shorter edge
        const std::vector<Vec2> rect = {{0, 0}, {0.04, 0}, {0.04, 0.01}, {0, 0.01}};
        CHECK_APPROX(ProfileCurve::polygon(rect, 0.02).minFillet(), 0.005, 1e-12);
        CHECK_APPROX(ProfileCurve::polygon(rect, 0.003).minFillet(), 0.003, 1e-15);
    }

    // ---- grammar and diagnostics ------------------------------------------------------------
    {
        const Program p = parseText("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
                                    "profile t on s { polygon: (0,0) (0.04,0) (0.04,0.01) (0,0.01) fillet 0.001 } }");
        const ProfileDecl& pd = p.mechanisms[0].profiles[0];
        CHECK(pd.kind == ProfileDecl::Kind::Polygon && pd.points.size() == 4);
        CHECK(pd.hasFillet && std::fabs(pd.fillet - 0.001) < 1e-15);
        // a free body may now carry a release angle, like a rod
        const Program q = parseText("mechanism m { ground g { anchor a at (0,0) } body w { anchor c at (0,0) mass: 2 from: 30 deg } }");
        CHECK_APPROX(q.mechanisms[0].bodies[1].props.at("from"), 30.0, 1e-12);
        CHECK_APPROX(q.mechanisms[0].bodies[1].props.at("mass"), 2.0, 1e-12);
    }
    auto expectModelError = [&](const std::string& body, const char* needle) {
        bool threw = false;
        try {
            buildFromText(("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a profile t on s { " +
                           body + " } }").c_str());
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };
    expectModelError("polygon: (0,0) (0.04,0) (0.04,0.01) (0,0.01)", "needs a positive 'fillet'");
    expectModelError("polygon: (0,0) (0.04,0) (0.04,0.01) (0,0.01) fillet 0", "needs a positive 'fillet'");
    expectModelError("polygon: (0,0) (0.04,0) fillet 0.001", "at least 3 vertices");
    expectModelError("polygon: (0,0) (0,0) (0.04,0.01) fillet 0.001", "coincident");
    expectModelError("polygon: (0,0) (0.02,0) (0.04,0) fillet 0.001", "no area");
    expectModelError("polygon: (0,0) (0.04,0) (0.04,0.01) (0,0.01) fillet 0.2", "does not fit");
    buildFromText("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
                  "profile t on s { polygon: (0,0) (0.04,0) (0.04,0.01) (0,0.01) fillet 0.001 } }");

    // ---- the ratchet ------------------------------------------------------------------------------
    std::string ratchetSrc;
    {
        std::ifstream in("examples/ratchet.mech");
        if (!in) {
            std::fprintf(stderr, "note: examples/ratchet.mech not found (cwd?)\n");
            return;
        }
        std::stringstream ss;
        ss << in.rdbuf();
        ratchetSrc = ss.str();
    }
    auto withTorque = [&](const char* torque) {
        std::string t = ratchetSrc;
        const size_t pos = t.find("drive wheel torque -0.004");
        t.replace(pos, std::string("drive wheel torque -0.004").size(), std::string("drive wheel torque ") + torque);
        return t;
    };
    // the shipped file uses friction 0.3; the frictionless numbers below pin it to 0
    auto variant = [&](const char* torque, const char* mu) {
        std::string t = withTorque(torque);
        const std::string key = "friction 0.3";
        t.replace(t.find(key), key.size(), std::string("friction ") + mu);
        return t;
    };
    {
        Model m = buildFromText(ratchetSrc.c_str());
        CHECK(m.profiles.size() == 1 && m.profiles[0].curve->pieces() == 48);   // 24 edges + 24 fillets
        CHECK_APPROX(m.touches[0].friction, 0.3, 1e-15);
        std::vector<double> q0;
        m.assemble(q0);
        CHECK(!m.touches[0].engaged);                                           // placed just above the root
    }

    // forward (frictionless): one click per tooth, and every joule accounted for
    {
        Model m = buildFromText(variant("-0.004", "0").c_str());
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        const int iw = m.bodyIndex("wheel");
        const double th0 = q[(size_t)m.bodies[iw].qOffset];
        const double h = m.world.step;
        double worstEngaged = 0.0, deepest = 0.0, worstForce = 1e9, worstRes = 0.0;
        for (long k = 0; k < 3000; ++k) {                         // 1.5 s
            dyn.step(q, qd, k * h);
            const TouchJoint& tj = m.touches[0];
            const double gap = m.touchState(tj, q).gap - tj.roller;
            if (tj.engaged) {
                worstEngaged = std::max(worstEngaged, std::fabs(gap));
                std::vector<double> qdd;
                dyn.accelerations(q, qd, qdd);
                worstForce = std::min(worstForce, dyn.contactForces()[0]);
            } else {
                deepest = std::min(deepest, gap);                 // flight never digs into a tooth
            }
            if (k % 50 == 0) worstRes = std::max(worstRes, sys.maxResidual(q, k * h));
        }
        const HybridStats& hs = dyn.hybridStats();
        const double turned = th0 - q[(size_t)m.bodies[iw].qOffset];   // clockwise = positive
        const double teeth = turned / deg2rad(30.0);
        CHECK(turned > deg2rad(240.0));                           // it really ratcheted around
        // one lift-off per tooth tip passed, each ending in an impact
        CHECK(std::fabs((double)hs.liftoffs - std::floor(teeth)) <= 1.0);
        CHECK(hs.impacts >= hs.liftoffs && hs.impacts <= hs.liftoffs + 2);
        CHECK(worstEngaged < 1e-8);
        CHECK(deepest > -2e-6);
        CHECK(worstForce > -1e-6);
        CHECK(worstRes < 1e-8);
        // the ledger: torque work in, impacts out, the change in energy in between
        const double dE = dyn.kinetic(q, qd) + dyn.potential(q) - E0;
        CHECK_APPROX(hs.driveWork, 0.004 * turned, 2e-5);         // the torque's work is tau x angle
        CHECK(std::fabs(dE - (hs.driveWork - hs.impactLoss)) < 2e-6);
        CHECK(hs.impactLoss > 0.005);
    }

    // reverse: the radial face meets the tip and the wheel locks. A frictionless pawl and
    // wheel are a conservative system, so the jammed pair keeps ringing (about a degree);
    // what must hold regardless is that the wheel is held, and that the contact force is
    // the force that balances the torque. That is the wheel's own equation of motion,
    //   I dw/dt = tau - N cross(footArm, n),
    // integrated over a window:  integral of N cross dt = tau T - I (w_end - w_start).
    {
        Model m = buildFromText(variant("0.004", "0").c_str());
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const int iw = m.bodyIndex("wheel");
        const double h = m.world.step;
        const double I = m.mass[(size_t)iw].inertia, tau = 0.004;
        double impulse = 0.0, w0 = 0.0, prev = 0.0, thMin = 1e9, thMax = -1e9;
        bool have = false;
        const long k0 = 800, k1 = 2800;                              // the window: 0.4 s .. 1.4 s
        for (long k = 0; k < k1; ++k) {
            if (k == k0) w0 = qd[(size_t)m.bodies[iw].qOffset];
            dyn.step(q, qd, k * h);
            if (k < k0) continue;
            const TouchJoint& tj = m.touches[0];
            double cur = 0.0;
            if (tj.engaged) {
                std::vector<double> qdd;
                dyn.accelerations(q, qd, qdd);
                const ContactState cs = m.touchState(tj, q);
                cur = dyn.contactForces()[0] * crossz(cs.footArm, cs.normal);
            }
            if (have) impulse += 0.5 * (prev + cur) * h;
            prev = cur;
            have = true;
            const double th = q[(size_t)m.bodies[iw].qOffset];
            thMin = std::min(thMin, th);
            thMax = std::max(thMax, th);
        }
        const double T = (double)(k1 - k0) * h;
        const double w1 = qd[(size_t)m.bodies[iw].qOffset];
        CHECK(thMax < deg2rad(5.0) && thMin > -deg2rad(1.0));      // held: it never gets past the tooth
        CHECK(thMax - thMin < deg2rad(2.0));                       // ringing, not creeping
        CHECK_APPROX(impulse, tau * T - I * (w1 - w0), 2e-6);      // the contact force balances the torque
        CHECK_APPROX(impulse / T, tau, 0.08 * tau);                // so on average it IS the torque
        CHECK(dyn.hybridStats().liftoffs <= 4);                    // and it is not bouncing
        CHECK(sys.maxResidual(q, k1 * h) < 1e-8);
    }

    // with friction the locked wheel comes to REST (it rang forever without), held by exactly
    // the normal force that balances the torque: tau = N cross(footArm, n)
    {
        Model m = buildFromText(variant("0.004", "0.3").c_str());
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const int iw = m.bodyIndex("wheel");
        const double h = m.world.step;
        for (long k = 0; k < 3000; ++k) dyn.step(q, qd, k * h);          // 1.5 s
        const double th1 = q[(size_t)m.bodies[iw].qOffset];
        for (long k = 3000; k < 4000; ++k) dyn.step(q, qd, k * h);       // 0.5 s more
        CHECK(m.touches[0].engaged);
        CHECK(std::fabs(qd[(size_t)m.bodies[iw].qOffset]) < 1e-6);       // at rest
        CHECK(std::fabs(q[(size_t)m.bodies[iw].qOffset] - th1) < deg2rad(1e-3));   // and staying there
        CHECK(std::fabs(th1) < deg2rad(5.0));                            // held within a few degrees
        const ContactState cs = m.touchState(m.touches[0], q);
        std::vector<double> qdd;
        dyn.accelerations(q, qd, qdd);
        const double N = dyn.contactForces()[0];
        CHECK_APPROX(N * crossz(cs.footArm, cs.normal), 0.004, 1e-5);   // the static balance
        CHECK(dyn.hybridStats().frictionLoss > 1e-6);                    // and friction did the damping
    }
    // forward with friction: still ratchets (slower: the ramps resist), and the ledger,
    // now with a friction term, still closes
    {
        auto turned = [&](const std::string& src, HybridStats* hsOut, double* ledgerOut) {
            Model m = buildFromText(src.c_str());
            ConstraintSystem sys(m);
            DynamicsSystem dyn(m, sys);
            std::vector<double> q, qd((size_t)m.nq(), 0.0);
            m.assemble(q);
            int iters = 0;
            double res = 0.0;
            sys.solve(q, 0.0, iters, &res);
            dyn.projectVelocities(q, qd);
            const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
            const int iw = m.bodyIndex("wheel");
            const double th0 = q[(size_t)m.bodies[iw].qOffset];
            for (long k = 0; k < 3000; ++k) dyn.step(q, qd, k * m.world.step);
            const HybridStats& hs = dyn.hybridStats();
            const double dE = dyn.kinetic(q, qd) + dyn.potential(q) - E0;
            *ledgerOut = dE - (hs.driveWork - hs.impactLoss - hs.frictionLoss);
            *hsOut = hs;
            return th0 - q[(size_t)m.bodies[iw].qOffset];
        };
        HybridStats h0, h3;
        double l0, l3;
        const double free = turned(variant("-0.004", "0"), &h0, &l0);
        const double rough = turned(variant("-0.004", "0.3"), &h3, &l3);
        CHECK(h0.frictionLoss == 0.0);
        CHECK(h3.frictionLoss > 1e-3);
        CHECK(rough > deg2rad(120.0) && rough < free);                   // slower, but it turns
        CHECK(std::fabs((double)h3.liftoffs - std::floor(rough / deg2rad(30.0))) <= 1.0);   // still one click per tooth
        CHECK(std::fabs(l0) < 2e-6);
        CHECK(std::fabs(l3) < 2e-6);                                     // the ledger closes with friction
    }

    // determinism
    {
        auto run = [&](std::vector<double>& qOut, HybridStats& stOut) {
            Model m = buildFromText(ratchetSrc.c_str());
            ConstraintSystem sys(m);
            DynamicsSystem dyn(m, sys);
            std::vector<double> q, qd((size_t)m.nq(), 0.0);
            m.assemble(q);
            int iters = 0;
            double res = 0.0;
            sys.solve(q, 0.0, iters, &res);
            dyn.projectVelocities(q, qd);
            for (long k = 0; k < 1000; ++k) dyn.step(q, qd, k * m.world.step);
            qOut = q;
            stOut = dyn.hybridStats();
        };
        std::vector<double> qa, qb;
        HybridStats sa, sb;
        run(qa, sa);
        run(qb, sb);
        CHECK(qa == qb);
        CHECK(sa.liftoffs == sb.liftoffs && sa.impacts == sb.impacts && sa.impactLoss == sb.impactLoss);
    }
}


// ---- M6f: friction ---------------------------------------------------------------------

// A brake: a wheel (inertia `I`) spins against a shoe on a vertical rail through the
// top of its rim; a spring sets the normal force N = k (R - rest) = 200 x 0.02 = 4 N,
// and the rail absorbs the tangential load. Friction torque = mu N R, exactly.
std::string brakeSrc(double mu, double inertia, double step) {
    char b[1000];
    std::snprintf(b, sizeof b,
        "mechanism brake {\n"
        "  world { gravity: 0  step: %g }\n"
        "  ground frame { anchor axle at (0, 0)  anchor rail at (0, 0.05) dir (0, 1) }\n"
        "  body wheel { anchor axle at (0, 0)  mass: %g  from: 0 deg }\n"
        "  pin wheel.axle -> frame.axle\n"
        "  profile rim on wheel.axle { circle: 0.05 }\n"
        "  body shoe { anchor tip at (0, 0) dir (1, 0)  mass: 0.1 }\n"
        "  slide shoe.tip -> frame.rail\n"
        "  spring press shoe.tip -> frame.axle { k: 200  rest: 0.03 }\n"
        "  touch shoe.tip on rim friction %g\n}\n", step, inertia / 0.01, mu);
    return std::string(b);
}

struct BrakeRun {
    std::vector<double> w;        // wheel speed sampled at t = 0.2, 0.4, ...
    double N = 0.0, loss = 0.0, ledger = 0.0, ke0 = 0.0, wEnd = 0.0, shoeY = 0.0;
    double stopT = -1.0;          // first time |w| < 1e-3
    bool signFlipped = false;
};
BrakeRun runBrake(double mu, double w0, double inertia, double step, double T) {
    BrakeRun r;
    Model m = buildFromText(brakeSrc(mu, inertia, step).c_str());
    ConstraintSystem sys(m);
    DynamicsSystem dyn(m, sys);
    std::vector<double> q, qd((size_t)m.nq(), 0.0);
    m.assemble(q);
    int iters = 0;
    double res = 0.0;
    sys.solve(q, 0.0, iters, &res);
    dyn.projectVelocities(q, qd);
    const int iw = m.bodyIndex("wheel"), is = m.bodyIndex("shoe");
    qd[(size_t)m.bodies[iw].qOffset] = w0;
    const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
    r.ke0 = 0.5 * m.mass[(size_t)iw].inertia * w0 * w0;
    long nextSample = (long)std::llround(0.2 / step);
    bool stopped = false;
    for (long k = 0; k * step < T; ++k) {
        dyn.step(q, qd, k * step);
        const double w = qd[(size_t)m.bodies[iw].qOffset];
        const double t = (k + 1) * step;
        if (k + 1 == nextSample) { r.w.push_back(w); nextSample += (long)std::llround(0.2 / step); }
        if (!stopped && std::fabs(w) < 1e-3) { stopped = true; r.stopT = t; }
        if (stopped && w * w0 < -1e-3 * std::fabs(w0)) r.signFlipped = true;   // overshoot through zero
    }
    const HybridStats& hs = dyn.hybridStats();
    const double dE = dyn.kinetic(q, qd) + dyn.potential(q) - E0;
    r.loss = hs.frictionLoss;
    r.ledger = dE - (hs.driveWork - hs.impactLoss - hs.frictionLoss);
    r.wEnd = qd[(size_t)m.bodies[iw].qOffset];
    r.shoeY = q[(size_t)m.bodies[is].qOffset + 2];
    std::vector<double> qdd;
    dyn.accelerations(q, qd, qdd);
    r.N = dyn.contactForces()[0];
    return r;
}

void testFriction() {
    std::printf("[friction M6f]\n");

    // ---- grammar and diagnostics --------------------------------------------------------------
    {
        const Program p = parseText("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
                                    "profile p on s { circle: 0.03 } "
                                    "touch g.a on p friction 0.4 restitution 0.2 roller 0.001 world { gravity: 1 } }");
        const TouchDecl& t = p.mechanisms[0].touches[0];
        CHECK(t.hasFriction && std::fabs(t.friction - 0.4) < 1e-15);
        CHECK(t.hasRestitution && t.hasRoller);                 // any order
    }
    auto expectModelError = [&](const std::string& tail, const char* needle) {
        bool threw = false;
        try {
            buildFromText(("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
                           "profile p on s { circle: 0.03 } touch g.a on p " + tail + " }").c_str());
        } catch (const SemanticError& e) {
            threw = true;
            CHECK(std::string(e.msg).find(needle) != std::string::npos);
        }
        CHECK(threw);
    };
    expectModelError("friction 0.3", "no effect in a kinematic");            // nothing slides under a force
    expectModelError("friction -0.1 world { gravity: 9.81 }", "cannot be negative");
    {
        Model m = buildFromText("mechanism m { ground g { anchor a at (0,0) } shaft s at g.a "
                                "profile p on s { circle: 0.03 } touch g.a on p friction 0 "
                                "world { gravity: 9.81 } }");
        CHECK(m.touches[0].friction == 0.0);                    // zero is legal: frictionless
    }

    // ---- the brake, in closed form ------------------------------------------------------------------
    // friction torque mu N R = 0.25 x 4 x 0.05 = 0.05 N m on I = 0.01: a constant 5 rad/s^2
    {
        const BrakeRun r = runBrake(0.25, 5.0, 0.01, 0.0005, 1.4);
        CHECK(r.w.size() >= 6);
        for (int i = 0; i < 4; ++i)                            // t = 0.2 .. 0.8, still sliding
            CHECK_APPROX(r.w[(size_t)i], 5.0 - 5.0 * 0.2 * (i + 1), 1e-4);
        CHECK(r.stopT > 0.99 && r.stopT < 1.03);              // 1 s, plus the creep tail
        CHECK(std::fabs(r.wEnd) < 1e-4);                       // and it stays stopped
        CHECK(!r.signFlipped);                                 // no overshoot through zero
        CHECK_APPROX(r.N, 4.0, 1e-5);                          // the spring's force, undisturbed
        CHECK_APPROX(r.shoeY, 0.05, 1e-9);                     // the shoe never moved
        CHECK_APPROX(r.loss, r.ke0, 1e-4);                     // all of (1/2) I w^2 went to friction: 0.125 J
        CHECK(std::fabs(r.ledger) < 1e-6);                     // and the ledger closes

        // doubling mu doubles the deceleration; zero friction does not decelerate at all
        const BrakeRun r2 = runBrake(0.5, 5.0, 0.01, 0.0005, 0.7);
        CHECK_APPROX(r2.w[0], 5.0 - 10.0 * 0.2, 1e-4);
        CHECK(r2.stopT > 0.49 && r2.stopT < 0.53);
        const BrakeRun r0 = runBrake(0.0, 5.0, 0.01, 0.0005, 0.6);
        CHECK_APPROX(r0.w[2], 5.0, 1e-9);
        CHECK(r0.loss == 0.0);
        // spinning the other way, friction opposes it just the same
        const BrakeRun rn = runBrake(0.25, -5.0, 0.01, 0.0005, 1.2);
        CHECK_APPROX(rn.w[1], -5.0 + 5.0 * 0.4, 1e-4);
        CHECK_APPROX(rn.loss, rn.ke0, 1e-4);
    }

    // ---- stiffness: a light wheel, strong friction, a coarse step ---------------------------------------
    // mu N / eps over the effective inertia would be ~1e5 /s: an explicit integrator with a
    // fixed friction width would blow up. The width adapts, so it must not.
    {
        const BrakeRun r = runBrake(0.5, 40.0, 0.0001, 0.002, 1.0);
        CHECK(std::isfinite(r.wEnd) && std::isfinite(r.loss));
        CHECK(r.stopT > 0.035 && r.stopT < 0.06);              // mu N R / I = 1000 rad/s^2: 0.04 s
        CHECK(std::fabs(r.wEnd) < 1e-3);
        CHECK(!r.signFlipped);
        CHECK_APPROX(r.loss, r.ke0, 2e-4);                     // 0.08 J
    }

    // ---- determinism ----------------------------------------------------------------------------------------
    {
        const BrakeRun a = runBrake(0.25, 5.0, 0.01, 0.0005, 0.6);
        const BrakeRun b = runBrake(0.25, 5.0, 0.01, 0.0005, 0.6);
        CHECK(a.w == b.w && a.loss == b.loss && a.wEnd == b.wEnd);
    }
}


// ---- M7: time-series export ---------------------------------------------------------------

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',') { out.push_back(cur); cur.clear(); } else cur += c;
    }
    out.push_back(cur);
    return out;
}

int colIndex(const std::vector<std::string>& cols, const std::string& name) {
    for (size_t i = 0; i < cols.size(); ++i)
        if (cols[i] == name) return (int)i;
    return -1;
}

void testRecorder() {
    std::printf("[record M7]\n");

    // ---- kinematic: the eccentric disc through the Recorder ------------------------------
    {
        Model m = buildFromText(eccentricSrc("profile disc on cam.rim { circle: 0.03 }", "roller 0.006").c_str());
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        const int ifol = m.bodyIndex("follower");
        const int itip = m.anchorIndex(ifol, "tip");
        Recorder rec(m, nullptr, {Probe{ifol, itip}});
        const std::vector<std::string>& cols = rec.columns();
        CHECK(cols[0] == "t[s]");
        for (const char* n : {"cam.theta[rad]", "follower.y[m]", "follower.tip.x[m]", "follower.tip.y[m]",
                              "maxRes", "follower.tip@disc.gap[m]", "follower.tip@disc.foot.x[m]",
                              "follower.tip@disc.normal[rad]"})
            CHECK(colIndex(cols, n) >= 0);
        CHECK(colIndex(cols, "KE[J]") < 0);                       // no energies in a kinematic run
        CHECK(colIndex(cols, "follower.vx[m/s]") < 0);
        CHECK(rec.headerLine().find('\n') == std::string::npos);
        {   // names are unique and the header has exactly one cell per column
            CHECK(splitCsv(rec.headerLine()).size() == cols.size());
            for (size_t i = 0; i < cols.size(); ++i)
                for (size_t j = i + 1; j < cols.size(); ++j) CHECK(cols[i] != cols[j]);
        }
        const int iy = colIndex(cols, "follower.y[m]"), ipy = colIndex(cols, "follower.tip.y[m]");
        const int igap = colIndex(cols, "follower.tip@disc.gap[m]");
        const int ith = colIndex(cols, "cam.theta[rad]"), inorm = colIndex(cols, "follower.tip@disc.normal[rad]");
        const int ifx = colIndex(cols, "follower.tip@disc.foot.x[m]");
        double worstY = 0.0, worstProbe = 0.0, worstGap = 0.0, worstRound = 0.0;
        for (int k = 0; k <= 72; ++k) {                           // one turn, 5 degrees a step
            const double t = k / 6.0;
            sys.solve(q, t, iters, &res);
            const std::vector<double> row = rec.sample(t, q, {}, sys.maxResidual(q, t));
            CHECK(row.size() == cols.size());
            worstY = std::max(worstY, std::fabs(row[(size_t)iy] - eccentricY(row[(size_t)ith], 0.006)));
            worstProbe = std::max(worstProbe, std::fabs(row[(size_t)ipy] - row[(size_t)iy]));   // tip at the origin
            worstGap = std::max(worstGap, std::fabs(row[(size_t)igap]));                        // in contact throughout
            // the foot lies on the disc: |foot - rim centre| = R, the rim centre being e off the axle
            const double th = row[(size_t)ith];
            const Vec2 centre{0.010 * std::cos(th), 0.010 * std::sin(th)};
            const Vec2 foot{row[(size_t)ifx], row[(size_t)ifx + 1]};
            CHECK_APPROX(vlen(foot - centre), 0.030, 1e-9);
            // and the normal points from the disc centre through the foot
            const Vec2 n = normalize(foot - centre);
            CHECK_APPROX(std::atan2(n.y, n.x), row[(size_t)inorm], 1e-9);
            // the CSV text round-trips every value to 12 significant digits
            const std::vector<std::string> cells = splitCsv(Recorder::csvLine(row));
            CHECK(cells.size() == row.size());
            for (size_t i = 0; i < row.size(); ++i)
                worstRound = std::max(worstRound, std::fabs(std::strtod(cells[i].c_str(), nullptr) - row[i]) /
                                                      std::max(1e-300, std::max(1e-9, std::fabs(row[i]))));
        }
        CHECK(worstY < 1e-9);          // the exported follower height IS the closed form
        CHECK(worstProbe < 1e-12);
        CHECK(worstGap < 1e-9);
        CHECK(worstRound < 1e-10);
    }

    // ---- two independent routes to the same number ----------------------------------------------
    // The cam synthesizer's design pressure angle comes from a FORMULA (tan a = s'/r); the
    // Recorder's contact normal comes from the SOLVED geometry. They must agree.
    {
        Model m = buildFromText(plateCamSrc(kCamBody, "30").c_str());
        ConstraintSystem sys(m);
        std::vector<double> q;
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        Recorder rec(m, nullptr, {});
        const int inorm = colIndex(rec.columns(), "follower.tip@lobe.normal[rad]");
        const int ith = colIndex(rec.columns(), "cam.theta[rad]");
        CHECK(inorm >= 0 && ith >= 0);
        double maxAlpha = 0.0, atTheta = 0.0;
        for (int k = 0; k <= 360; ++k) {                          // one turn, 1 degree a step
            const double t = k / 30.0;
            sys.solve(q, t, iters, &res);
            const std::vector<double> row = rec.sample(t, q, {}, 0.0);
            const double alpha = std::fabs(row[(size_t)inorm] - kPi / 2.0);   // vertical rail
            if (alpha > maxAlpha) { maxAlpha = alpha; atTheta = std::fmod(row[(size_t)ith], 2.0 * kPi); }
        }
        CHECK_APPROX(rad2deg(maxAlpha), rad2deg(m.cams[0].maxPressure), 0.05);     // 20.59 deg, both ways
        // and at the same cam angle, to the 1 degree grid (46.7 designed, 46.6 measured)
        CHECK_APPROX(rad2deg(atTheta), rad2deg(m.cams[0].maxPressureAt), 1.0);
    }

    // ---- dynamic: the brake's energy, force and ledger as time series ----------------------------
    {
        Model m = buildFromText(brakeSrc(0.25, 0.01, 0.0005).c_str());
        ConstraintSystem sys(m);
        DynamicsSystem dyn(m, sys);
        std::vector<double> q, qd((size_t)m.nq(), 0.0);
        m.assemble(q);
        int iters = 0;
        double res = 0.0;
        sys.solve(q, 0.0, iters, &res);
        dyn.projectVelocities(q, qd);
        const int iw = m.bodyIndex("wheel");
        qd[(size_t)m.bodies[iw].qOffset] = 5.0;
        const double E0 = dyn.kinetic(q, qd) + dyn.potential(q);
        Recorder rec(m, &dyn, {});
        rec.setBaseline(E0);
        const std::vector<std::string>& cols = rec.columns();
        for (const char* n : {"wheel.omega[rad/s]", "shoe.tip@rim.N[N]", "shoe.tip@rim.engaged", "KE[J]", "PE[J]",
                              "E[J]", "ledger.friction[J]", "ledger.residual[J]"})
            CHECK(colIndex(cols, n) >= 0);
        CHECK(colIndex(cols, "drive.wheel[N*m]") < 0);            // no rate drive here
        const int iw_ = colIndex(cols, "wheel.omega[rad/s]"), iN = colIndex(cols, "shoe.tip@rim.N[N]");
        const int ie = colIndex(cols, "shoe.tip@rim.engaged"), iKE = colIndex(cols, "KE[J]");
        const int iPE = colIndex(cols, "PE[J]"), iE = colIndex(cols, "E[J]");
        const int iFr = colIndex(cols, "ledger.friction[J]"), iRes = colIndex(cols, "ledger.residual[J]");
        double worstRes = 0.0, worstN = 0.0, worstE = 0.0, worstDecel = 0.0, prevKE = 1e9;
        bool monotone = true, always = true;
        std::vector<double> last;
        for (long k = 0; k <= 1600; ++k) {                        // 0.8 s
            if (k > 0) dyn.step(q, qd, (k - 1) * m.world.step);
            if (k % 100 != 0) continue;
            const double t = k * m.world.step;
            const std::vector<double> row = rec.sample(t, q, qd, sys.maxResidual(q, t));
            CHECK(row.size() == cols.size());
            worstRes = std::max(worstRes, std::fabs(row[(size_t)iRes]));
            worstN = std::max(worstN, std::fabs(row[(size_t)iN] - 4.0));
            worstE = std::max(worstE, std::fabs(row[(size_t)iE] - (row[(size_t)iKE] + row[(size_t)iPE])));
            always = always && row[(size_t)ie] == 1.0;
            if (row[(size_t)iKE] > prevKE + 1e-12) monotone = false;
            prevKE = row[(size_t)iKE];
            if (t > 0.0) worstDecel = std::max(worstDecel, std::fabs(row[(size_t)iw_] - (5.0 - 5.0 * t)));
            last = row;
        }
        CHECK(worstRes < 1e-6);        // the ledger closes at every sample, not just at the end
        CHECK(worstN < 1e-5);          // the spring's 4 N, as a column
        CHECK(worstE < 1e-12);
        CHECK(worstDecel < 1e-3);      // the wheel decelerates at 5 rad/s^2 (and is still sliding at 0.8 s)
        CHECK(always && monotone);     // always on the curve; kinetic energy only ever falls
        CHECK_APPROX(last[(size_t)iFr], 0.5 * 0.01 * (25.0 - 1.0), 1e-3);   // friction so far: (1/2) I (w0^2 - w^2)
    }

    // ---- the command line, end to end -------------------------------------------------------------
    if (std::ifstream("./mech")) {
        auto run = [&](const std::string& args) {
            const int st = std::system(("./mech run " + args + " >/dev/null 2>&1").c_str());
            return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
        };
        char tmpl[] = "/tmp/mech_csv_XXXXXX";
        const int fd = mkstemp(tmpl);
        if (fd >= 0) close(fd);
        const std::string out = tmpl;

        CHECK(run("examples/eccentric.mech --headless 1 --csv " + out + " --every 6 --probe follower.tip") == 0);
        {
            std::ifstream in(out);
            std::string line;
            std::getline(in, line);
            const std::vector<std::string> head = splitCsv(line);
            CHECK(colIndex(head, "follower.tip.y[m]") >= 0);
            const int iy = colIndex(head, "follower.y[m]"), it = colIndex(head, "t[s]");
            int rows = 0;
            double lastT = 0.0, worst = 0.0;
            while (std::getline(in, line)) {
                const std::vector<std::string> cells = splitCsv(line);
                CHECK(cells.size() == head.size());
                const double th = deg2rad(360.0) * std::strtod(cells[(size_t)it].c_str(), nullptr);   // 60 rpm
                worst = std::max(worst, std::fabs(std::strtod(cells[(size_t)iy].c_str(), nullptr) - eccentricY(th, 0.006)));
                lastT = std::strtod(cells[(size_t)it].c_str(), nullptr);
                ++rows;
            }
            CHECK(rows == 11);                          // t = 0 plus every 6th of 60 frames
            CHECK_APPROX(lastT, 1.0, 1e-9);
            CHECK(worst < 1e-8);                        // the file holds the closed form
        }
        CHECK(run("examples/jumpcam.mech --headless 0.05 --csv " + out + " --every 10") == 0);
        {
            std::ifstream in(out);
            std::string line;
            std::getline(in, line);
            CHECK(colIndex(splitCsv(line), "ledger.residual[J]") >= 0);   // dynamic files carry the ledger
        }
        // misuse is refused with a nonzero status, not silently ignored
        CHECK(run("examples/eccentric.mech --headless 1 --probe nope.tip") != 0);
        CHECK(run("examples/eccentric.mech --headless 1 --probe frame.rail") != 0);     // ground never moves
        CHECK(run("examples/eccentric.mech --csv " + out) != 0);                         // needs --headless
        CHECK(run("examples/eccentric.mech --headless 1 --every 0") != 0);
        CHECK(run("examples/jumpcam.mech --headless 0.1 --dt 0.01") != 0);               // dynamic: world step rules
        CHECK(run("examples/eccentric.mech --headless 1 --dt 0.1 --csv " + out) == 0);   // kinematic: dt is fine
        unlink(tmpl);
    } else {
        std::fprintf(stderr, "note: ./mech not found (cwd?); skipping the command-line checks\n");
    }
}


// ---- M8: parameters, expressions and sweeps --------------------------------------------------

Program parseWith(const std::string& src, const ParamOverrides& ov = {}) {
    return parse(lex(src), ov);
}

// the value of `rod a { length: <expr> }` in a one-rod mechanism
double evalExpr(const std::string& expr, const std::string& params = "") {
    const Program p = parseWith("mechanism m { " + params + " ground g { anchor p at (0, 0) } "
                                "rod a { length: " + expr + " } }");
    return p.mechanisms[0].bodies[1].props.at("length");
}

bool parseFails(const std::string& src, const char* needle, const ParamOverrides& ov = {}) {
    try {
        parseWith(src, ov);
    } catch (const ParseError& e) {
        return std::string(e.msg).find(needle) != std::string::npos;
    }
    return false;
}

void testParams() {
    std::printf("[params M8]\n");

    // ---- tokens: the new operators, and the old ones undisturbed -----------------------------
    {
        const std::vector<Token> t = lex("a = 2*3/4 + 1 // a comment");
        std::vector<std::string> syms;
        for (const Token& k : t) if (k.type == Tok::Sym) syms.push_back(k.text);
        CHECK((syms == std::vector<std::string>{"=", "*", "/", "+"}));          // "//" was a comment
        const std::vector<Token> u = lex("drive x at 30 deg/s");
        bool sawUnit = false;
        for (const Token& k : u) if (k.type == Tok::UnitDegPerSec) sawUnit = true;
        CHECK(sawUnit);                                                          // "deg/s" is still one unit
    }

    // ---- expression evaluation ------------------------------------------------------------------
    CHECK_APPROX(evalExpr("2 + 3 * 4"), 14.0, 1e-15);                            // precedence
    CHECK_APPROX(evalExpr("(2 + 3) * 4"), 20.0, 1e-15);
    CHECK_APPROX(evalExpr("10 - 4 - 3"), 3.0, 1e-15);                            // left-associative
    CHECK_APPROX(evalExpr("100 / 10 / 5"), 2.0, 1e-15);
    CHECK_APPROX(evalExpr("2 * -3 + 10"), 4.0, 1e-15);                           // unary minus
    CHECK_APPROX(evalExpr("-(2 + 3) + 8"), 3.0, 1e-15);
    CHECK_APPROX(evalExpr("+4"), 4.0, 1e-15);
    CHECK_APPROX(evalExpr("pi"), kPi, 1e-15);
    CHECK_APPROX(evalExpr("2 * pi * 0.5"), kPi, 1e-15);
    CHECK_APPROX(evalExpr("sqrt(16)"), 4.0, 1e-15);
    CHECK_APPROX(evalExpr("sqrt(3*3 + 4*4)"), 5.0, 1e-15);
    CHECK_APPROX(evalExpr("sin(pi / 2)"), 1.0, 1e-15);
    CHECK_APPROX(evalExpr("cos(0) + tan(0) + atan(0)"), 1.0, 1e-15);
    CHECK_APPROX(evalExpr("abs(-7.5)"), 7.5, 1e-15);
    CHECK_APPROX(evalExpr("min(3, 8) + max(3, 8)"), 11.0, 1e-15);
    CHECK_APPROX(evalExpr("pow(2, 10)"), 1024.0, 1e-12);
    CHECK_APPROX(evalExpr("L * 2 + k", "param L = 0.25  param k = 1 / 4"), 0.75, 1e-15);   // params, and params of params
    CHECK_APPROX(evalExpr("b", "param a = 3 param b = a * a + 1"), 10.0, 1e-15);

    // a unit word after a number ends the expression exactly as before
    {
        const Program p = parseWith("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 } "
                                    "drive a at 30 + 6 deg/s from 90 - 30 deg }");
        const DriveDecl& d = p.mechanisms[0].drives[0];
        CHECK_APPROX(d.omegaRad, deg2rad(36.0), 1e-12);                          // (30 + 6) deg/s
        CHECK_APPROX(d.theta0Rad, deg2rad(60.0), 1e-12);                         // (90 - 30) deg
    }

    // ---- diagnostics ----------------------------------------------------------------------------------
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: 1 / 0 } }", "division by zero"));
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: foo } }", "declare it with 'param foo"));
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: sqrt(-1) } }", "negative"));
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: min(1) } }", "two arguments"));
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: sqrt(1, 2) } }", "one argument"));
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: (1 + 2 } }", "expected ')'"));
    CHECK(parseFails("mechanism m { param a = 1 param a = 2 }", "already declared"));
    CHECK(parseFails("mechanism m { param sqrt = 2 }", "cannot be a parameter name"));
    CHECK(parseFails("mechanism m { param base = 2 }", "cannot be a parameter name"));
    CHECK(parseFails("mechanism m { param pi = 3 }", "cannot be a parameter name"));
    CHECK(parseFails("mechanism m { param a = }", "expected number"));
    CHECK(parseFails("mechanism m { param a 3 }", "expected '='"));
    // a parameter is not visible before it is declared, nor in another mechanism
    CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } rod a { length: late } param late = 2 }", "not a number"));
    CHECK(parseFails("mechanism a { param x = 1 } mechanism b { ground g { anchor p at (x, 0) } }", "not a number"));

    // ---- overrides -------------------------------------------------------------------------------------
    {
        const std::string src = "mechanism m { param L = 0.25 param twice = L * 2 "
                                "ground g { anchor p at (0,0) } rod a { length: twice } }";
        const Program d = parseWith(src);
        CHECK_APPROX(d.mechanisms[0].bodies[1].props.at("length"), 0.5, 1e-15);
        CHECK(d.mechanisms[0].params.size() == 2 && !d.mechanisms[0].params[0].overridden);
        const Program o = parseWith(src, {{"L", 1.0}});
        CHECK_APPROX(o.mechanisms[0].bodies[1].props.at("length"), 2.0, 1e-15);   // dependents recompute
        CHECK(o.mechanisms[0].params[0].overridden);
        CHECK_APPROX(o.mechanisms[0].params[0].declared, 0.25, 1e-15);            // the file's own value is kept
        CHECK_APPROX(o.mechanisms[0].params[0].value, 1.0, 1e-15);
        const Program both = parseWith(src, {{"twice", 7.0}});                    // overriding the derived one wins
        CHECK_APPROX(both.mechanisms[0].bodies[1].props.at("length"), 7.0, 1e-15);
        // a name the file does not declare is an error that lists what it does
        CHECK(parseFails(src, "no such parameter", {{"M", 1.0}}));
        CHECK(parseFails(src, "it declares: L, twice", {{"M", 1.0}}));
        CHECK(parseFails("mechanism m { ground g { anchor p at (0,0) } }", "it declares none", {{"x", 1.0}}));
    }

    // ---- a parameter works wherever a number does: defaults equal the literal file ---------------------
    {
        const std::string lit =
            "mechanism m { world { gravity: 9.81 step: 0.001 } ground g { anchor a at (0, 0) anchor b at (0.1, 0) } "
            "shaft s at g.a  body w { anchor c at (0, 0) mass: 0.5 from: 20 deg } pin w.c -> g.b "
            "profile pg on w.c { polygon: (0.03, 0) (0, 0.03) (-0.03, 0) (0, -0.03) fillet 0.004 } "
            "profile pe on s { ellipse: 0.05 0.04 focus } profile pp on s { polar: (0 deg, 0.03) (120 deg, 0.04) (240 deg, 0.035) } "
            "cam lobe on s { base 0.025 roller 0.006 start 80 deg rise 0.012 over 100 deg harmonic dwell 60 deg "
            "return over 140 deg dwell } "
            "touch s.A on pg restitution 0.3 friction 0.2 roller 0.002 "
            "drive s at 30 rpm from 10 deg }";
        const std::string par =
            "mechanism m { param gv = 9.81 param stp = 0.001 param span = 0.1 param ms = 0.5 param a0 = 20 "
            "param r4 = 0.03 param fil = 0.004 param ea = 0.05 param eb = 0.04 param base_r = 0.025 param ro = 0.006 "
            "param st = 80 param lift = 0.012 param rise_deg = 100 param dw = 60 param ret = 140 "
            "param rest = 0.3 param mu = 0.2 param spd = 30 param ph = 10 "
            "world { gravity: gv step: stp } ground g { anchor a at (0, 0) anchor b at (span, 0) } "
            "shaft s at g.a  body w { anchor c at (0, 0) mass: ms from: a0 deg } pin w.c -> g.b "
            "profile pg on w.c { polygon: (r4, 0) (0, r4) (-r4, 0) (0, -r4) fillet fil } "
            "profile pe on s { ellipse: ea eb focus } profile pp on s { polar: (0 deg, r4) (120 deg, r4 + 0.01) (240 deg, 0.035) } "
            "cam lobe on s { base base_r roller ro start st deg rise lift over rise_deg deg harmonic dwell dw deg "
            "return over ret deg dwell } "
            "touch s.A on pg restitution rest friction mu roller 0.002 "
            "drive s at spd rpm from ph deg }";
        Model ml = buildFromText(lit.c_str()), mp = buildFromText(par.c_str());
        CHECK(mp.bodies.size() == ml.bodies.size() && mp.profiles.size() == ml.profiles.size());
        CHECK(mp.world.gravity == ml.world.gravity && mp.world.step == ml.world.step);
        CHECK(mp.cams.size() == 1 && mp.cams[0].maxLift == ml.cams[0].maxLift);
        CHECK(mp.profiles[1].curve->ellipseA() == ml.profiles[1].curve->ellipseA());
        CHECK(mp.touches[0].friction == ml.touches[0].friction && mp.touches[0].restitution == 0.3);
        CHECK(mp.drives[0].omegaRad == ml.drives[0].omegaRad && mp.drives[0].theta0Rad == ml.drives[0].theta0Rad);
        // `dwell` takes an expression too, and a bare `dwell` is still "the rest of the turn"
        CHECK(mp.cams[0].spec.segments.size() == 4 && mp.cams[0].spec.segments[3].restOfTurn);
        CHECK_APPROX(mp.cams[0].spec.segments[1].angle, deg2rad(60.0), 1e-15);
        std::vector<double> ql, qp;
        ml.assemble(ql);
        mp.assemble(qp);
        CHECK(ql == qp);                                                            // bit for bit
    }

    // ---- sweep axes ----------------------------------------------------------------------------------------
    {
        SweepAxis a;
        std::string err;
        CHECK(parseAxis("x=0:1:0.25", a, err) && a.name == "x" && a.values.size() == 5);
        CHECK_APPROX(a.values[4], 1.0, 1e-12);                                      // the end is included
        CHECK(parseAxis("x=0:1:0.1", a, err) && a.values.size() == 11);              // no float drift past 1
        CHECK(parseAxis("x=10:0:-5", a, err) && a.values.size() == 3 && a.values[2] == 0.0);   // descending
        CHECK(parseAxis("x=1,2.5,7", a, err) && a.values.size() == 3 && a.values[1] == 2.5);
        CHECK(parseAxis("x=3", a, err) && a.values.size() == 1);
        CHECK(!parseAxis("x", a, err));
        CHECK(!parseAxis("=3", a, err));
        CHECK(!parseAxis("x=abc", a, err) && err.find("not a number") != std::string::npos);
        CHECK(!parseAxis("x=0:1:0", a, err));
        CHECK(!parseAxis("x=0:1:-0.1", a, err) && err.find("never reaches") != std::string::npos);
        CHECK(!parseAxis("x=0:1", a, err));
        CHECK(!parseAxis("x=0:1:1e-9", a, err) && err.find("values") != std::string::npos);   // too many
        // 0.3 / 0.1 is 2.9999999999999996 in doubles: the endpoint must still be counted
        CHECK(parseAxis("x=0:0.3:0.1", a, err) && a.values.size() == 4);
        CHECK_APPROX(a.values[3], 0.3, 1e-12);
    }

    // ---- sweeps: a kinematic file against its closed form ---------------------------------------------------
    {
        // `final:arm.theta` after 1 s at w deg/s is exactly w degrees, for every cell of a grid
        const std::string src = "mechanism spin { param w = 30  param len = 0.2 "
                                "ground frame { anchor axle at (0, 0) } rod arm on frame.axle { length: len } "
                                "drive arm at w deg/s }";
        SweepAxis aw, al;
        std::string err;
        CHECK(parseAxis("w=30:90:30", aw, err));
        CHECK(parseAxis("len=0.1,0.3", al, err));
        SweepOptions opt;
        opt.seconds = 1.0;
        opt.reports = {"final:arm.theta", "maxres", "max:arm.B.x"};
        opt.probes = {"arm.B"};
        const SweepResult r = runSweep(src, {aw, al}, opt);
        CHECK(r.rows.size() == 6 && !r.dynamic);
        CHECK(r.axisNames.size() == 2 && r.axisNames[0] == "w" && r.metricLabels.size() == 3);
        for (size_t i = 0; i < r.rows.size(); ++i) {
            const SweepRow& row = r.rows[i];
            CHECK(row.error.empty());
            CHECK_APPROX(row.params[0], 30.0 + 30.0 * (double)(i / 2), 1e-12);     // first axis slowest
            CHECK_APPROX(row.params[1], i % 2 ? 0.3 : 0.1, 1e-12);
            CHECK_APPROX(row.metrics[0], deg2rad(row.params[0]), 1e-9);              // theta = w t
            CHECK(row.metrics[1] < 1e-9);
            CHECK(row.metrics[2] <= row.params[1] + 1e-9 && row.metrics[2] > 0.0);   // the tip stays within the rod's reach
        }
    }
    {   // the eccentric's highest and lowest point are R + rho -/+ e
        std::ifstream in("examples/eccentric.mech");
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            SweepAxis ae;
            std::string err;
            CHECK(parseAxis("e=0.005:0.02:0.005", ae, err));
            SweepOptions opt;
            opt.seconds = 1.0;
            opt.reports = {"max:follower.y", "min:follower.y"};      // unit omitted: resolves to follower.y[m]
            const SweepResult r = runSweep(ss.str(), {ae}, opt);
            CHECK(r.rows.size() == 4);
            for (const SweepRow& row : r.rows) {
                CHECK(row.error.empty());
                CHECK_APPROX(row.metrics[0], 0.036 + row.params[0], 1e-9);
                CHECK_APPROX(row.metrics[1], 0.036 - row.params[0], 1e-9);
            }
        }
    }

    // ---- sweeps: a dynamic file against Newton's law ---------------------------------------------------------
    // The jump-cam's follower lets go once the contact force would have to go negative:
    //   N cos(alpha) = m (w^2 s'' + g) + k (y - rest) = 0  at  w^2 = (g + k (y - rest)/m) / (-s'')
    // The critical speed is the smallest such w over the cycle.
    {
        std::ifstream in("examples/jumpcam.mech");
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            const std::string src = ss.str();
            Model m = buildModel(parseWith(src).mechanisms[0]);
            CamDiagram dia;
            CamError ce;
            CHECK(dia.build(m.cams[0].spec, ce));
            double w2min = 1e300;
            for (int k = 0; k < 360000; ++k) {
                double sv, ds, dds;
                dia.eval(2.0 * kPi * k / 360000.0, sv, ds, dds);
                if (dds < -1e-9) w2min = std::min(w2min, (9.81 + 200.0 * (0.031 + sv - 0.020) / 0.1) / (-dds));
            }
            const double rpmCrit = std::sqrt(w2min) * 30.0 / kPi;               // 443.9 rpm
            CHECK_APPROX(rpmCrit, 443.9, 0.1);

            SweepAxis as;
            std::string err;
            CHECK(parseAxis("speed=420:470:2", as, err));
            SweepOptions opt;
            opt.seconds = 0.25;
            opt.reports = {"liftoffs", "ledger"};
            const SweepResult r = runSweep(src, {as}, opt);
            CHECK(r.dynamic && r.rows.size() == 26);
            double first = -1.0;
            bool monotone = true, ledgerOk = true;
            for (size_t i = 0; i < r.rows.size(); ++i) {
                CHECK(r.rows[i].error.empty());
                if (r.rows[i].metrics[0] > 0.0 && first < 0.0) first = r.rows[i].params[0];
                if (first >= 0.0 && r.rows[i].metrics[0] == 0.0) monotone = false;   // once it jumps it keeps jumping
                ledgerOk = ledgerOk && r.rows[i].metrics[1] < 1e-5;
            }
            CHECK(first > 0.0);
            // never early (the force cannot vanish below the critical speed), and within one step of it
            CHECK(first >= rpmCrit - 0.5);
            CHECK(first <= rpmCrit + 2.0);
            CHECK(monotone);
            CHECK(ledgerOk);               // the books close in every run of the sweep
        }
    }

    // ---- metric reductions against their definitions ------------------------------------------------------------
    {
        // theta = w t is an arithmetic progression over the samples, so its mean is (first + last) / 2
        const std::string src = "mechanism spin { param w = 30 "
                                "ground frame { anchor axle at (0, 0) } rod arm on frame.axle { length: 0.2 } "
                                "drive arm at w deg/s }";
        SweepAxis aw;
        std::string err;
        CHECK(parseAxis("w=30,90", aw, err));
        SweepOptions opt;
        opt.seconds = 1.0;
        opt.reports = {"mean:arm.theta", "min:arm.theta", "max:arm.theta", "final:arm.theta"};
        const SweepResult r = runSweep(src, {aw}, opt);
        CHECK(r.rows.size() == 2 && r.rows[0].error.empty());
        for (const SweepRow& row : r.rows) {
            CHECK_APPROX(row.metrics[0], deg2rad(row.params[0]) / 2.0, 1e-9);
            CHECK_APPROX(row.metrics[1], 0.0, 1e-12);
            CHECK_APPROX(row.metrics[2], deg2rad(row.params[0]), 1e-9);
            CHECK_APPROX(row.metrics[3], row.metrics[2], 1e-12);
        }
    }
    {
        // the ledger metric is the worst |residual|, and the residual of this pendulum swings both ways
        // with the negative side the larger (-4.0e-10 against +3.3e-11)
        const std::string src = "mechanism p { param a0 = 60 world { gravity: 9.81 } "
                                "ground frame { anchor pivot at (0, 0) } rod arm { length: 1.0, from: a0 deg } on frame.pivot }";
        SweepAxis aa;
        std::string err;
        CHECK(parseAxis("a0=60", aa, err));
        SweepOptions opt;
        opt.seconds = 2.0;
        opt.reports = {"ledger"};
        const SweepResult r = runSweep(src, {aa}, opt);
        CHECK(r.rows.size() == 1 && r.rows[0].error.empty());
        const Model m = buildFromText(src.c_str());
        const SimSeries s = simulate(m, opt.seconds, opt.dt, {}, 1);
        int c = -1;
        for (size_t i = 0; i < s.columns.size(); ++i) if (s.columns[i] == "ledger.residual[J]") c = (int)i;
        CHECK(c >= 0);
        double lo = 0.0, hi = 0.0, worst = 0.0;
        for (const auto& row : s.rows) {
            lo = std::min(lo, row[(size_t)c]);
            hi = std::max(hi, row[(size_t)c]);
            worst = std::max(worst, std::fabs(row[(size_t)c]));
        }
        CHECK(lo < 0.0 && -lo > hi);                                  // so dropping fabs would change the answer
        CHECK(r.rows[0].metrics[0] == worst);
    }

    // ---- a design that becomes invalid partway reports it in its own row -------------------------------------
    {
        const std::string src =
            "mechanism c { param span = 100 "
            "ground frame { anchor axle at (0, 0)  anchor rail at (0, 0.09) dir (0, 1) } shaft cam at frame.axle "
            "cam lobe on cam.A { base 0.025 roller 0.006 rise 0.012 over span deg cycloidal dwell 60 deg "
            "return over 140 deg modtrap dwell } "
            "body follower { anchor tip at (0, 0) dir (1, 0) } slide follower.tip -> frame.rail "
            "touch follower.tip on lobe drive cam at 30 deg/s }";
        SweepAxis a;
        std::string err;
        CHECK(parseAxis("span=100,30,10,60", a, err));
        SweepOptions opt;
        opt.seconds = 12.0;                  // one full turn at 30 deg/s
        opt.reports = {"max:follower.y"};
        const SweepResult r = runSweep(src, {a}, opt);
        CHECK(r.rows.size() == 4);
        CHECK(r.rows[0].error.empty() && r.rows[1].error.empty());
        CHECK(r.rows[2].error.find("undercut") != std::string::npos);            // the compiler's own message
        CHECK(std::isnan(r.rows[2].metrics[0]));
        CHECK(r.rows[3].error.empty());                                          // and the sweep carried on past it
        CHECK_APPROX(r.rows[3].metrics[0], 0.043, 1e-6);
    }

    // ---- refusals that make the whole sweep meaningless ------------------------------------------------------
    {
        const std::string src = "mechanism spin { param w = 30 ground frame { anchor axle at (0, 0) } "
                                "rod arm on frame.axle { length: 0.2 } drive arm at w deg/s }";
        SweepAxis a;
        std::string err;
        parseAxis("w=30,60", a, err);
        auto throwsWith = [&](const SweepAxis& ax, const SweepOptions& o, const char* needle) {
            try {
                runSweep(src, {ax}, o);
            } catch (const std::runtime_error& e) {
                return std::string(e.what()).find(needle) != std::string::npos;
            }
            return false;
        };
        SweepOptions ok;
        CHECK(!throwsWith(a, ok, "x"));                                          // sanity: the plain sweep runs
        SweepAxis typo;
        parseAxis("ww=1,2", typo, err);
        CHECK(throwsWith(typo, ok, "no such parameter"));
        SweepOptions bad;
        bad.reports = {"max:nope"};
        CHECK(throwsWith(a, bad, "no recorded column"));
        bad.reports = {"liftoffs"};
        CHECK(throwsWith(a, bad, "needs a dynamic file"));
        bad.reports = {"median:arm.theta"};
        CHECK(throwsWith(a, bad, "unknown report kind"));
        bad.reports = {};
        bad.probes = {"frame.axle"};
        CHECK(throwsWith(a, bad, "moving body.anchor"));
        SweepAxis huge;
        parseAxis("w=0:4999:1", huge, err);
        SweepAxis huge2;
        parseAxis("w=0:2:1", huge2, err);
        // a grid of 5000 x 3 is refused up front rather than run for an hour
        try {
            runSweep(src, {huge, huge2}, ok);
            CHECK(false);
        } catch (const std::runtime_error& e) {
            CHECK(std::string(e.what()).find("5000") != std::string::npos);
        }
    }

    // ---- the command line, end to end ---------------------------------------------------------------------------
    if (std::ifstream("./mech")) {
        auto sh = [&](const std::string& cmd) {
            const int st = std::system((cmd + " >/dev/null 2>&1").c_str());
            return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
        };
        char tmpl[] = "/tmp/mech_sweep_XXXXXX";
        const int fd = mkstemp(tmpl);
        if (fd >= 0) close(fd);
        const std::string out = tmpl;
        CHECK(sh("./mech sweep examples/jumpcam.mech --set speed=300,600 --time 0.1 --report liftoffs --csv " + out) == 0);
        {
            std::ifstream in(out);
            std::string line;
            std::getline(in, line);
            CHECK(line == "speed,liftoffs,error");
            std::getline(in, line);
            CHECK(line.rfind("300,0,", 0) == 0);                                  // 300 rpm: never lets go
            std::getline(in, line);
            CHECK(line.rfind("600,", 0) == 0 && line != "600,0,");                // 600 rpm: it does
        }
        CHECK(sh("./mech check examples/jumpcam.mech --set speed=300") == 0);
        CHECK(sh("./mech check examples/jumpcam.mech --set sped=300") != 0);      // a typo is an error
        CHECK(sh("./mech check examples/jumpcam.mech --set speed=fast") != 0);
        CHECK(sh("./mech run examples/jumpcam.mech --headless 0.1 --set speed=300") == 0);
        CHECK(sh("./mech sweep examples/jumpcam.mech") != 0);                      // nothing to sweep
        CHECK(sh("./mech sweep examples/jumpcam.mech --set speed=1:2") != 0);      // bad range
        CHECK(sh("./mech sweep examples/jumpcam.mech --set sped=1,2") != 0);
        CHECK(sh("./mech sweep examples/jumpcam.mech --set speed=300 --report max:nope") != 0);
        unlink(tmpl);
    } else {
        std::fprintf(stderr, "note: ./mech not found (cwd?); skipping the command-line checks\n");
    }
}

// ============================================================================
// M9 — parts and instances. The oracle throughout is the flat file: an instanced
// mechanism must build the same model, and run the same numbers bit for bit, as the
// mechanism written out by hand — the part machinery is expansion and nothing else.
// ============================================================================

std::vector<std::vector<double>> runRows(const std::string& src, double seconds, const ParamOverrides& ov = {}) {
    const Model m = buildModel(parse(lex(src), ov).mechanisms[0]);
    return simulate(m, seconds, 1.0 / 60.0, {}, 1).rows;
}

std::string readFile(const char* path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// The body of a file's only mechanism with its `world { ... }` block cut out, and that block.
void splitMechanism(const std::string& src, std::string& body, std::string& world) {
    const size_t mk = src.find("mechanism ");
    const size_t nb = src.find('{', mk);
    if (mk == std::string::npos || nb == std::string::npos) { body.clear(); return; }
    body = src.substr(nb + 1, src.rfind('}') - nb - 1);
    const size_t w = body.find("world {");
    if (w != std::string::npos) {
        const size_t e = body.find('}', w);
        world = body.substr(w, e - w + 1);
        body.erase(w, e - w + 1);
    }
}

std::string wrapAsPart(const std::string& body, const std::string& world, const std::string& use) {
    return "part unit {" + body + "}\nmechanism wrapped {\n " + world +
           "\n ground mount { anchor o at (0, 0) }\n " + use + "\n}\n";
}

const char* kPlatePart = R"(
part plate_cam(lift = 0.012, base_r = 0.025) {
  ground frame {
    anchor axle at (0, 0)
    anchor rail at (0, 0.09) dir (0, 1)
  }
  shaft cam at frame.axle
  cam lobe on cam.A {
    base base_r
    roller 0.006
    rise lift over 100 deg cycloidal
    dwell 60 deg
    return over 140 deg modtrap
    dwell
  }
  body follower { anchor tip at (0, 0) dir (1, 0) }
  slide follower.tip -> frame.rail
  touch follower.tip on lobe
}
)";

bool partsFail(const std::string& src, const char* needle, int* lineOut = nullptr) {
    try {
        (void)parse(lex(src));
    } catch (const ParseError& e) {
        if (lineOut) *lineOut = e.line;
        return e.msg.find(needle) != std::string::npos;
    }
    return false;
}

void testParts() {
    std::printf("[parts M9]\n");

    // ---- two instances equal the flat file, bit for bit ----------------------------------------
    {
        const std::string inst = std::string(kPlatePart) + R"(
mechanism two {
  ground base { anchor origin at (0, 0) }
  use plate_cam(lift = 0.02) as left  on base at (0, 0)
  use plate_cam              as right on base at (0.2, 0)
  drive left.cam at 30 rpm
  drive right.cam at 60 rpm
})";
        const std::string flat = R"(
mechanism two {
  ground base {
    anchor origin at (0, 0)
    anchor l_axle at (0, 0)
    anchor l_rail at (0, 0.09) dir (0, 1)
    anchor r_axle at (0.2, 0)
    anchor r_rail at (0.2, 0.09) dir (0, 1)
  }
  shaft l_cam at base.l_axle
  cam l_lobe on l_cam.A { base 0.025 roller 0.006 rise 0.02 over 100 deg cycloidal dwell 60 deg
                          return over 140 deg modtrap dwell }
  body l_follower { anchor tip at (0, 0) dir (1, 0) }
  slide l_follower.tip -> base.l_rail
  touch l_follower.tip on l_lobe
  shaft r_cam at base.r_axle
  cam r_lobe on r_cam.A { base 0.025 roller 0.006 rise 0.012 over 100 deg cycloidal dwell 60 deg
                          return over 140 deg modtrap dwell }
  body r_follower { anchor tip at (0, 0) dir (1, 0) }
  slide r_follower.tip -> base.r_rail
  touch r_follower.tip on r_lobe
  drive l_cam at 30 rpm
  drive r_cam at 60 rpm
})";
        const Program pi = parse(lex(inst));
        const MechanismDecl& d = pi.mechanisms[0];
        CHECK(d.instances.size() == 2 && d.instances[0].name == "left" && d.instances[0].part == "plate_cam");
        CHECK(d.bodies.size() == 5);                                    // base + 2 x (cam, follower)
        const Model m = buildModel(d);
        const int il = m.bodyIndex("left.follower"), ir = m.bodyIndex("right.follower");
        CHECK(il > 0 && ir > 0);                                        // names are qualified by the instance
        const auto a = runRows(inst, 3.0), b = runRows(flat, 3.0);
        CHECK(a.size() == b.size() && a.size() > 100);
        CHECK(a == b);                                                  // every number of every row, exactly
        // the two instances are independent: the left rises 20 mm, the right 12
        double hl = 0.0, hr = 0.0;
        for (const auto& row : a) {
            hl = std::max(hl, row[1 + 3 * (size_t)(il - 1) + 2]);
            hr = std::max(hr, row[1 + 3 * (size_t)(ir - 1) + 2]);
        }
        CHECK_APPROX(hl, 0.031 + 0.02, 1e-6);
        CHECK_APPROX(hr, 0.031 + 0.012, 1e-6);
    }

    // ---- a whole dynamic file as one part: the clock, 30 s of hybrid dynamics ------------------
    {
        const std::string clock = readFile("examples/clock.mech");
        std::string body, world;
        splitMechanism(clock, body, world);
        if (body.empty()) {
            std::fprintf(stderr, "note: examples/clock.mech not found (cwd?)\n");
        } else {
            const auto flat = runRows(clock, 30.0);
            const auto wrapped = runRows(wrapAsPart(body, world, "use unit as c on mount at (0, 0)"), 30.0);
            CHECK(flat.size() > 1000);
            CHECK(flat == wrapped);       // escapement, reductions, hands, dials, weight drive: every row identical
            // moved elsewhere in the world it is still the same clock
            const auto moved = runRows(wrapAsPart(body, world, "use unit as c on mount at (5, -3)"), 30.0);
            CHECK(moved.size() == flat.size());
            bool same = true;
            for (size_t r = 0; r < flat.size(); ++r)
                same = same && std::fabs(flat[r].back() - moved[r].back()) < 1e-9;   // pendulum omega
            CHECK(same);
        }
    }

    // ---- a dynamic part with parameters: the jumping cam, overridden through the instance ------
    {
        const std::string jump = readFile("examples/jumpcam.mech");
        std::string body, world;
        splitMechanism(jump, body, world);
        if (!body.empty()) {
            const std::string wrapped = wrapAsPart(body, world, "use unit as u on mount at (0, 0)");
            // part-internal parameters carry the instance's name: --set u.speed
            ParamOverrides flatOv, wrapOv;
            flatOv["speed"] = 300.0;
            wrapOv["u.speed"] = 300.0;
            CHECK(runRows(jump, 0.3, flatOv) == runRows(wrapped, 0.3, wrapOv));
            CHECK(runRows(jump, 0.3) == runRows(wrapped, 0.3));
            CHECK(runRows(jump, 0.3, flatOv) != runRows(jump, 0.3));               // the override does something
            CHECK(!parseFails(wrapped, "no such parameter", wrapOv));
            CHECK(parseFails(wrapped, "u.speed", ParamOverrides{{"speed", 300.0}}));   // the bare name is not a parameter
            // and a sweep over the instance's parameter finds the same lift-off counts and ledger as the flat one
            SweepAxis af, aw;
            std::string err;
            CHECK(parseAxis("speed=300,600", af, err));
            CHECK(parseAxis("u.speed=300,600", aw, err));
            SweepOptions opt;
            opt.seconds = 0.15;
            opt.reports = {"liftoffs", "ledger"};
            const SweepResult rf = runSweep(jump, {af}, opt);
            const SweepResult rw = runSweep(wrapped, {aw}, opt);
            CHECK(rf.rows.size() == 2 && rw.rows.size() == 2);
            for (size_t i = 0; i < 2 && i < rw.rows.size(); ++i) {
                CHECK(rw.rows[i].error.empty());
                CHECK(rw.rows[i].metrics[0] == rf.rows[i].metrics[0]);
                CHECK(rw.rows[i].metrics[1] == rf.rows[i].metrics[1]);
            }
            CHECK(rw.rows[0].metrics[0] == 0.0 && rw.rows[1].metrics[0] > 0.0);
        }
    }

    // ---- part parameters: defaults, arguments, overrides ---------------------------------------
    {
        const std::string src = R"(
part link(len = 0.1, ext = len * 2, n = 3) {
  param total = len + ext + n * 0.001
  ground g { anchor p at (0, 0) }
  rod r on g.p { length: total }
}
mechanism m {
  ground frame { anchor o at (0, 0) }
  param bs = 0.05
  use link as a on frame at (0, 0)
  use link(len = bs * 2) as b on frame at (1, 0)
  use link(len = 1, ext = 2, n = 0) as c on frame at (2, 0)
})";
        auto lengthOf = [](const Program& p, const char* body) {
            for (const BodyDecl& b : p.mechanisms[0].bodies) if (b.name == body) return b.props.at("length");
            return -1.0;
        };
        const Program p = parse(lex(src));
        const MechanismDecl& d = p.mechanisms[0];
        CHECK_APPROX(lengthOf(p, "a.r"), 0.1 + 0.2 + 0.003, 1e-15);    // ext defaults from len, n from its default
        CHECK_APPROX(lengthOf(p, "b.r"), 0.1 + 0.2 + 0.003, 1e-15);    // argument evaluated in the caller's scope (base * 2 = 0.1)
        CHECK_APPROX(lengthOf(p, "c.r"), 3.0, 1e-15);
        auto param = [&](const Program& pr, const char* n) -> const ParamDecl* {
            for (const ParamDecl& q : pr.mechanisms[0].params) if (q.name == n) return &q;
            return nullptr;
        };
        CHECK(param(p, "a.len") && param(p, "a.len")->value == 0.1 && !param(p, "a.len")->overridden);
        CHECK(param(p, "c.total") && param(p, "c.total")->value == 3.0);
        CHECK(param(p, "bs") && !param(p, "len"));                    // a part's names do not leak out
        CHECK(d.params.size() == 1 + 3 * 4);                           // base + (len, ext, n, total) per instance

        // --set reaches an instance's parameter, and what depends on it follows
        const Program q = parse(lex(src), ParamOverrides{{"a.len", 0.5}, {"c.n", 10.0}});
        CHECK_APPROX(lengthOf(q, "a.r"), 0.5 + 1.0 + 0.003, 1e-15);   // ext = len * 2 followed
        CHECK_APPROX(lengthOf(q, "b.r"), 0.1 + 0.2 + 0.003, 1e-15);   // the other instances untouched
        CHECK_APPROX(lengthOf(q, "c.r"), 3.0 + 0.010, 1e-15);
        CHECK(param(q, "a.len") && param(q, "a.len")->overridden && param(q, "a.len")->declared == 0.1);
        // an override beats an argument too
        const Program r = parse(lex(src), ParamOverrides{{"c.len", 5.0}});
        CHECK_APPROX(lengthOf(r, "c.r"), 7.0, 1e-15);
        // an unknown instance is an error that lists the real parameters
        CHECK(parseFails(src, "a.len", ParamOverrides{{"zz.len", 1.0}}));
    }

    // ---- parts inside parts ----------------------------------------------------------------------
    {
        const std::string nested = std::string(kPlatePart) + R"(
part pair(lift = 0.012) {
  ground g { anchor m at (0, 0) }
  use plate_cam(lift = lift) as a on g at (0, 0)
  use plate_cam(lift = lift * 2) as b on g at (0.2, 0)
}
mechanism m {
  ground base { anchor origin at (0, 0) }
  use pair(lift = 0.01) as p on base at (0.1, 0)
  drive p.a.cam at 30 rpm
  drive p.b.cam at 30 rpm
})";
        const std::string flat = R"(
mechanism m {
  ground base {
    anchor origin at (0, 0)
    anchor g_m at (0.1, 0)
    anchor a_axle at (0.1, 0)
    anchor a_rail at (0.1, 0.09) dir (0, 1)
    anchor b_axle at (0.1 + 0.2, 0)
    anchor b_rail at (0.1 + 0.2, 0.09) dir (0, 1)
  }
  shaft a_cam at base.a_axle
  cam a_lobe on a_cam.A { base 0.025 roller 0.006 rise 0.01 over 100 deg cycloidal dwell 60 deg
                          return over 140 deg modtrap dwell }
  body a_follower { anchor tip at (0, 0) dir (1, 0) }
  slide a_follower.tip -> base.a_rail
  touch a_follower.tip on a_lobe
  shaft b_cam at base.b_axle
  cam b_lobe on b_cam.A { base 0.025 roller 0.006 rise 0.02 over 100 deg cycloidal dwell 60 deg
                          return over 140 deg modtrap dwell }
  body b_follower { anchor tip at (0, 0) dir (1, 0) }
  slide b_follower.tip -> base.b_rail
  touch b_follower.tip on b_lobe
  drive a_cam at 30 rpm
  drive b_cam at 30 rpm
})";
        const Program p = parse(lex(nested));
        CHECK(p.mechanisms[0].instances.size() == 3);
        CHECK(p.mechanisms[0].instances[0].name == "p" && p.mechanisms[0].instances[1].name == "p.a");
        CHECK(runRows(nested, 2.0) == runRows(flat, 2.0));
        // a nested parameter is reached by its full name; an outer one flows into the inner argument
        const Program q = parse(lex(nested), ParamOverrides{{"p.a.lift", 0.02}, {"p.lift", 0.011}});
        double la = 0.0, lb = 0.0;
        for (const CamDecl& c : q.mechanisms[0].cams) {
            for (const CamSegment& sg : c.spec.segments) {
                if (sg.kind != CamSegment::Kind::Rise) continue;
                (c.name == "p.a.lobe" ? la : lb) = sg.lift;
            }
        }
        CHECK_APPROX(la, 0.02, 1e-15);                                  // overridden directly
        CHECK_APPROX(lb, 0.022, 1e-15);                                 // 2 x the overridden outer lift
    }

    // ---- the mount moves the whole part, in y as well as x --------------------------------------------
    {
        const std::string src = std::string(kPlatePart) + R"(
mechanism m {
  ground bench { anchor origin at (0, 0) }
  use plate_cam as c on bench at (0.2, 0.05)
  drive c.cam at 30 rpm
})";
        const Model m = buildModel(parse(lex(src)).mechanisms[0]);
        const int ic = m.bodyIndex("c.cam"), iff = m.bodyIndex("c.follower");
        const auto rows = runRows(src, 2.0);
        double top = 0.0;
        for (const auto& row : rows) top = std::max(top, row[1 + 3 * (size_t)(iff - 1) + 2]);
        CHECK_APPROX(rows[0][1 + 3 * (size_t)(ic - 1) + 1], 0.2, 1e-12);            // the shaft sits at the mount
        CHECK_APPROX(rows[0][1 + 3 * (size_t)(ic - 1) + 2], 0.05, 1e-12);
        CHECK_APPROX(top, 0.05 + 0.031 + 0.012, 1e-6);                              // the lift is measured from there
    }

    // ---- a part's own ground keeps its mount after a nested part has been placed ---------------------
    {
        const std::string src = std::string(kPlatePart) + R"(
part outer {
  ground g { anchor m at (0, 0) }
  use plate_cam as a on g at (0.2, 0.07)
  ground h { anchor q at (0.01, 0.02) }
}
mechanism m {
  ground base { anchor origin at (0, 0) }
  use outer as p on base at (0.1, 0.03)
})";
        const Program pr = parse(lex(src));
        const BodyDecl* base = nullptr;
        for (const BodyDecl& b : pr.mechanisms[0].bodies) if (b.name == "base") base = &b;
        CHECK(base != nullptr);
        auto at = [&](const char* n, double x, double y) {
            if (!base) return false;
            for (const AnchorDecl& a : base->anchors)
                if (a.name == n) return std::fabs(a.pos.x - x) < 1e-15 && std::fabs(a.pos.y - y) < 1e-15;
            return false;
        };
        CHECK(at("p.h.q", 0.1 + 0.01, 0.03 + 0.02));            // h is mounted where the outer part is, not where the inner went
        CHECK(at("p.g.m", 0.1, 0.03));
        CHECK(at("p.a.frame.axle", 0.1 + 0.2, 0.03 + 0.07));
    }

    // ---- every kind of name site, in a kinematic gear train and in the clock -------------------------------
    {
        const std::string train = readFile("examples/train.mech");
        std::string body, world;
        splitMechanism(train, body, world);
        if (!body.empty()) {
            const std::string wrapped = wrapAsPart(body, world, "use unit as t on mount at (0, 0)");
            const Program pr = parse(lex(wrapped));
            const MechanismDecl& d = pr.mechanisms[0];
            bool via = false;
            for (const ReduceDecl& r : d.reduces) {
                CHECK(r.bodyA.rfind("t.", 0) == 0 && r.bodyB.rfind("t.", 0) == 0);
                for (const std::string& v : r.via) { via = true; CHECK(v.rfind("t.", 0) == 0); }
            }
            CHECK(via);                                                  // the file does use `via`
            CHECK(runRows(train, 3.0) == runRows(wrapped, 3.0));
        }
        const std::string clock = readFile("examples/clock.mech");
        splitMechanism(clock, body, world);
        if (!body.empty()) {
            const Program cp = parse(lex(wrapAsPart(body, world, "use unit as c on mount at (0, 0)")));
            const MechanismDecl& d = cp.mechanisms[0];
            CHECK(d.modes.size() == 1 && d.modes[0].name == "c.esc");
            CHECK(d.hands.size() == 3);
            for (const HandDecl& h : d.hands) CHECK(h.name.rfind("c.", 0) == 0 && h.host.rfind("c.", 0) == 0);
            for (const GearDecl& g : d.gears) CHECK(g.name.rfind("c.", 0) == 0 && g.host.rfind("c.", 0) == 0);
            for (const DriveDecl& dr : d.drives) CHECK(dr.body.rfind("c.", 0) == 0);
        }
    }

    // ---- composition without grounds: the shipped crank-slider built from one `link` part ------------------
    {
        const std::string parts = readFile("examples/linkage_parts.mech"), flat = readFile("examples/crankslider.mech");
        if (!parts.empty() && !flat.empty()) CHECK(runRows(parts, 2.0) == runRows(flat, 2.0));
        else std::fprintf(stderr, "note: examples not found (cwd?)\n");
    }

    // ---- misuse is refused where it is written ---------------------------------------------------
    {
        const std::string pre = std::string(kPlatePart) + "mechanism m { ground base { anchor o at (0, 0) } ";
        CHECK(partsFail(pre + "use nope as x }", "unknown part 'nope'"));
        CHECK(partsFail(pre + "use nope as x }", "declared: plate_cam"));
        CHECK(partsFail(pre + "use plate_cam(zz = 1) as x on base at (0, 0) }", "has no parameter 'zz'"));
        CHECK(partsFail(pre + "use plate_cam(zz = 1) as x on base at (0, 0) }", "it has: lift, base_r"));
        CHECK(partsFail(pre + "use plate_cam(lift = 1, lift = 2) as x on base at (0, 0) }", "given twice"));
        CHECK(partsFail(pre + "use plate_cam as x }", "mounted"));                                  // its ground has nowhere to go
        CHECK(partsFail(pre + "use plate_cam as x on nowhere at (0, 0) }", "not a ground declared before"));
        CHECK(partsFail(pre + "use plate_cam on base at (0, 0) }", "expected 'as'"));
        CHECK(partsFail(pre + "use plate_cam as x on base at (0, 0) use plate_cam as x on base at (1, 0) }",
                        "already the name of a part instance"));
        CHECK(partsFail(pre + "body x { anchor a at (0, 0) } use plate_cam as x on base at (0, 0) }", "already a body name"));
        CHECK(partsFail(pre + "use plate_cam as x on base at (0, 0) body x { anchor a at (0, 0) } }",
                        "already the name of a part instance"));
        CHECK(partsFail("part p { } part p { } mechanism m { }", "part 'p' is already declared"));
        CHECK(partsFail("part p(a = 1, a = 2) { } mechanism m { }", "declared twice"));
        CHECK(partsFail("part p(pi = 1) { } mechanism m { }", "cannot be a parameter name"));
        CHECK(partsFail("part p { world { gravity: 9.81 } } mechanism m { }", "cannot contain a 'world' block"));
        CHECK(partsFail("part p { use p as x } mechanism m { }", "uses itself"));
        CHECK(partsFail("part a { use b as x } part b { } mechanism m { }", "unknown part 'b'"));   // declared in order
        CHECK(partsFail("mechanism m { ground g { anchor o at (0,0) } use late as x on g at (0, 0) } part late { }",
                        "unknown part 'late'"));
        // a part with no ground cannot be mounted
        CHECK(partsFail("part nog { shaft s } mechanism m { ground g { anchor o at (0,0) } use nog as x on g at (0, 0) }",
                        "has no ground"));
        // ... and one without a mount is fine when it has no ground of its own
        CHECK(!partsFail("part nog { body b { anchor a at (0, 0) } } mechanism m { ground g { anchor o at (0,0) } use nog as x }", "x"));

        // a mistake inside a part is reported where it is written, even if the part is never used
        int line = 0;
        const std::string bad = "// first line\npart broken {\n  ground g { anchor a at (0, 0) }\n  rod r on g.a { lenth: 1 }\n}\nmechanism m { }\n";
        CHECK(partsFail(bad, "rod property", &line));
        CHECK(line == 4);
        // ... and a semantic error names the instance it happened in
        bool caught = false;
        try {
            (void)buildModel(parse(lex(std::string(kPlatePart) +
                "mechanism m { ground base { anchor o at (0, 0) } use plate_cam(base_r = -1) as left on base at (0, 0) "
                "drive left.cam at 30 rpm }")).mechanisms[0]);
        } catch (const SemanticError& e) {
            caught = e.msg.find("left.lobe") != std::string::npos;
        }
        CHECK(caught);
    }

    // ---- parts stay out of the way of the old language ---------------------------------------------
    {
        // 'part', 'use' and 'as' are only words in these positions
        const Program p = parse(lex("mechanism m { ground use { anchor part at (0, 0) } rod as on use.part { length: 1 } "
                                    "drive as at 30 deg/s }"));
        CHECK(p.mechanisms[0].instances.empty() && p.mechanisms[0].bodies.size() == 2);
        // a second mechanism does not see the first one's instances
        const Program q = parse(lex(std::string(kPlatePart) +
            "mechanism a { ground g { anchor o at (0, 0) } use plate_cam as x on g at (0, 0) } "
            "mechanism b { ground x { anchor o at (0, 0) } rod r on x.o { length: 1 } }"));
        CHECK(q.mechanisms.size() == 2 && q.mechanisms[1].instances.empty() && q.mechanisms[1].bodies.size() == 2);
    }

    // ---- sweeps and the command line through instance names ----------------------------------------
    {
        const std::string src = std::string(kPlatePart) + R"(
mechanism two {
  ground base { anchor origin at (0, 0) }
  use plate_cam as left  on base at (0, 0)
  use plate_cam as right on base at (0.2, 0)
  drive left.cam at 30 rpm
  drive right.cam at 30 rpm
})";
        SweepAxis a;
        std::string err;
        CHECK(parseAxis("left.lift=0.012,0.02,0.03", a, err) && a.name == "left.lift" && a.values.size() == 3);
        SweepOptions opt;
        opt.seconds = 2.0;
        opt.reports = {"max:left.follower.y", "max:right.follower.y"};
        const SweepResult r = runSweep(src, {a}, opt);
        CHECK(r.rows.size() == 3);
        for (size_t i = 0; i < r.rows.size(); ++i) {
            CHECK(r.rows[i].error.empty());
            CHECK_APPROX(r.rows[i].metrics[0], 0.031 + a.values[i], 1e-6);          // closed form: base + roller + lift
            CHECK_APPROX(r.rows[i].metrics[1], 0.043, 1e-6);                        // the other cam stays at its design
        }

        char path[] = "/tmp/mech_parts_XXXXXX";
        const int fd = mkstemp(path);
        CHECK(fd >= 0);
        if (fd >= 0) {
            const ssize_t w = write(fd, src.data(), src.size());
            CHECK(w == (ssize_t)src.size());
            close(fd);
            const std::string csv = std::string(path) + ".csv";
            const std::string cmd = std::string("./mech run ") + path + " --headless 1 --set left.lift=0.03 --csv " + csv +
                                    " --probe right.follower.tip > /dev/null 2>&1";
            CHECK(std::system(cmd.c_str()) == 0);
            std::ifstream in(csv);
            std::string header;
            std::getline(in, header);
            CHECK(header.find("right.follower.tip.x[m]") != std::string::npos);     // probe through a dotted body name
            CHECK(header.find("left.follower.tip@left.lobe.gap[m]") != std::string::npos);
            std::remove(csv.c_str());
            const std::string bare = std::string("./mech check ") + path + " --set lift=0.03 > /dev/null 2>&1";
            CHECK(std::system(bare.c_str()) != 0);                                  // the bare name is not a parameter
            std::remove(path);
        }
    }
}

} // namespace (anonymous)

namespace mech {

int runAllTests() {
    testLexer();
    testParser();
    testParserM2();
    testModel();
    testModelM2();
    testKinematics();
    testKinematicsM2();
    testParserM3();
    testModelM3();
    testKinematicsM3();
    testDynamics();
    testParserM4();
    testModelM4();
    testKinematicsM4();
    testParserM5();
    testModelM5();
    testDynamicsM5();
    testProfileCurve();
    testParserM6();
    testModelM6();
    testKinematicsM6();
    testCamLaws();
    testCamDiagram();
    testCamDesign();
    testParserModelM6b();
    testKinematicsM6b();
    testContactDynamics();
    testNonCircularMesh();
    testPolygonRatchet();
    testFriction();
    testRecorder();
    testParams();
    testParts();
    std::printf("tests: %d checks, %d failure%s\n", g_checks, g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures;
}

} // namespace mech
