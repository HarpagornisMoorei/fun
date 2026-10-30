// mech — tests.cpp: unit tests for the M1 + M2 core (no OpenGL required)
#include "tests.hpp"

#include "../src/geom.hpp"
#include "../src/dynamics.hpp"
#include "../src/kinematics.hpp"
#include "../src/lexer.hpp"
#include "../src/model.hpp"
#include "../src/parser.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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
    std::printf("tests: %d checks, %d failure%s\n", g_checks, g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures;
}

} // namespace mech
