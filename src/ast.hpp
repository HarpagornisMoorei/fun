// mech — ast.hpp: parse tree for the .mech language (M4)
#pragma once
#include "geom.hpp"
#include <map>
#include <string>
#include <vector>

namespace mech {

// An anchor may carry a direction: the axis of a slide rail (on ground) or
// the guide axis of a sliding body. Directional anchors are what makes
// prismatic joints expressible in the port model.
struct AnchorDecl {
    std::string name;
    Vec2 pos;               // body-local (ground: world) position
    bool hasDir = false;
    Vec2 dir{1.0, 0.0};     // body-local (ground: world) direction
    int line = 0;
};

// World parameters: presence of a `world` block switches the mechanism from
// kinematic (prescribed motion only) to dynamic (masses + forces + gravity).
// The same is true of any spring or torque drive.
struct WorldDecl {
    bool present = false;
    double gravity = 9.81;  // m/s^2, acting along -y
    double step = 0.002;    // integration timestep, seconds
    int line = 0;
};

struct BodyDecl {
    enum class Kind { Ground, Rod, Shaft, Free };
    Kind kind = Kind::Rod;
    std::string name;
    std::vector<AnchorDecl> anchors;      // ground / free bodies
    std::map<std::string, double> props;  // rod: { length, mass, from }
                                          // free body: { mass }
    // attach sugar: "rod crank on base.p1 { ... }" / "shaft a at frame.p1"
    bool hasAttach = false;
    std::string attachBody, attachAnchor;
    int line = 0;
};

// pin  <A>.<a> -> <B>.<b>     revolute: welds two anchor points
// slide <A>.<a> -> <B>.<b>    prismatic: A's guide anchor rides B's rail
//                             (B must be ground in M2; both need dirs)
struct JointDecl {
    enum class Kind { Pin, Slide };
    Kind kind = Kind::Pin;
    std::string bodyA, anchorA, bodyB, anchorB;
    int line = 0;
};

// gear g1 { teeth: 8 } on a   — pitch-circle feature of body 'a'.
// M3: the host may be any body including ground (a fixed ring), and the
// pitch circle may be centered on any host anchor (`on a.A` or `on a`).
struct GearDecl {
    std::string name;
    std::map<std::string, double> props;  // teeth / module / radius / mass
    std::string host;
    std::string hostAnchor;               // "" -> default anchor "A"
    int line = 0;
};

// mesh g1 g2 [tangent] — roll without slip; `tangent` additionally enforces
// the center-distance (pitch-circle tangency) row live, which is what lets
// unconstrained planets orbit their suns without a carrier.
struct MeshDecl {
    std::string gearA, gearB;
    bool tangent = false;
    int line = 0;
};

// spring sp a.h -> b.k { k: 9, rest: 1.2, damp: 0.1 }
// rest defaults to the assembled length (captured at t = 0); damp to 0.
struct SpringDecl {
    std::string name;
    std::string bodyA, anchorA, bodyB, anchorB;
    std::map<std::string, double> props;  // k / rest / damp
    int line = 0;
};

// drive <body> at <rate> <unit> [from <angle> deg]   — prescribed motion
// drive <body> torque <num>                          — energy injection
// drive <body> weight <m> [kg] [barrel <r> [m]] [drop <d> [m]]  — M5:
//   an energy source. A weight on a cord wound on a barrel of radius r
//   (default: the host's largest gear) applies torque m*g*r; 'drop' bounds
//   the fall (budget m*g*d) after which the torque ceases.
struct DriveDecl {
    std::string body;
    double omegaRad = 0.0;   // angular velocity, radians per second
    double theta0Rad = 0.0;  // initial angle, radians
    bool hasFrom = false;
    bool isTorque = false;   // M3: torque drive (a force, not a constraint)
    double torque = 0.0;     // N*m, applied about the body's frame origin
    bool isWeight = false;   // M5: weight drive (a bounded energy source)
    double weightMass = 0.0; // kg
    int weightSign = 1;      // +1: unwinds ccw (+theta); -1: 'cw'
    bool hasBarrel = false;
    double barrelR = 0.0;    // m
    bool hasDrop = false;
    double dropM = 0.0;      // m
    int line = 0;
};

// M4 — the intent layer: the author states WHAT the train must do; the
// compiler synthesizes teeth counts, meshes and (unless pinned) shaft
// positions. Desugars entirely into the M3 kernel: shafts + gears + meshes
// + pins to auto-created ground anchors.
//   reduce s1->s4 ratio 1:1800 [via s2 s3] [same|opposite]
//          [module: 0.006] [spread: 30 deg]
// 'via' names the intermediate shafts (which must be declared, position
// optional); without it they are synthesized and named '<A>_1', '<A>_2'...
// 'same'/'opposite' constrain the rotation sense of B relative to A
// (external meshes flip sense per stage; an idler shaft is inserted when
// the parity cannot be met by stage count alone).
struct ReduceDecl {
    std::string bodyA, bodyB;
    long long ratioNum = 1, ratioDen = 1;   // speed ratio A:B as written
    std::vector<std::string> via;
    enum class Dir { Any, Same, Opposite };
    Dir dir = Dir::Any;
    bool hasModule = false;
    double module = 0.0;
    bool hasSpread = false;
    double spreadDeg = 0.0;
    int line = 0;
};

// hand second on s4 { length: 0.3 } — a clock hand: a render-only needle
// mounted on a host anchor, rotating with the host. Pure intent: no
// constraint rows, no mass. (Length default 0.3.)
struct HandDecl {
    std::string name;
    std::string host;
    std::string hostAnchor;               // "" -> default anchor "A"
    std::map<std::string, double> props;  // length
    int line = 0;
};

// dial { radius: 0.36, major: 12, minor: 60 } [at frame.c] — a static
// tick ring. Without 'at' it auto-centers on the average of all hand
// mounts; without 'radius' it sizes to 1.2 x the longest hand.
struct DialDecl {
    bool hasAt = false;
    std::string atBody, atAnchor;
    std::map<std::string, double> props;  // radius / major / minor
    int line = 0;
};

// M5 — a hybrid element: a state machine over the mechanism's constraint
// sets, the kernel's only non-smooth element (the escapement is the flagship).
//   mode esc {
//     state tick { hold escape }              // locked: entry pallet engaged
//     state run  { }                          // free: the weight runs the wheel
//     tick -> run  when pend.theta     > -87.5 deg
//     run  -> tock when escape.advance > 6 deg: impulse pend maintain 3 deg
//   }
// Semantics: a 'hold' is a rate-0 drive row whose target is captured each
// time its state is entered (so switching states switches constraint sets);
// a transition fires when its guard crosses the threshold from below
// (rising edge) during a step; 'advance' is the guard body's angle travelled
// since state entry. Resets apply at the crossing instant.
struct ModeDecl {
    struct State {
        std::string name;
        std::vector<std::string> holds;   // body names held in this state
        int line = 0;
    };
    struct Reset {
        enum class Kind { Impulse, Maintain };
        Kind kind = Kind::Impulse;
        std::string body;
        double energy = 0.0;   // J  (impulse form; sign allowed)
        double swingDeg = 0.0; // maintain: target half-swing angle
        int line = 0;
    };
    struct Transition {
        std::string from, to;
        std::string body;      // guard body
        enum class Field { Theta, Omega, Advance };
        Field field = Field::Theta;
        bool greater = true;   // '>' vs '<'
        double threshold = 0.0; // radians (theta/advance) or rad/s (omega)
        std::vector<Reset> resets;
        int line = 0;
    };
    std::string name;
    std::vector<State> states;
    std::vector<Transition> transitions;
    int line = 0;
};

struct MechanismDecl {
    std::string name;
    WorldDecl world;
    std::vector<BodyDecl> bodies;
    std::vector<JointDecl> joints;
    std::vector<GearDecl> gears;
    std::vector<MeshDecl> meshes;
    std::vector<SpringDecl> springs;
    std::vector<DriveDecl> drives;
    std::vector<ReduceDecl> reduces;   // M4 intent layer
    std::vector<HandDecl> hands;
    std::vector<DialDecl> dials;
    std::vector<ModeDecl> modes;       // M5 hybrid elements
    int line = 0;
};

struct Program {
    std::vector<MechanismDecl> mechanisms;
};

} // namespace mech
