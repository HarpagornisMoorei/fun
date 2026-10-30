// mech — model.hpp: runtime model of a mechanism (M4)
//
// A mechanism is a graph:
//   - bodies   (nouns):  ground frames, rods, shafts and free rigid bodies,
//                        each exposing named anchors (ports), optionally
//                        directional (slide rails / guides)
//   - pins     (verbs):  revolute joints welding two anchors together
//   - slides   (verbs):  prismatic joints: a guide anchor rides a ground rail
//   - gears    (features): pitch circles centered on any host anchor,
//                        rotating with the host (ground hosts = fixed rings)
//   - meshes   (grammar): gear pairs roll without slip; `tangent` meshes also
//                        enforce pitch-circle contact live
//   - springs  (forces): Hooke elements between two anchors (M3)
//   - drives   (energy): prescribed-coordinate constraints (rate) or pure
//                        generalized torques (torque) — the solver decides
//                        what the rest of the mechanism does (non-causal)
//
// bodies[0] is always the (single) ground body; its anchor coordinates and
// directions are world coordinates. Non-ground bodies carry a pose
// (theta, x, y) packed into the solver's unknown vector q at qOffset.
//
// Sugar (desugared here, keeping source lines for diagnostics):
//   rod r on A.p { length: L }  ->  body r {A=(0,0), B=(L,0)}  + pin r.A->A.p
//   shaft s at A.p              ->  body s {A=(0,0)}           + pin s.A->A.p
//   gear g { teeth: N } on b    ->  pitch circle radius m*N/2 centered at b.A
//   gear g { teeth: N } on b.c  ->  the same, centered at anchor b.c
//   mesh g1 g2                  ->  roll row + tangency validation
//   mesh g1 g2 tangent          ->  roll row + live tangency row
//   profile p { ... } on b.c    ->  closed curve centered at anchor b.c (M6)
//   touch f.tip on p [roller r] ->  one row: signed gap to the curve = r (M6)
//
// Dynamics (M3): when a `world` block, a spring or a torque drive is present
// the mechanism is *dynamic* — masses matter (auto-derived from geometry when
// not given explicitly) and motion is integrated, not solved. Kinematic files
// compile and run exactly as they did in M2.

#pragma once
#include "ast.hpp"
#include "cam.hpp"
#include "geom.hpp"
#include "profile.hpp"
#include <memory>
#include <string>
#include <vector>

namespace mech {

struct SemanticError {
    int line;
    std::string msg;
};

struct Anchor {
    std::string name;
    Vec2 local;                 // position in body frame; ground: world
    bool hasDir = false;
    Vec2 dirLocal{1.0, 0.0};    // direction in body frame; ground: world
};

struct Body {
    std::string name;
    bool isGround = false;
    bool isShaft = false;        // zero-extent rotational body (M4 flag)
    std::vector<Anchor> anchors;
    double length = 0.0;        // rods: distance between anchors A and B
    double beatSeconds = 0.0;   // M4: > 0 when the length came from 'beat:'
    int qOffset = -1;           // ground: -1; otherwise index of (theta,x,y) in q
    bool hasSeed = false;       // 'from:' release angle for assembly seeding
    double seedAngle = 0.0;     // radians
    int declLine = 0;
};

struct PinJoint {
    int bodyA, anchA, bodyB, anchB;
    int line = 0;
};

// Prismatic joint: the guide anchor of a moving body rides a rail fixed to
// ground. M2 restricts rails to ground (the rail pose is then constant).
struct SlideJoint {
    int guideBody, guideAnchor;
    Vec2 railPos, railDir;      // world constants (rail is on ground)
    int line = 0;
};

// A gear is a pitch-circle feature of its host body: radius r centered at
// the host anchor, rotating with the host. Ground hosts are allowed in M3
// (fixed rings / stationary gears).
struct Gear {
    std::string name;
    int body = -1;
    int hostAnchor = 0;         // anchor index on the host body
    double radius = 0.0;        // pitch radius
    double module = 0.0;        // 0 when the gear was specified via radius
    int teeth = 0;              // 0 when the gear was specified via radius
    double massOverride = -1.0; // >= 0: explicit total gear mass (kg)
    int line = 0;
};

// Gear mesh. Roll-without-slip row (external, s = +1 / internal, s = -1):
//   rA*(thetaA - thetaA0) + s*rB*(thetaB - thetaB0)
//       - (rA + s*rB)*(psi - psi0)                  = 0
// where psi is the angle of the center line cA -> cB. For fixed centers the
// psi term is constant and the row reduces to the M2 form; for epicyclic
// trains it is what couples the carrier's motion into the rolling relation.
// psi is tracked with branch continuity (it may wind without bound).
// `tangent` additionally enforces |cA - cB|^2 = (rA + s*rB)^2 live, which
// lets a planet orbit a sun with no carrier at all.
// The phase constant is captured at assembly time. External vs internal is
// detected from the assembled geometry (center distance vs pitch radii).
struct MeshJoint {
    int gearA = -1, gearB = -1;
    bool tangent = false;
    mutable int sgn = 0;              // 0 undetected, +1 external, -1 internal
    mutable double rollConst = 0.0;   // row value at assembly (with psi)
    mutable double psiTrack = 0.0;    // continuity-tracked center-line angle
    mutable bool psiInit = false;
    int line = 0;
};

// M6 — a closed curve riding a host body (see profile.hpp). The curve's
// origin is the host anchor; its axes are the host body's.
struct Profile {
    std::string name;
    int body = -1;
    int hostAnchor = 0;
    std::shared_ptr<const ProfileCurve> curve;
    bool fromCam = false;        // M6b: synthesized by a `cam` statement
    double designRoller = 0.0;   // M6b: the roller the cam was cut for
    int line = 0;
};

// M6b — what the cam synthesizer decided, kept for reporting and tests.
struct CamInfo {
    std::string name;
    int profile = -1;
    CamSpec spec;
    double maxLift = 0.0;
    double maxPressure = 0.0, maxPressureAt = 0.0;     // rad
    double minConvexRadius = 0.0, minConvexAt = 0.0;   // m, rad
    int line = 0;
};

// M6 — a point of `body` (an anchor, optionally with a roller of the given
// radius centered on it) rides `profile`. One row: signed gap - roller = 0.
// M6a is bilateral (force closure assumed) and kinematic-only.
struct TouchJoint {
    int body = -1, anchor = 0;
    int profile = -1;
    double roller = 0.0;
    int line = 0;
};

// Where a touch currently sits: the signed gap (positive outside the curve,
// before the roller is subtracted), the foot of the perpendicular, and the
// outward unit normal, in world coordinates.
struct ContactState {
    double gap = 0.0;
    double s = 0.0;          // foot parameter on the curve
    Vec2 footLocal;          // foot in the profile's own (anchor-centered) frame
    Vec2 foot;               // world
    Vec2 normal;             // world, unit, outward from the profile
};

struct Drive {
    int body;
    double omegaRad = 0.0;   // prescribed angular velocity (rad/s)
    mutable double theta0Rad = 0.0;  // prescribed angle (rad); M5: holds capture
                                     // their target on state entry
    mutable bool enabled = true;     // M5: mode holds enable/disable their row
    int line = 0;
};

// Torque drive: pure generalized force about the body's frame origin.
// This is "energy injection" — no coordinate is prescribed; what the
// mechanism does is solved from masses + constraints + forces.
struct TorqueDrive {
    int body;
    double torque;           // N*m
    int line = 0;
};

// Spring between two anchors. rest < 0 means "capture the assembled length"
// (filled in at assembly time, like the mesh roll phase).
struct SpringJoint {
    int bodyA, anchA, bodyB, anchB;
    double k = 0.0;
    mutable double rest = -1.0;
    double damp = 0.0;
    std::string name;
    int line = 0;
};

// Rigid-body mass properties about the center of mass (body frame).
struct MassProps {
    double mass = 0.0;       // kg (0 only in kinematic mode)
    Vec2 com{0.0, 0.0};      // center of mass in body frame
    double inertia = 0.0;    // kg*m^2 about the COM
};

// M4 — a clock hand: render-only needle mounted on a host anchor. No
// constraint rows, no mass; it rotates with its host and points along the
// host's local +x axis.
struct Hand {
    std::string name;
    int body = -1;
    int mountAnchor = 0;
    double length = 0.3;      // metres, along host +x
    int line = 0;
};

// M4 — a static dial: a tick ring in the world frame. Center/radius are
// captured at assembly time (like mesh roll phase and spring rest lengths):
// without 'at' the dial auto-centers on the average hand mount; without
// 'radius' it sizes to 1.2x the longest hand.
struct Dial {
    bool hasCenter = false;         // explicit 'at'
    mutable Vec2 center{0.0, 0.0}; // captured at assembly when auto-centered
    bool hasRadius = false;
    mutable double radius = -1.0;   // < 0: capture at assembly from hands
    int major = 12;
    int minor = 60;
    int line = 0;
};

// M4 — what a 'reduce' synthesized, for `mech check` to report.
struct ReduceInfo {
    std::string bodyA, bodyB;
    std::vector<std::pair<int, int>> stages;  // (wheelTeeth, pinionTeeth)
    int idlers = 0;                           // idler shafts inserted
    int idlerTeeth = 0;
    double module = 0.0;
    std::vector<std::string> placed;          // auto-placed shaft names
    bool opposite = false;                    // B turns against A
    int line = 0;
};

// M5 — a weight drive: an energy source with a budget. A weight of mass m
// on a cord wound on a barrel of radius r applies torque m*g*r about the
// host's frame origin, in the cord's unwind sense ('cw' -> -theta, the
// direction clock hands sweep; default 'ccw' -> +theta). 'drop' bounds the
// fall: once r*dtheta of cord has been paid out (a ratchet — only travel in
// the unwind sense pays), the torque ceases and the mechanism runs down.
// Torque drives without a budget are the kernel form; this is the physical
// statement of the same force.
struct WeightDrive {
    int body = -1;
    double mass = 0.0;             // kg
    double barrelR = 0.0;          // m
    double torque = 0.0;           // N*m = m*g*barrelR (magnitude)
    int sign = 1;                  // +1: pulls along +theta (ccw); -1: 'cw'
    double drop = -1.0;            // m; < 0 = unlimited
    mutable double paid = 0.0;     // m of cord paid out so far
    mutable bool exhausted = false;
    int line = 0;
};

// M5 — a hybrid element: a state machine over constraint sets. States may
// 'hold' bodies: a hold is a rate-0 drive row whose target is captured each
// time the state is entered, so a transition switches the active constraint
// set. Transitions fire when a guard crosses its threshold from below
// (rising edge) during a step — 'theta' and 'omega' watch the body's angle
// and rate; 'advance' watches the angle travelled since state entry (this
// is what steps a ratchet or an escape wheel half a tooth). Resets apply at
// the crossing instant: 'impulse' injects (or removes) energy along the
// body's current rotation; 'maintain' computes exactly the energy needed
// for the body to reach the stated half-swing about its natural hang angle.
struct ModeReset {
    int body = -1;
    bool maintain = false;   // false: fixed energy; true: swing target
    double energy = 0.0;     // J  (impulse form)
    double swingRad = 0.0;   // maintain: target half-swing (rad)
    int line = 0;
};

struct ModeTransition {
    int from = -1, to = -1;      // state indices
    int body = -1;               // guard body
    enum class Field { Theta, Omega, Advance };
    Field field = Field::Theta;
    bool greater = true;         // '>' vs '<'
    double threshold = 0.0;      // rad (theta/advance) or rad/s (omega)
    std::vector<ModeReset> resets;
    int line = 0;
};

struct Mode {
    std::string name;
    struct State {
        std::string name;
        std::vector<int> holdBodies;   // body indices held in this state
    };
    std::vector<State> states;
    std::vector<ModeTransition> transitions;
    std::vector<int> guardBodies;     // bodies watched by guards (advance origins)
    std::vector<int> holdBodies;      // union of held bodies across states
    std::vector<int> holdDrive;       // drive row index per held body
    mutable int current = 0;          // active state
    mutable long fires = 0;           // transitions fired since assembly
    mutable std::vector<double> entryTheta;  // theta at state entry, per guardBody
    int line = 0;

    bool stateHolds(int stateIdx, int bodyIdx) const {
        for (int b : states[(size_t)stateIdx].holdBodies)
            if (b == bodyIdx) return true;
        return false;
    }
};

struct WorldParams {
    bool dynamic = false;    // world block / spring / torque present
    double gravity = 9.81;   // m/s^2 along -y
    double step = 0.002;     // integration timestep (s)
};

struct DofReport {
    int nBodies = 0;
    int nPins = 0;
    int nSlides = 0;
    int nMeshes = 0;         // roll rows
    int nTangentMeshes = 0;  // meshes that also carry a tangency row
    int nGears = 0;
    int nDrives = 0;         // rate drives (constraint rows, enabled only)
    int nTorques = 0;
    int nSprings = 0;
    int nWeights = 0;        // M5: weight drives (forces with a budget)
    int nModes = 0;          // M5: hybrid elements
    int nHolds = 0;          // M5: hold rows enabled in the initial states
    int nProfiles = 0;       // M6: contact curves
    int nTouches = 0;        // M6: point-on-curve contact rows
    int freeDof = 0;      // Grubler-Kutzbach mobility before drives
    int residualDof = 0;  // mobility left over after rate drives
};

class Model {
public:
    std::string name;
    WorldParams world;
    std::vector<Body> bodies;         // bodies[0] is ground
    std::vector<PinJoint> pins;
    std::vector<SlideJoint> slides;
    std::vector<Gear> gears;
    std::vector<MeshJoint> meshes;
    std::vector<Drive> drives;        // rate drives (constraints)
    std::vector<TorqueDrive> torques; // force drives
    std::vector<SpringJoint> springs;
    std::vector<MassProps> mass;      // parallel to bodies; ground unused
    std::vector<Hand> hands;          // M4: render-only needles
    std::vector<Dial> dials;          // M4: static tick rings
    std::vector<ReduceInfo> reduceInfos;  // M4: synthesis bookkeeping
    std::vector<WeightDrive> weights; // M5: bounded energy sources
    std::vector<Mode> modes;          // M5: hybrid state machines
    std::vector<Profile> profiles;    // M6: contact curves
    std::vector<TouchJoint> touches;  // M6: point-on-curve contacts
    std::vector<CamInfo> cams;        // M6b: synthesized plate cams

    int nq() const { return 3 * (int)(bodies.size() - 1); }

    int bodyIndex(const std::string& name) const;
    int anchorIndex(int bodyIdx, const std::string& anchorName) const;
    int gearIndex(const std::string& name) const;
    int profileIndex(const std::string& name) const;

    // World position of an anchor given solver unknowns q.
    Vec2 worldAnchor(int bodyIdx, int anchorIdx, const std::vector<double>& q) const;

    // World position of a gear's pitch-circle center.
    Vec2 gearCenter(int gearIdx, const std::vector<double>& q) const;

    // M6: contact geometry of a touch at configuration q.
    ContactState touchState(const TouchJoint& t, const std::vector<double>& q) const;

    // Static mobility analysis (planar Grubler-Kutzbach):
    //   F = 3(n-1) - 2*(pins + slides) - meshes - tangentMeshes - touches
    DofReport dofReport() const;

    // Greedy graph placement of an initial configuration; detects mesh types
    // (external/internal), captures mesh roll phase and spring rest lengths,
    // and resolves auto-centered/auto-sized dials. Remaining residuals are
    // Newton's job. Returns warnings.
    std::vector<std::string> assemble(std::vector<double>& q) const;

    // Geometric validation of the assembled configuration (mesh tangency).
    // Returns error messages (empty = valid).
    std::vector<std::string> validateAssembly(const std::vector<double>& q) const;

    bool bodyDriven(int bodyIdx) const;

private:
    void placeBody(std::vector<double>& q, int bodyIdx, int viaAnchor,
                   Vec2 target, double theta) const;
    double driveTheta0(int bodyIdx) const;
    double seedAngleFor(int bodyIdx) const;
};

// Build a runtime model from a declaration. Throws SemanticError.
Model buildModel(const MechanismDecl& decl);

} // namespace mech
