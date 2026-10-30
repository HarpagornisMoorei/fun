// mech — cam.hpp: displacement-diagram synthesis of plate cams (M6b)
//
// The author states the follower's motion — rise, dwell, return, each with a
// motion law — and the compiler cuts the cam: it solves the surface that
// produces that motion, the way `reduce` solves tooth counts. The result is an
// ordinary closed `points` profile, so the solver never learns cams exist.
//
// Geometry (translating roller follower on a rail through the axle, or a
// knife-edge tip). With base radius Rb and roller radius rho, the *prime
// circle* has radius r0 = Rb + rho and the roller centre — the pitch point —
// sits at radius r(theta) = r0 + s(theta), where s is the displacement diagram
// and theta the cam's rotation. In the cam's own frame the follower rail lies
// along polar angle phi = start - turn*theta (the cam turns by +theta, so the
// surface passes under the follower in the opposite sense). The pitch curve is
// P(theta) = r(theta) (cos phi, sin phi); the cam surface is that curve offset
// inward by rho along its normal.
//
// Two diagnostics come out of the same numbers:
//   pressure angle   tan(alpha) = s'(theta) / r(theta)      (rail through the axle)
//   undercut         the pitch curve's radius of curvature, where convex, must
//                    exceed rho, or the inward offset folds over itself
#pragma once
#include "geom.hpp"
#include <string>
#include <vector>

namespace mech {

enum class MotionLaw { Cycloidal, Harmonic, ModTrap };

// Normalized motion on x in [0, 1]: s goes 0 -> 1, with s' = ds/dx and
// s'' = d2s/dx2. All three laws leave both ends at rest (s' = 0).
struct LawValue { double s, v, a; };
LawValue evalLaw(MotionLaw law, double x);
const char* lawName(MotionLaw law);

struct CamSegment {
    enum class Kind { Rise, Fall, Return, Dwell };
    Kind kind = Kind::Dwell;
    double lift = 0.0;          // m (Rise / Fall)
    double angle = 0.0;         // rad of cam rotation
    bool restOfTurn = false;    // Dwell only: fill whatever is left of the turn
    MotionLaw law = MotionLaw::Cycloidal;
};

struct CamSpec {
    double base = 0.0;          // base circle radius of the cam surface (m)
    double roller = 0.0;        // follower roller radius (0 = knife-edge)
    double startRad = kPi / 2.0; // polar angle of the rail at theta = 0 (+y)
    int turn = +1;              // +1: cam turns counter-clockwise (+theta)
    std::vector<CamSegment> segments;
    int samples = 720;          // surface points handed to the profile
};

struct CamError {
    int segment = -1;           // index of the offending segment, or -1
    std::string msg;
};

// The displacement diagram s(theta): piecewise, periodic, theta in radians.
class CamDiagram {
public:
    // Resolves segment lifts, validates closure; false (with err) if the
    // diagram is not a closed, non-negative, full-turn motion.
    bool build(const CamSpec& spec, CamError& err);

    // s, ds/dtheta, d2s/dtheta2 at theta (wrapped to one turn).
    void eval(double theta, double& s, double& ds, double& dds) const;

    double maxLift() const { return maxLift_; }

private:
    struct Seg {
        double th0 = 0.0, beta = 0.0, s0 = 0.0, lift = 0.0;
        MotionLaw law = MotionLaw::Cycloidal;
        bool dwell = true;
    };
    std::vector<Seg> segs_;
    double maxLift_ = 0.0;
};

struct CamDesign {
    std::vector<Vec2> surface;          // cam surface, anchor-centered
    double maxLift = 0.0;               // m
    double maxPressure = 0.0;           // rad
    double maxPressureAt = 0.0;         // theta (rad)
    double minConvexRadius = 1e300;     // m; pitch curve, convex parts only
    double minConvexAt = 0.0;           // theta (rad)
    bool undercut = false;              // roller > min convex radius
};

// Cut the cam. `dia` must have been built from `spec`.
CamDesign designCam(const CamSpec& spec, const CamDiagram& dia);

} // namespace mech
