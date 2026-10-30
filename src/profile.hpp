// mech — profile.hpp: closed planar curves for contact (M6a)
//
// A profile is a closed, counter-clockwise C2 curve c(s), s in [0, period),
// expressed in a frame centered on its host anchor (axes parallel to the host
// body's). Three kinds:
//   circle   exact: c(s) = R (cos s, sin s)
//   ellipse  exact: c(s) = (cx + a cos s, b sin s), centered on the anchor or
//            with the anchor at the focus (cx = sqrt(a^2 - b^2))
//   polar    periodic cubic spline r(phi) through (angle, radius) knots,
//            c(phi) = r(phi) (cos phi, sin phi)      (star-shaped about the anchor)
//   points   periodic cubic splines x(s), y(s) through the given points,
//            s = cumulative chord length             (any simple closed curve)
//
// The one query contact needs is `nearest`: the foot of the perpendicular
// from a point to the curve, with the signed distance (positive outside).
// At the foot the distance is stationary in s, which is what lets the contact
// row's Jacobian ignore ds (see docs/design/contact.md).
#pragma once
#include "geom.hpp"
#include <vector>

namespace mech {

class ProfileCurve {
public:
    enum class Kind { Circle, Polar, Points, Ellipse };

    // Preconditions are checked by the model builder (positive radii,
    // strictly increasing angles in [0, 2*pi), >= 3 distinct points).
    static ProfileCurve circle(double radius);
    // Semi-axes a (along x) and b; `focus` puts the anchor at the right-hand
    // focus (needs a >= b), i.e. the ellipse is a gear pivoted at a focus.
    static ProfileCurve ellipse(double a, double b, bool focus);
    static ProfileCurve polar(const std::vector<double>& angles,
                              const std::vector<double>& radii);
    // Orientation is normalized to counter-clockwise (reversed if needed).
    static ProfileCurve points(const std::vector<Vec2>& pts);

    Kind kind() const { return kind_; }
    double period() const { return period_; }

    // Position and first/second derivative at parameter s (wrapped).
    void eval(double s, Vec2& p, Vec2& d1, Vec2& d2) const;

    struct Foot {
        double s = 0.0;          // wrapped to [0, period)
        Vec2 point;              // c(s)
        Vec2 tangent;            // c'(s) (not normalized)
        Vec2 normal;             // unit, outward (right of the tangent)
        double signedDist = 0.0; // normal . (u - point): > 0 outside the curve
        double curvature = 0.0;  // signed, > 0 where the curve is convex (turns toward its inside)
    };
    // Closest point on the curve to u. Global coarse scan, then a bracketed
    // Newton refinement; deterministic and free of hidden state.
    Foot nearest(Vec2 u) const;

    // M6d: star-shaped about the anchor (every ray meets the curve once), so
    // the curve is a polar function r(phi). Circle, polar and ellipse are;
    // a point list in general is not.
    bool starShaped() const { return kind_ != Kind::Points; }
    // r, dr/dphi and d2r/dphi2 at polar angle phi (radians, any branch).
    void polarEval(double phi, double& r, double& dr, double& d2r) const;

    // Sampled outline (n points, counter-clockwise) for drawing.
    std::vector<Vec2> outline(int n) const;
    // Largest distance from the anchor to the curve (camera fit).
    double maxRadius() const { return maxRadius_; }
    double minRadius() const { return minRadius_; }
    // Number of defining knots (0 for a circle).
    size_t knots() const { return a_.t.size(); }
    // Circle radius (0 for the other kinds).
    double circleRadius() const { return radius_; }
    // Ellipse parameters (0 for the other kinds).
    double ellipseA() const { return ea_; }
    double ellipseB() const { return eb_; }
    bool ellipseFocus() const { return efocus_; }

private:
    // Periodic cubic spline of one scalar over shared knots.
    struct Spline {
        std::vector<double> t, y, M;   // knots, values, second derivatives
        double period = 0.0;
        void build(const std::vector<double>& knots, const std::vector<double>& vals,
                   double per);
        void eval(double s, double& v, double& d1, double& d2) const;
    };

    void finish();   // sample table + extent

    Kind kind_ = Kind::Circle;
    double radius_ = 0.0;
    double ea_ = 0.0, eb_ = 0.0, ecx_ = 0.0;   // ellipse: semi-axes, centre offset
    bool efocus_ = false;
    double period_ = 0.0;
    Spline a_, b_;   // polar: a_ = r(phi); points: a_ = x(s), b_ = y(s)
    std::vector<double> sampleS_;
    std::vector<Vec2> sampleP_;
    double maxRadius_ = 0.0;
    double minRadius_ = 0.0;
};

} // namespace mech
