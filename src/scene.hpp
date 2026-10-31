// mech — scene.hpp: what a pose looks like, as a list of drawing items (M15)
//
// One description of the picture, drawn two ways: `mech render` writes the items as SVG, the OpenGL
// viewer draws them with GL. Everything the two outputs share lives here — which gears are rings,
// whether the ground is a bar or a row of marks, where a rail on a moving body runs, how a body with
// several anchors is drawn — so a fix made once is made in both, and the tests that read the SVG back
// against the solver cover the viewer's geometry too.
//
// All coordinates are world coordinates (metres, y up). Sizes that are pixels on screen (line widths,
// the radius of a joint hub) are kept in pixels where they are widths and converted with the pixel size
// where they are lengths, so the picture is the same at any zoom.
#pragma once
#include "model.hpp"
#include <string>
#include <vector>

namespace mech {

struct Rgb { double r, g, b; };

struct Theme {
    Rgb bg, ground, hub, bore, trace, spring, dial, dialMajor, hand, outline;
    Rgb palette[5];
};

Theme lightTheme();   // a white page, for print
Theme darkTheme();    // the viewer's colours

struct SceneItem {
    enum class Kind { GroupOpen, GroupClose, Line, Circle, Polygon, Polyline };
    Kind kind = Kind::Line;
    std::string cls, attrs;        // SVG class and extra attributes (` data-body="x"`); a GL drawer ignores them
    std::vector<Vec2> pts;         // Line: two points; Circle: the centre; Polygon/Polyline: the vertices
    double r = 0.0;                // Circle: radius, in world units
    bool hasStroke = false, hasFill = false;
    Rgb stroke{0, 0, 0}, fill{0, 0, 0};
    double strokePx = 0.0;         // stroke width, in screen pixels
    double strokeOpacity = 1.0, fillOpacity = 1.0;
    bool round = false;            // Line: round caps
};

// How far a slide's guide travels along its rail over everything shown, measured from the rail anchor,
// so the rail can be drawn where the follower goes.
struct RailSpan {
    Vec2 pos, dir;
    double t0 = 1e30, t1 = -1e30;
};

// M16: where along each rack the pinion has been, measured from the rack's anchor along its line, so the
// rack is drawn as long as the motion needs. One span per RackMesh.
std::vector<RailSpan> newRackSpans(const Model& m);
void growRackSpans(const Model& m, const std::vector<double>& q, std::vector<RailSpan>& spans);

class SceneBuilder {
public:
    // pxWorld: world units per screen pixel. extent: the size of the view in metres (rails and marks are
    // scaled by it). rails: spans of the slides whose rail is on the ground; movingRails: one span per slide
    // (used for the slides whose rail rides a moving body).
    SceneBuilder(const Model& m, const Theme& th, double pxWorld, double extent, std::vector<RailSpan> rails,
                 std::vector<RailSpan> movingRails, std::vector<RailSpan> rackSpans = {})
        : m_(m), th_(th), px_(pxWorld), extent_(extent), rails_(std::move(rails)), moving_(std::move(movingRails)),
          racks_(std::move(rackSpans)) {}

    // what does not move: dials and the ground
    void fixed(std::vector<SceneItem>& out) const;
    // everything that depends on the pose q
    void pose(const std::vector<double>& q, std::vector<SceneItem>& out) const;

private:
    const Model& m_;
    Theme th_;
    double px_, extent_;
    std::vector<RailSpan> rails_;
    std::vector<RailSpan> moving_;
    std::vector<RailSpan> racks_;
};

} // namespace mech
