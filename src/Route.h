#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "City.h"
#include "SpeedPlanner.h"

// A light the route crosses, at the arc-length position of its stop line.
struct RouteLight {
    std::size_t lightIndex = 0;
    Axis axis = Axis::EastWest;  // The axis of travel through the crossing.
    double stopLine = 0.0;
};

struct RoutePose {
    Vec2 position;
    Dir heading = East;
};

// A drivable path through the city, parameterized by arc length along the
// right-hand lane. Positions are metres from the route's start.
class Route {
public:
    // Throws std::invalid_argument unless `nodes` holds at least two existing
    // nodes, each adjacent to the last, with no U-turns. The route stops `endTrim`
    // metres short of its last node, so it can end partway along its last street;
    // that distance must lie within the street.
    Route(const City &city, std::vector<Node> nodes, double endTrim = 0.0);

    // Appends a node, keeping every position already on the route valid up to
    // the old final node. Throws like the constructor.
    void extend(Node next);

    const City &city() const { return *city_; }
    const std::vector<Node> &nodes() const { return nodes_; }
    double length() const { return cumulative_.back() - endTrim_; }
    const std::vector<RouteLight> &lights() const { return lights_; }

    // The heading of the last street, for choosing how to extend the route.
    Dir finalHeading() const;

    RoutePose poseAt(double position) const;

    // The route's streets, in order. Street k runs from nodes()[k] to nodes()[k + 1].
    std::size_t streetCount() const { return headings_.size(); }
    // The street holding `position`. A street ends at the turn inside its last node.
    std::size_t streetAt(double position) const;
    int streetIndex(std::size_t street) const;
    // Where street k ends along the route.
    double streetEnd(std::size_t street) const { return cumulative_[street + 1]; }
    Dir streetHeading(std::size_t street) const { return headings_[street]; }
    // Whether the route turns left at the end of street k.
    bool turnsLeftAfter(std::size_t street) const;
    // Where on the route, at or after `from`, a point moving with `velocity` lies:
    // in the route's own lane and going its way. A standing point can go either way.
    std::optional<double> locate(Vec2 point, Vec2 velocity, double from) const;
    // `position` as metres past the node street k starts from, the same for every
    // car on that street whatever route it took. Beyond the street's ends this
    // keeps counting along the route.
    double streetCoordinate(std::size_t street, double position) const;

    // The route between two positions: its ends, and every corner in between.
    std::vector<Vec2> polyline(double from, double to) const;

    // Index of the first light whose stop line is at or beyond `position`.
    std::size_t nextLightIndex(double position) const;

    // What the car may know: every light at or ahead of `position` within
    // `horizon` metres along the route, nearest first, with its schedule.
    std::vector<UpcomingLight> lightsAhead(double position, double horizon) const;

private:
    void rebuildWaypoints();

    const City *city_;
    std::vector<Node> nodes_;
    double endTrim_;
    std::vector<Vec2> waypoints_;
    std::vector<Dir> headings_;  // headings_[i] runs from nodes_[i] to nodes_[i + 1].
    std::vector<double> cumulative_;  // Arc length at each waypoint.
    std::vector<RouteLight> lights_;
};
