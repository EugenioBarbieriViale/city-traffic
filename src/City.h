#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "IntervalSet.h"

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

enum Dir { North, East, South, West, DirCount };

enum class Axis { NorthSouth, EastWest };

Vec2 dirVector(Dir d);
Dir opposite(Dir d);
Axis axisOf(Dir d);
// Unit vector pointing to the driver's right (y grows southward).
Vec2 rightOf(Dir d);

struct Node {
    int col = 0;
    int row = 0;

    friend bool operator==(Node a, Node b) { return a.col == b.col && a.row == b.row; }
    friend bool operator!=(Node a, Node b) { return !(a == b); }
};

Node neighbor(Node n, Dir d);
// The heading that leads from `from` to its adjacent node `to`.
Dir headingBetween(Node from, Node to);

// A periodic green signal: green during [greenStart + k * cycle,
// greenStart + k * cycle + greenDuration) for every integer k.
struct GreenSchedule {
    double cycle = 20.0;
    double greenDuration = 10.0;
    double greenStart = 0.0;

    // The signal itself is half-open: the instant green ends is already red.
    bool isGreen(double t) const;
    // Seconds until the state next changes, measured from t.
    double timeToNextChange(double t) const;
    // Every whole green window overlapping [from, to]. Windows are not clipped
    // to the range, because a caller's margins apply to their true endpoints.
    IntervalSet windows(double from, double to) const;
    // The earliest time at or after t inside a green shrunk by `margin` at both ends.
    double nextOpening(double t, double margin) const;
};

// One crossing's signal. The two axes alternate: one is green while the other
// is red, so each direction sees 10 s red then 10 s green, half a cycle apart.
struct TrafficLight {
    Node node;
    double phaseOffset = 0.0;  // Seconds the cycle is advanced by.
    double phaseSeconds = 10.0;  // Duration of both red and green.

    GreenSchedule schedule(Axis axis) const;
    bool isGreen(Axis axis, double t) const { return schedule(axis).isGreen(t); }
};

struct CityConfig {
    int cols = 4;
    int rows = 3;
    double blockLength = 200.0;  // Metres between neighboring nodes.
    double roadWidth = 36.0;
    double lightPhaseSeconds = 10.0;
    double lightStaggerSeconds = 2.0;  // Offset between neighboring crossings.
};

// A grid of straight streets. Corners of the grid are plain bends, so only
// nodes where three or four streets meet are crossings with a light.
class City {
public:
    explicit City(const CityConfig &config = {});

    const CityConfig &config() const { return config_; }
    double laneOffset() const { return config_.roadWidth / 4.0; }  // Cars keep to the right.
    // Distance from a crossing's center to its stop line.
    double stopLineDistance() const { return config_.roadWidth / 2.0 + 4.0; }

    // Every direction of every street has its own index, below streetIndexCount().
    // The two directions are separate lanes, so they count as separate streets.
    int streetIndex(Node from, Dir heading) const;
    int streetIndexCount() const { return config_.cols * config_.rows * DirCount; }
    // The street whose lane holds `position`, read off the map. `velocity` settles
    // which lane a point in a crossing is on. Null off every lane.
    std::optional<int> streetAt(Vec2 position, Vec2 velocity) const;

    bool nodeExists(Node n) const;
    bool isCrossing(Node n) const;
    Vec2 nodeCenter(Node n) const;
    // The point inside a node where the incoming and outgoing lanes meet.
    Vec2 turnPoint(Node n, Dir in, Dir out) const;

    const std::vector<TrafficLight> &lights() const { return lights_; }
    const TrafficLight &light(std::size_t index) const { return lights_.at(index); }
    std::optional<std::size_t> lightIndexAt(Node n) const;
    // Replaces a crossing's offset. Throws std::invalid_argument if n has no light.
    void setLightOffset(Node n, double offset);

private:
    CityConfig config_;
    std::vector<TrafficLight> lights_;
};
