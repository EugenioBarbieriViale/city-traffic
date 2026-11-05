#include "City.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Vec2 dirVector(Dir d) {
    static constexpr Vec2 kVectors[DirCount] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    return kVectors[d];
}

Dir opposite(Dir d) { return static_cast<Dir>((d + 2) % DirCount); }

Axis axisOf(Dir d) { return d == North || d == South ? Axis::NorthSouth : Axis::EastWest; }

Vec2 rightOf(Dir d) {
    Vec2 v = dirVector(d);
    return {-v.y, v.x};
}

Node neighbor(Node n, Dir d) {
    Vec2 v = dirVector(d);
    return {n.col + static_cast<int>(v.x), n.row + static_cast<int>(v.y)};
}

Dir headingBetween(Node from, Node to) {
    for (int i = 0; i < DirCount; i++) {
        Dir d = static_cast<Dir>(i);
        if (neighbor(from, d) == to) return d;
    }
    throw std::invalid_argument("headingBetween: nodes are not adjacent");
}

namespace {

// Floored, so that it also holds for times before the schedule's greenStart.
double positiveMod(double value, double modulus) {
    return value - modulus * std::floor(value / modulus);
}

}  // namespace

bool GreenSchedule::isGreen(double t) const {
    return positiveMod(t - greenStart, cycle) < greenDuration;
}

double GreenSchedule::timeToNextChange(double t) const {
    double phase = positiveMod(t - greenStart, cycle);
    return phase < greenDuration ? greenDuration - phase : cycle - phase;
}

IntervalSet GreenSchedule::windows(double from, double to) const {
    std::vector<Interval> result;
    double k = std::floor((from - greenStart) / cycle);
    for (double start = greenStart + k * cycle; start <= to; start += cycle) {
        double end = start + greenDuration;
        if (end >= from) result.push_back({start, end});
    }
    return IntervalSet(std::move(result));
}

double GreenSchedule::nextOpening(double t, double margin) const {
    const IntervalSet greens = windows(t, t + 2.0 * cycle);
    for (const Interval &green : greens.intervals()) {
        if (t <= green.hi - margin) return std::max(t, green.lo + margin);
    }
    throw std::logic_error("GreenSchedule::nextOpening: no green window");
}

GreenSchedule TrafficLight::schedule(Axis axis) const {
    GreenSchedule s;
    s.cycle = 2.0 * phaseSeconds;
    s.greenDuration = phaseSeconds;
    // North-south is green for the first half of each advanced cycle.
    s.greenStart = -phaseOffset + (axis == Axis::NorthSouth ? 0.0 : phaseSeconds);
    return s;
}

City::City(const CityConfig &config) : config_(config) {
    for (int col = 0; col < config_.cols; col++) {
        for (int row = 0; row < config_.rows; row++) {
            Node n{col, row};
            if (!isCrossing(n)) continue;
            double offset = (col + row) * config_.lightStaggerSeconds;
            lights_.push_back({n, offset, config_.lightPhaseSeconds});
        }
    }
}

int City::streetIndex(Node from, Dir heading) const {
    return (from.col * config_.rows + from.row) * DirCount + heading;
}

std::optional<int> City::streetAt(Vec2 position, Vec2 velocity) const {
    constexpr double kOnLane = 0.5;  // Metres.
    const double block = config_.blockLength;
    for (int i = 0; i < DirCount; i++) {
        const Dir heading = static_cast<Dir>(i);
        const Vec2 forward = dirVector(heading);
        if (dot(velocity, velocity) > 0.0 && dot(velocity, forward) <= 0.0) continue;

        const Vec2 centered = position - rightOf(heading) * laneOffset();
        const bool vertical = axisOf(heading) == Axis::NorthSouth;
        const double across = vertical ? centered.x : centered.y;
        const double line = std::round(across / block);
        if (std::abs(across - line * block) > kOnLane) continue;

        // The node behind the point, whichever way the heading runs.
        const double along = (vertical ? centered.y : centered.x) / block;
        const double back = dot(forward, {1.0, 1.0}) > 0.0 ? std::floor(along) : std::ceil(along);
        const Node from = vertical ? Node{static_cast<int>(line), static_cast<int>(back)}
                                   : Node{static_cast<int>(back), static_cast<int>(line)};
        if (nodeExists(from) && nodeExists(neighbor(from, heading))) {
            return streetIndex(from, heading);
        }
    }
    return std::nullopt;
}

bool City::nodeExists(Node n) const {
    return n.col >= 0 && n.col < config_.cols && n.row >= 0 && n.row < config_.rows;
}

bool City::isCrossing(Node n) const {
    int roads = 0;
    for (int d = 0; d < DirCount; d++) {
        if (nodeExists(neighbor(n, static_cast<Dir>(d)))) roads++;
    }
    return roads >= 3;
}

Vec2 City::nodeCenter(Node n) const {
    return {n.col * config_.blockLength, n.row * config_.blockLength};
}

Vec2 City::turnPoint(Node n, Dir in, Dir out) const {
    Vec2 point = nodeCenter(n) + rightOf(in) * laneOffset();
    if (in != out) point = point + rightOf(out) * laneOffset();
    return point;
}

std::optional<std::size_t> City::lightIndexAt(Node n) const {
    for (std::size_t i = 0; i < lights_.size(); i++) {
        if (lights_[i].node == n) return i;
    }
    return std::nullopt;
}

void City::setLightOffset(Node n, double offset) {
    auto index = lightIndexAt(n);
    if (!index) throw std::invalid_argument("setLightOffset: node has no light");
    lights_[*index].phaseOffset = offset;
}
