#include "Trip.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>

#include "Kinematics.h"
#include "Simulation.h"

namespace {

constexpr int kMinCols = 3;
constexpr int kMaxCols = 9;
constexpr int kMinRows = 3;
constexpr int kMaxRows = 7;

constexpr double kLightPhaseSeconds = 5.0;
constexpr double kMaxSpeed = 20.0;  // m/s, 72 km/h.

// Beyond the least distance trip endpoints need from a street's ends.
constexpr double kClearanceSlack = 2.0;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

int randomInt(std::mt19937 &rng, int low, int high) {
    return std::uniform_int_distribution<int>(low, high)(rng);
}

// A node reached with a heading, at the time the car crosses its stop line.
struct Arrival {
    double time = 0.0;
    int state = 0;

    friend bool operator>(const Arrival &a, const Arrival &b) { return a.time > b.time; }
};

// Metres from `node`'s center to the end of the trip, if the street leaving `node`
// along `heading` is the one the trip ends on.
double distanceToEnd(const City &city, const StreetPoint &end, Node node, Dir heading) {
    if (node == end.from && heading == end.heading) return end.offset;
    if (node == neighbor(end.from, end.heading) && heading == opposite(end.heading)) {
        return city.config().blockLength - end.offset;
    }
    return -1.0;
}

}  // namespace

PlannerConfig tripPlannerConfig() {
    PlannerConfig config;
    config.vMax = kMaxSpeed;
    return config;
}

CityConfig randomCityConfig(std::mt19937 &rng) {
    CityConfig config;
    config.lightPhaseSeconds = kLightPhaseSeconds;
    config.cols = randomInt(rng, kMinCols, kMaxCols);
    config.rows = randomInt(rng, kMinRows, kMaxRows);
    return config;
}

namespace {

// Every street once, whichever way it is driven.
std::vector<StreetPoint> allStreets(const City &city) {
    const CityConfig &config = city.config();
    std::vector<StreetPoint> streets;
    for (int col = 0; col < config.cols; col++) {
        for (int row = 0; row < config.rows; row++) {
            for (Dir d : {East, South}) {
                if (city.nodeExists(neighbor({col, row}, d))) streets.push_back({{col, row}, d, 0.0});
            }
        }
    }
    return streets;
}

bool sameStreet(const StreetPoint &a, const StreetPoint &b) {
    if (a.from == b.from && a.heading == b.heading) return true;
    return neighbor(a.from, a.heading) == b.from && opposite(a.heading) == b.heading;
}

// A car entering the last street crosses its stop line before the node, so
// that distance counts toward stopping from vMax after the last light.
double randomOffset(const City &city, std::mt19937 &rng) {
    const PlannerConfig planner = tripPlannerConfig();
    const double stopping = rampDistance(planner.vMax, 0.0, kArrivalBrakingShare * planner.aMax);
    const double clearance = stopping - city.stopLineDistance() + kClearanceSlack;
    return std::uniform_real_distribution<double>(
        clearance, city.config().blockLength - clearance)(rng);
}

}  // namespace

Trip randomTrip(const City &city, std::mt19937 &rng, double hotspotShare) {
    const std::vector<StreetPoint> streets = allStreets(city);
    const int count = static_cast<int>(streets.size());
    StreetPoint start = streets[randomInt(rng, 0, count - 1)];
    if (randomInt(rng, 0, 1) == 1) {
        start.from = neighbor(start.from, start.heading);
        start.heading = opposite(start.heading);
    }
    start.offset = randomOffset(city, rng);
    return nextTrip(city, start, rng, hotspotShare);
}

Trip nextTrip(const City &city, const StreetPoint &from, std::mt19937 &rng,
              double hotspotShare) {
    std::vector<StreetPoint> streets = allStreets(city);
    if (std::bernoulli_distribution(hotspotShare)(rng)) {
        const Node center{city.config().cols / 2, city.config().rows / 2};
        streets.erase(std::remove_if(streets.begin(), streets.end(),
                                     [&](const StreetPoint &s) {
                                         return s.from != center &&
                                                neighbor(s.from, s.heading) != center;
                                     }),
                      streets.end());
    }
    streets.erase(std::remove_if(streets.begin(), streets.end(),
                                 [&](const StreetPoint &s) { return sameStreet(s, from); }),
                  streets.end());
    StreetPoint end = streets[randomInt(rng, 0, static_cast<int>(streets.size()) - 1)];
    end.offset = randomOffset(city, rng);
    return {from, end};
}

TripPath fastestPath(const City &city, const PlannerConfig &config, const Trip &trip,
                     const PathOptions &options) {
    const StreetPoint &start = trip.start;
    const Node firstNode = neighbor(start.from, start.heading);
    if (distanceToEnd(city, trip.end, start.from, start.heading) >= 0.0 ||
        distanceToEnd(city, trip.end, firstNode, opposite(start.heading)) >= 0.0) {
        throw std::invalid_argument("fastestPath: both points are on the same street");
    }

    const CityConfig &grid = city.config();
    const double stopDistance = city.stopLineDistance();
    const double rampLoss = config.vMax / (2.0 * config.aMax);  // Against moving at vMax.
    const auto streetDelay = [&](Node from, Dir heading) {
        return options.streetDelay ? options.streetDelay(from, heading) : 0.0;
    };

    const auto stateOf = [&](Node n, Dir heading) { return city.streetIndex(n, heading); };
    const auto nodeOf = [&](int state) {
        const int cell = state / DirCount;
        return Node{cell / grid.rows, cell % grid.rows};
    };
    const auto crossingTime = [&](Node n, Dir heading, double t) {
        const auto light = city.lightIndexAt(n);
        if (!light) return t;
        return city.light(*light).schedule(axisOf(heading)).nextOpening(t, config.safetyMargin);
    };

    std::vector<double> best(city.streetIndexCount(), kInfinity);
    std::vector<int> parent(best.size(), -1);
    std::priority_queue<Arrival, std::vector<Arrival>, std::greater<Arrival>> queue;

    const auto visit = [&](Node n, Dir heading, double time, int from) {
        const int state = stateOf(n, heading);
        if (time >= best[state]) return;
        best[state] = time;
        parent[state] = from;
        queue.push({time, state});
    };

    // A short first leg can end with the ramp up unfinished, so what is left of
    // its loss still has to be counted.
    const double firstLine = grid.blockLength - start.offset - stopDistance;
    const double firstLeg = travelTime(firstLine, options.startSpeed, config.vMax, config.aMax);
    const double speedLeft = config.vMax - options.startSpeed;
    const double startLoss = speedLeft * speedLeft / (2.0 * config.aMax * config.vMax);
    const double rampLeft = startLoss - (firstLeg - firstLine / config.vMax);
    const double firstReached = std::max(options.startTime + firstLeg, options.notBefore);
    visit(firstNode, start.heading,
          crossingTime(firstNode, start.heading, firstReached) + rampLeft, -1);

    double bestTime = kInfinity;
    int lastState = -1;
    Node lastNode;
    double lastDistance = 0.0;

    while (!queue.empty()) {
        const Arrival arrival = queue.top();
        queue.pop();
        if (arrival.time >= bestTime) break;
        if (arrival.time > best[arrival.state]) continue;  // Superseded.

        const Node node = nodeOf(arrival.state);
        const Dir in = static_cast<Dir>(arrival.state % DirCount);
        for (int i = 0; i < DirCount; i++) {
            const Dir out = static_cast<Dir>(i);
            const Node next = neighbor(node, out);
            if (out == opposite(in) || !city.nodeExists(next)) continue;

            // Cars keep to the right, so a right turn cuts the corner by two lane
            // offsets and a left turn goes around it by as much.
            const double turnChange = -dot(rightOf(in), dirVector(out)) * 2.0 * city.laneOffset();

            const double toEnd = distanceToEnd(city, trip.end, node, out);
            if (toEnd >= 0.0) {
                const double distance = stopDistance + turnChange + toEnd;
                const double time = arrival.time + distance / config.vMax + rampLoss;
                if (time < bestTime) {
                    bestTime = time;
                    lastState = arrival.state;
                    lastNode = next;
                    lastDistance = toEnd;
                }
                continue;
            }

            const double distance = grid.blockLength + turnChange;
            const double reached = arrival.time + distance / config.vMax + streetDelay(node, out);
            visit(next, out, crossingTime(next, out, reached), arrival.state);
        }
    }
    if (lastState < 0) throw std::logic_error("fastestPath: the end is unreachable");

    TripPath path;
    path.nodes.push_back(lastNode);
    for (int state = lastState; state >= 0; state = parent[state]) {
        path.nodes.push_back(nodeOf(state));
    }
    path.nodes.push_back(start.from);
    std::reverse(path.nodes.begin(), path.nodes.end());
    path.startPosition = start.offset;
    path.endTrim = grid.blockLength - lastDistance;
    path.estimatedTime = bestTime - options.startTime;
    return path;
}
