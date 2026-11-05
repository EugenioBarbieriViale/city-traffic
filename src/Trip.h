#pragma once

#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "City.h"
#include "SpeedPlanner.h"

// A point on a street: `offset` metres from node `from`, along `heading`.
struct StreetPoint {
    Node from;
    Dir heading = East;
    double offset = 0.0;
};

// Where a car sets off (the start's heading is the way it faces) and where it
// has to get to, which may be reached from either direction. Always on two
// different streets.
struct Trip {
    StreetPoint start;
    StreetPoint end;
};

// How cars drive on a trip. Faster than PlannerConfig's defaults.
PlannerConfig tripPlannerConfig();

// A grid with a random number of streets each way, with short light phases.
CityConfig randomCityConfig(std::mt19937 &rng);

// Both points stay far enough from the crossings that a car driving by
// tripPlannerConfig() can stop at the end without its braking interfering with
// the last light.
// `hotspotShare` is the chance that the end is on a street touching the node
// nearest the city's center.
Trip randomTrip(const City &city, std::mt19937 &rng, double hotspotShare = 0.0);

// A new trip for a car already at `from`, ending on a different street.
Trip nextTrip(const City &city, const StreetPoint &from, std::mt19937 &rng,
              double hotspotShare = 0.0);

// What it takes to build the Route and place the car on it.
struct TripPath {
    std::vector<Node> nodes;
    double startPosition = 0.0;  // Where the car starts along the route.
    double endTrim = 0.0;        // Metres the route stops short of its last node.
    double estimatedTime = 0.0;  // Seconds from the start, by the search's model.
};

// Extra seconds a car expects a whole street to cost it on top of the free run.
using StreetDelay = std::function<double(Node from, Dir heading)>;

// What the search knows beyond the map and the lights. The defaults are a parked
// car at time 0 with an empty city.
struct PathOptions {
    double startTime = 0.0;
    double startSpeed = 0.0;
    // The cars queued ahead keep the car from its first stop line until this time.
    double notBefore = -std::numeric_limits<double>::infinity();
    StreetDelay streetDelay;  // Empty: no street is expected to be slow.
};

// The earliest arrival, found with a time-dependent Dijkstra search over (node,
// heading) pairs, since the way a car enters a node decides its light and the
// U-turn it may not make. Lights are deterministic and neither waiting for one nor
// a street's delay ever makes a car arrive earlier than a later car would, so the
// search stays exact for this model: vMax between stop lines plus each street's
// delay, plus the ramp up at the start and the braking at the end, waiting whenever
// a stop line is reached outside its green less the planner's safety margin.
//
// Throws std::invalid_argument if both points are on the same street.
TripPath fastestPath(const City &city, const PlannerConfig &config, const Trip &trip,
                     const PathOptions &options = {});
