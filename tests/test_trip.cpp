#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

#include "City.h"
#include "Route.h"
#include "Simulation.h"
#include "SpeedPlanner.h"
#include "TestSupport.h"
#include "Trip.h"

namespace {

// A car that drives its trip the way the app does: GLOSA, then stopping at the end.
SimulationResult driveTrip(const Route &route, const TripPath &path, const PlannerConfig &planner,
                           double dt) {
    GlosaController controller(planner, 0.2, std::make_shared<EarliestArrivalCost>());
    SimulationOptions options;
    options.dt = dt;
    return simulate(route, CarState{path.startPosition, 0.0}, options,
                    [&](const Route &r, const CarState &car, double time) {
                        return std::min(controller.targetSpeed(r, car, time),
                                        arrivalSpeedLimit(r, car, planner.aMax));
                    });
}

}  // namespace

TEST(random_cities_are_grids_of_varied_size) {
    std::mt19937 rng(1);
    int minCols = 100, maxCols = 0, minRows = 100, maxRows = 0;
    for (int i = 0; i < 500; i++) {
        CityConfig config = randomCityConfig(rng);
        CHECK(config.lightPhaseSeconds == 5.0);
        minCols = std::min(minCols, config.cols);
        maxCols = std::max(maxCols, config.cols);
        minRows = std::min(minRows, config.rows);
        maxRows = std::max(maxRows, config.rows);
    }
    CHECK(minCols == 3);
    CHECK(maxCols == 9);
    CHECK(minRows == 3);
    CHECK(maxRows == 7);
}

TEST(trip_cars_are_faster_than_the_default) {
    CHECK(tripPlannerConfig().vMax > PlannerConfig{}.vMax);
    tripPlannerConfig().validate();
}

TEST(random_trips_use_two_different_streets_away_from_the_crossings) {
    std::mt19937 rng(2);
    const PlannerConfig planner = tripPlannerConfig();
    const double stopping =
        planner.vMax * planner.vMax / (2.0 * kArrivalBrakingShare * planner.aMax);
    for (int i = 0; i < 2000; i++) {
        City city(randomCityConfig(rng));
        Trip trip = randomTrip(city, rng);
        CHECK(city.nodeExists(trip.start.from));
        CHECK(city.nodeExists(neighbor(trip.start.from, trip.start.heading)));
        CHECK(city.nodeExists(trip.end.from));
        CHECK(city.nodeExists(neighbor(trip.end.from, trip.end.heading)));
        for (const StreetPoint &p : {trip.start, trip.end}) {
            // Past the last light, from either side, there is room to stop from vMax.
            CHECK(p.offset + city.stopLineDistance() >= stopping);
            CHECK(city.config().blockLength - p.offset + city.stopLineDistance() >= stopping);
        }
        // Throws if they share a street.
        fastestPath(city, PlannerConfig{}, trip);
    }
}

TEST(trip_on_one_street_is_rejected) {
    City city;
    Trip sameWay{{{0, 0}, East, 80.0}, {{0, 0}, East, 120.0}};
    Trip oppositeWay{{{1, 0}, West, 80.0}, {{0, 0}, East, 120.0}};
    CHECK_THROWS(std::invalid_argument, fastestPath(city, PlannerConfig{}, sameWay));
    CHECK_THROWS(std::invalid_argument, fastestPath(city, PlannerConfig{}, oppositeWay));
}

TEST(path_runs_from_the_start_point_to_the_end_point) {
    std::mt19937 rng(3);
    PlannerConfig planner = tripPlannerConfig();
    for (int i = 0; i < 2000; i++) {
        City city(randomCityConfig(rng));
        Trip trip = randomTrip(city, rng);
        TripPath path = fastestPath(city, planner, trip);
        Route route(city, path.nodes, path.endTrim);  // Throws on an undrivable path.

        Vec2 along = dirVector(trip.start.heading) * trip.start.offset;
        Vec2 expected = city.nodeCenter(trip.start.from) + along +
                        rightOf(trip.start.heading) * city.laneOffset();
        RoutePose pose = route.poseAt(path.startPosition);
        CHECK(pose.heading == trip.start.heading);
        CHECK_NEAR(pose.position.x, expected.x, 1e-9);
        CHECK_NEAR(pose.position.y, expected.y, 1e-9);

        // Whichever way it came, the car ends level with the point, in its lane.
        Vec2 point = city.nodeCenter(trip.end.from) + dirVector(trip.end.heading) * trip.end.offset;
        RoutePose last = route.poseAt(route.length());
        CHECK(axisOf(last.heading) == axisOf(trip.end.heading));
        Vec2 gap = last.position - point;
        CHECK_NEAR(dot(gap, dirVector(trip.end.heading)), 0.0, 1e-9);
        CHECK_NEAR(std::abs(dot(gap, rightOf(trip.end.heading))), city.laneOffset(), 1e-9);
    }
}

TEST(a_car_facing_away_from_the_end_goes_around_the_block) {
    City city;
    // The end is just behind the car, on the same row.
    Trip trip{{{2, 0}, East, 100.0}, {{1, 0}, East, 100.0}};
    TripPath path = fastestPath(city, PlannerConfig{}, trip);
    Route route(city, path.nodes, path.endTrim);
    CHECK(path.nodes.front() == (Node{2, 0}));
    CHECK(path.nodes.size() > 4);
    CHECK(route.length() - path.startPosition > 400.0);
}

TEST(the_path_goes_around_a_red_light_when_that_is_faster) {
    // After crossing (1,0) the car can reach (2,1) by two routes of equal length:
    // through (2,0), or through (1,1). Which one is faster depends only on when
    // those two lights are green, so across their phases each route has to win.
    PlannerConfig planner;
    Trip trip{{{0, 0}, East, 100.0}, {{2, 1}, East, 100.0}};
    const Node viaEast{2, 0};
    const Node viaSouth{1, 1};
    CityConfig config;
    config.cols = 4;
    config.rows = 3;
    City city(config);

    bool eastWon = false;
    bool southWon = false;
    for (double east = 0.0; east < 20.0; east += 1.0) {
        for (double south = 0.0; south < 20.0; south += 1.0) {
            city.setLightOffset(viaEast, east);
            city.setLightOffset(viaSouth, south);
            const std::vector<Node> nodes = fastestPath(city, planner, trip).nodes;
            eastWon |= std::find(nodes.begin(), nodes.end(), viaEast) != nodes.end();
            southWon |= std::find(nodes.begin(), nodes.end(), viaSouth) != nodes.end();
        }
    }
    CHECK(eastWon);
    CHECK(southWon);
}

TEST(a_car_goes_straight_on_when_the_end_is_on_the_next_street) {
    CityConfig config;
    config.cols = 3;
    config.rows = 3;
    City city(config);
    Trip trip{{{0, 1}, East, 100.0}, {{1, 1}, East, 100.0}};
    TripPath path = fastestPath(city, PlannerConfig{}, trip);
    CHECK((path.nodes == std::vector<Node>{{0, 1}, {1, 1}, {2, 1}}));
    CHECK_NEAR(path.endTrim, 100.0, 1e-9);
}

TEST(the_estimate_uses_the_true_route_length) {
    // Without a wait the estimate is the route at vMax plus what the ramp at each
    // end loses. Trips that never wait hit this bound, so a wrong distance for a
    // turn would either fall below it or stop it being reached.
    std::mt19937 rng(5);
    PlannerConfig planner = tripPlannerConfig();
    double tightest = 1e9;
    for (int i = 0; i < 4000; i++) {
        City city(randomCityConfig(rng));
        Trip trip = randomTrip(city, rng);
        TripPath path = fastestPath(city, planner, trip);
        Route route(city, path.nodes, path.endTrim);
        double rampLoss = planner.vMax / (2.0 * planner.aMax);
        double freeFlow = (route.length() - path.startPosition) / planner.vMax + 2.0 * rampLoss;
        double slack = path.estimatedTime - freeFlow;
        CHECK(slack >= -1e-9);
        tightest = std::min(tightest, slack);
    }
    CHECK_NEAR(tightest, 0.0, 1e-9);
}

TEST(route_trim_drops_the_lights_past_the_end) {
    City city;
    Route full(city, {{0, 0}, {1, 0}, {2, 0}});
    Route trimmed(city, {{0, 0}, {1, 0}, {2, 0}}, 100.0);
    CHECK(full.lights().size() == 2);
    CHECK(trimmed.lights().size() == 1);
    CHECK_NEAR(trimmed.length(), 300.0, 1e-9);
    CHECK_NEAR(trimmed.poseAt(1000.0).position.x, 300.0, 1e-9);
    CHECK_THROWS(std::invalid_argument, Route(city, {{0, 0}, {1, 0}}, 200.0));
    CHECK_THROWS(std::invalid_argument, Route(city, {{0, 0}, {1, 0}}, -1.0));
}

TEST(polyline_follows_the_corners_between_two_positions) {
    City city;
    Route route(city, {{0, 0}, {1, 0}, {1, 1}});
    double corner = 200.0 - city.laneOffset() + 0.0;
    std::vector<Vec2> points = route.polyline(50.0, route.length() - 50.0);
    CHECK(points.size() == 3);
    CHECK_NEAR(points[0].x, 50.0, 1e-9);
    CHECK_NEAR(points[1].x, corner, 1e-9);
    CHECK_NEAR(points[1].y, city.laneOffset(), 1e-9);
    CHECK_NEAR(points[2].x, corner, 1e-9);
    CHECK(route.polyline(10.0, 20.0).size() == 2);
}

TEST(cars_reach_the_end_and_stop_there_without_running_a_red) {
    std::mt19937 rng(4);
    PlannerConfig planner = tripPlannerConfig();
    for (int i = 0; i < 300; i++) {
        City city(randomCityConfig(rng));
        Trip trip = randomTrip(city, rng);
        TripPath path = fastestPath(city, planner, trip);
        Route route(city, path.nodes, path.endTrim);

        for (double dt : {0.05, 1.0 / 60.0}) {
            SimulationResult result = driveTrip(route, path, planner, dt);
            CHECK(result.finished);
            CHECK(result.crossings.size() == route.lights().size());
            for (const Crossing &c : result.crossings) CHECK(c.frontGreen);
            CHECK(result.samples.back().speed < 1.0);
        }
    }
}

int main() { return testing::runAll(); }
