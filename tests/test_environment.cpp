#include <cmath>
#include <stdexcept>

#include "City.h"
#include "Kinematics.h"
#include "Route.h"
#include "Simulation.h"
#include "TestSupport.h"

// The original light logic, kept here to pin the refactor to its behavior.
static bool originalIsGreen(Node n, Dir heading, double time) {
    double offset = (n.col + n.row) * 2.0;
    bool northSouthGreen = std::fmod(time + offset, 20.0) < 10.0;
    bool travelingNorthSouth = heading == North || heading == South;
    return travelingNorthSouth == northSouthGreen;
}

TEST(lights_behave_as_in_the_original_program) {
    City city;
    for (const TrafficLight &light : city.lights()) {
        for (int d = 0; d < DirCount; d++) {
            Dir heading = static_cast<Dir>(d);
            for (double t = 0.0; t < 45.0; t += 0.13) {
                CHECK(light.isGreen(axisOf(heading), t) == originalIsGreen(light.node, heading, t));
            }
        }
    }
}

TEST(grid_corners_have_no_light) {
    City city;
    CHECK(!city.lightIndexAt({0, 0}).has_value());
    CHECK(!city.lightIndexAt({3, 2}).has_value());
    CHECK(city.lightIndexAt({1, 0}).has_value());
    CHECK(city.lights().size() == 8);
    CHECK_THROWS(std::invalid_argument, city.setLightOffset({0, 0}, 1.0));
}

TEST(each_light_has_ten_seconds_of_each_color) {
    TrafficLight light{{1, 1}, 3.5, 10.0};
    for (Axis axis : {Axis::NorthSouth, Axis::EastWest}) {
        int green = 0;
        const int steps = 20000;
        for (int i = 0; i < steps; i++) green += light.isGreen(axis, i * 0.001) ? 1 : 0;
        CHECK_NEAR(green * 0.001, 10.0, 0.002);
    }
}

TEST(the_two_axes_are_never_green_together) {
    TrafficLight light{{1, 1}, 7.0, 10.0};
    for (double t = -30.0; t < 30.0; t += 0.07) {
        CHECK(light.isGreen(Axis::NorthSouth, t) != light.isGreen(Axis::EastWest, t));
    }
}

TEST(schedule_is_half_open) {
    GreenSchedule s{20.0, 10.0, 12.0};
    CHECK(s.isGreen(12.0));
    CHECK(s.isGreen(21.999));
    CHECK(!s.isGreen(22.0));
    CHECK(!s.isGreen(31.999));
    CHECK(s.isGreen(32.0));
}

TEST(schedule_works_before_time_zero) {
    GreenSchedule s{20.0, 10.0, 12.0};
    CHECK(s.isGreen(-8.0));
    CHECK(!s.isGreen(-8.5));
}

TEST(windows_are_whole_even_when_the_range_starts_inside_one) {
    GreenSchedule s{20.0, 10.0, 12.0};
    CHECK((s.windows(0.0, 40.0) == IntervalSet{{-8, 2}, {12, 22}, {32, 42}}));
    CHECK((s.windows(15.0, 16.0) == IntervalSet{{12, 22}}));
    CHECK(s.windows(23.0, 31.0).empty());
}

TEST(time_to_next_change) {
    GreenSchedule s{20.0, 10.0, 12.0};
    CHECK_NEAR(s.timeToNextChange(15.0), 7.0, 1e-12);
    CHECK_NEAR(s.timeToNextChange(25.0), 7.0, 1e-12);
}

TEST(route_rejects_bad_paths) {
    City city;
    CHECK_THROWS(std::invalid_argument, Route(city, {{0, 0}}));
    CHECK_THROWS(std::invalid_argument, Route(city, {{0, 0}, {2, 0}}));
    CHECK_THROWS(std::invalid_argument, Route(city, {{0, 0}, {1, 0}, {0, 0}}));
    CHECK_THROWS(std::invalid_argument, Route(city, {{3, 0}, {4, 0}}));
}

TEST(straight_route_positions_its_stop_lines) {
    City city;
    Route route(city, {{0, 0}, {1, 0}, {2, 0}});
    CHECK_NEAR(route.length(), 400.0, 1e-9);
    CHECK(route.lights().size() == 2);
    CHECK_NEAR(route.lights()[0].stopLine, 200.0 - city.stopLineDistance(), 1e-9);
    CHECK_NEAR(route.lights()[1].stopLine, 400.0 - city.stopLineDistance(), 1e-9);
    CHECK(route.lights()[0].axis == Axis::EastWest);
}

TEST(turning_route_uses_the_lane_and_incoming_axis) {
    City city;
    Route route(city, {{1, 0}, {1, 1}, {2, 1}});
    // A left turn from southbound onto eastbound crosses to the far lane.
    CHECK(route.lights().size() == 2);
    CHECK(route.lights()[0].axis == Axis::NorthSouth);
    CHECK(route.lights()[1].axis == Axis::EastWest);
    RoutePose pose = route.poseAt(50.0);
    CHECK(pose.heading == South);
    CHECK_NEAR(pose.position.x, 200.0 - city.laneOffset(), 1e-9);
    CHECK_NEAR(pose.position.y, 50.0, 1e-9);
    CHECK(route.poseAt(route.length()).heading == East);
}

TEST(lights_ahead_respects_the_horizon_and_position) {
    City city;
    Route route(city, {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}, {3, 2}});
    double firstLine = route.lights()[0].stopLine;
    CHECK(route.lightsAhead(0.0, 500.0).size() == 2);
    CHECK(route.lightsAhead(0.0, firstLine - 0.01).empty());
    CHECK(route.lightsAhead(0.0, firstLine).size() == 1);  // Exactly at the horizon.
    CHECK_NEAR(route.lightsAhead(firstLine, 1.0)[0].distance, 0.0, 1e-12);
    CHECK_NEAR(route.lightsAhead(firstLine + 0.01, 500.0)[0].distance, 199.99, 1e-6);
}

TEST(extending_a_route_keeps_earlier_positions) {
    City city;
    Route route(city, {{0, 0}, {1, 0}});
    double line = route.lights()[0].stopLine;
    route.extend({1, 1});
    CHECK_NEAR(route.lights()[0].stopLine, line, 1e-9);
    CHECK(route.lights().size() == 2);
    CHECK(route.finalHeading() == South);
}

TEST(travel_time_matches_hand_calculation) {
    // 100 m from 4 m/s toward 10 m/s at 2 m/s^2: 3 s and 21 m of ramp, then 7.9 s.
    CHECK_NEAR(travelTime(100.0, 4.0, 10.0, 2.0), 3.0 + 7.9, 1e-9);
    CHECK_NEAR(travelTime(100.0, 10.0, 10.0, 2.0), 10.0, 1e-9);
    // Ramp longer than the distance: 0 to 10 m/s needs 25 m, only 9 available.
    CHECK_NEAR(travelTime(9.0, 0.0, 10.0, 2.0), 3.0, 1e-9);
    // Braking to a halt short of the line never arrives.
    CHECK(std::isinf(travelTime(100.0, 10.0, 0.0, 2.0)));
    // Braking to a halt with too little room arrives anyway.
    CHECK(std::isfinite(travelTime(10.0, 10.0, 0.0, 2.0)));
}

TEST(cruise_speed_inverts_travel_time) {
    for (double v0 : {0.0, 3.0, 8.0, 13.9}) {
        for (double vCruise : {5.0, 7.0, 10.0, 13.9}) {
            double duration = travelTime(150.0, v0, vCruise, 2.0);
            double found = cruiseSpeedForDuration(150.0, duration, v0, 5.0, 13.9, 2.0);
            CHECK_NEAR(found, vCruise, 1e-6);
        }
    }
}

TEST(infinite_acceleration_is_a_plain_division) {
    double inf = std::numeric_limits<double>::infinity();
    CHECK_NEAR(travelTime(200.0, 3.0, 10.0, inf), 20.0, 1e-12);
    CHECK_NEAR(cruiseSpeedForDuration(200.0, 25.0, 3.0, 5.0, 13.9, inf), 8.0, 1e-9);
}

TEST(step_car_limits_acceleration_and_never_reverses) {
    CarState car{0.0, 10.0};
    stepCar(car, 20.0, 2.0, 0.5);
    CHECK_NEAR(car.speed, 11.0, 1e-12);
    CHECK_NEAR(car.position, 5.25, 1e-12);

    stepCar(car, 0.0, 2.0, 0.5);
    CHECK_NEAR(car.speed, 10.0, 1e-12);

    CarState slow{0.0, 0.5};
    stepCar(slow, 0.0, 2.0, 1.0);
    CHECK(slow.speed == 0.0);
}

int main() { return testing::runAll(); }
