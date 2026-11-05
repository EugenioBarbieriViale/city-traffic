#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "City.h"
#include "Kinematics.h"
#include "Route.h"
#include "SpeedPlanner.h"
#include "TestSupport.h"
#include "Traffic.h"
#include "Trip.h"

namespace {

constexpr double kDt = 1.0 / 30.0;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// A schedule that is green during [0, 5), [10, 15), ...
GreenSchedule fiveSecondGreen() {
    GreenSchedule s;
    s.cycle = 10.0;
    s.greenDuration = 5.0;
    return s;
}

bool passesThrough(const std::vector<Node> &nodes, Node a, Node b) {
    for (std::size_t i = 0; i + 1 < nodes.size(); i++) {
        if (nodes[i] == a && nodes[i + 1] == b) return true;
    }
    return false;
}

// Six cars standing up to the next stop line until `until`, then gone.
Inbox standingQueue(const Car &car, double until, double time) {
    Inbox inbox;
    if (time >= until) return inbox;
    const std::vector<RouteLight> &lights = car.route().lights();
    const std::size_t next = car.route().nextLightIndex(car.state().position);
    if (next >= lights.size()) return inbox;
    for (int i = 0; i < 6; i++) {
        const double position = lights[next].stopLine - i * 6.5;
        if (position > car.state().position) {
            inbox.push_back({car.route().poseAt(position).position, {0.0, 0.0}});
        }
    }
    return inbox;
}

// A car placed on `trip` that stays alone, for feeding it hand-made views.
Car loneCar(const City &city, const Trip &trip, TrafficConfig config = {}) {
    return Car(city, tripPlannerConfig(), config, trip, 0.0);
}

}  // namespace

TEST(no_queue_means_no_constraint) {
    CHECK(queueClearTime(fiveSecondGreen(), {}, {50.0, 10.0}, 0.0, tripPlannerConfig(),
                         TrafficConfig{}) == -kInfinity);
}

TEST(a_queue_holds_a_car_back_by_more_the_longer_it_is) {
    const PlannerConfig planner = tripPlannerConfig();
    const TrafficConfig traffic;
    double previous = -kInfinity;
    std::vector<QueuedCar> ahead;
    for (int cars = 1; cars <= 8; cars++) {
        ahead.push_back({(cars - 1) * 6.5, 0.0});
        const QueuedCar self{cars * 6.5, 0.0};
        const double clear = queueClearTime(fiveSecondGreen(), ahead, self, 5.0, planner, traffic);
        CHECK(clear > previous);
        previous = clear;
    }
}

TEST(a_long_queue_takes_more_than_one_green) {
    std::vector<QueuedCar> ahead;
    for (int i = 0; i < 8; i++) ahead.push_back({i * 6.5, 0.0});
    const double clear = queueClearTime(fiveSecondGreen(), ahead, {8 * 6.5, 0.0}, 5.0,
                                        tripPlannerConfig(), TrafficConfig{});
    CHECK(clear > 20.0);  // The green at 10 is not enough for eight cars.
}

TEST(a_car_arriving_at_speed_follows_closer_than_one_leaving_a_queue) {
    const PlannerConfig planner = tripPlannerConfig();
    const TrafficConfig traffic;
    const GreenSchedule green = fiveSecondGreen();
    const double moving = queueClearTime(green, {{20.0, 20.0}}, {40.0, 20.0}, 0.0, planner, traffic);
    const double standing = queueClearTime(green, {{0.0, 0.0}}, {6.5, 0.0}, 0.0, planner, traffic);
    CHECK(moving - 1.0 < standing - 0.25);
}

TEST(following_speed_stops_behind_a_standing_leader_and_matches_a_moving_one) {
    CHECK_NEAR(followingSpeed(0.0, 0.0, 2.0, 0.5), 0.0, 1e-12);
    CHECK(followingSpeed(50.0, 0.0, 2.0, 0.5) > followingSpeed(10.0, 0.0, 2.0, 0.5));
    // With a stopping distance to spare, a car can keep up with its leader.
    CHECK(followingSpeed(20.0, 15.0, 2.0, 0.5) >= 15.0);
}

TEST(a_car_too_close_falls_back_from_its_leader) {
    CHECK(followingSpeed(-3.0, 10.0, 2.0, 0.5) < followingSpeed(0.0, 10.0, 2.0, 0.5) - 3.0);
    CHECK_NEAR(followingSpeed(-50.0, 10.0, 2.0, 0.5), 0.0, 1e-12);
}

TEST(not_before_moves_the_plan_to_a_later_green) {
    PlannerConfig config = tripPlannerConfig();
    SpeedPlanner planner(config, std::make_shared<EarliestArrivalCost>());
    UpcomingLight light;
    light.distance = 60.0;
    light.schedule = fiveSecondGreen();

    PlanRequest request;
    request.speed = 20.0;
    request.lights = {light};
    const double free = planner.plan(request).lights.front().arrivalTime;

    request.lights.front().notBefore = 12.0;
    const Plan held = planner.plan(request);
    CHECK(free < 5.0);
    CHECK_NEAR(held.lights.front().arrivalTime, 12.0, 1e-9);
    CHECK(held.targetSpeed < 20.0);

    // Past the end of a green it opens the next one instead of throwing.
    request.lights.front().notBefore = 14.9;
    CHECK(planner.plan(request).lights.front().arrivalTime >= 20.0);
}

TEST(street_coordinates_agree_across_routes) {
    City city;
    Route straight(city, {{0, 1}, {1, 1}, {2, 1}});
    Route turning(city, {{1, 0}, {1, 1}, {2, 1}});
    const std::size_t k = 1;
    CHECK(straight.streetIndex(k) == turning.streetIndex(k));
    // The same world position gives the same coordinate on both routes.
    const double a = straight.streetEnd(0) + 100.0;
    const double b = turning.streetEnd(0) + 100.0 + city.laneOffset();  // Turned left.
    CHECK_NEAR(straight.poseAt(a).position.x, turning.poseAt(b).position.x, 1e-9);
    CHECK_NEAR(straight.streetCoordinate(k, a), turning.streetCoordinate(k, b), 1e-9);
}

TEST(the_search_goes_around_a_street_expected_to_be_slow) {
    CityConfig config;
    config.cols = 4;
    config.rows = 3;
    City city(config);
    const PlannerConfig planner = tripPlannerConfig();
    const Trip trip{{{0, 1}, East, 100.0}, {{2, 1}, East, 100.0}};

    const std::vector<Node> free = fastestPath(city, planner, trip).nodes;
    CHECK(passesThrough(free, {1, 1}, {2, 1}));

    PathOptions options;
    options.streetDelay = [](Node from, Dir heading) {
        return from == Node{1, 1} && heading == East ? 300.0 : 0.0;
    };
    const TripPath around = fastestPath(city, planner, trip, options);
    CHECK(!passesThrough(around.nodes, {1, 1}, {2, 1}));
    CHECK(around.estimatedTime < 300.0);
}

TEST(the_search_counts_the_queue_on_the_cars_own_street) {
    City city;
    const PlannerConfig planner = tripPlannerConfig();
    const Trip trip{{{0, 1}, East, 100.0}, {{2, 1}, East, 100.0}};
    PathOptions options;
    const double free = fastestPath(city, planner, trip, options).estimatedTime;
    options.notBefore = 60.0;
    CHECK(fastestPath(city, planner, trip, options).estimatedTime >= free + 40.0);
}

TEST(the_estimate_counts_from_the_start_time) {
    City city;
    const PlannerConfig planner = tripPlannerConfig();
    const Trip trip{{{0, 1}, East, 100.0}, {{2, 1}, East, 100.0}};
    PathOptions options;
    options.startTime = 20.0;  // A whole light cycle later: the same lights.
    CHECK_NEAR(fastestPath(city, planner, trip, options).estimatedTime,
               fastestPath(city, planner, trip).estimatedTime, 1e-9);
}

TEST(memory_fades_and_the_latest_visit_wins) {
    City city;
    StreetMemory memory(city, 100.0);
    const int street = city.streetIndex({0, 0}, East);
    CHECK_NEAR(memory.expectedDelay(street, 50.0), 0.0, 1e-12);
    memory.remember(street, 40.0, 50.0);
    CHECK_NEAR(memory.expectedDelay(street, 50.0), 40.0, 1e-9);
    CHECK_NEAR(memory.expectedDelay(street, 150.0), 20.0, 1e-9);
    CHECK_NEAR(memory.expectedDelay(city.streetIndex({0, 0}, South), 150.0), 0.0, 1e-12);
    memory.remember(street, 0.0, 200.0);
    CHECK_NEAR(memory.expectedDelay(street, 200.0), 0.0, 1e-12);
}

TEST(a_car_learns_only_the_street_it_is_on) {
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    const Trip trip{{{0, 2}, East, 100.0}, {{3, 2}, East, 100.0}};
    TrafficConfig traffic;
    traffic.rerouting = false;  // Keeps the route, and so the street being driven.
    Car car = loneCar(city, trip, traffic);
    const int firstStreet = car.route().streetIndex(0);

    // A long standing queue right in front of the car, there for a minute.
    double time = 0.0;
    while (car.route().streetIndex(car.route().streetAt(car.state().position)) == firstStreet ||
           time < 60.0) {
        car.step(standingQueue(car, 60.0, time), time, kDt);
        time += kDt;
        if (time > 300.0) break;
    }
    car.step({}, time, kDt);  // The step that notices the street has changed.

    CHECK(car.memory().expectedDelay(firstStreet, time) > 5.0);
    int rememberedElsewhere = 0;
    for (int i = 0; i < city.streetIndexCount(); i++) {
        if (i != firstStreet && car.memory().expectedDelay(i, time) > 0.0) rememberedElsewhere++;
    }
    CHECK(rememberedElsewhere == 0);
}

TEST(rerouting_keeps_the_car_where_it_is) {
    std::mt19937 rng(7);
    CityConfig config;
    config.cols = 4;
    config.rows = 4;
    config.lightPhaseSeconds = 5.0;
    City city(config);
    TrafficConfig traffic;
    traffic.cars = 40;
    Traffic all(city, tripPlannerConfig(), traffic, rng);

    std::vector<Vec2> before;
    for (const Car &car : all.cars()) before.push_back(car.route().poseAt(car.state().position).position);
    int previousReroutes = 0;
    for (int i = 0; i < 6000; i++) {
        all.step(kDt);
        for (std::size_t c = 0; c < all.cars().size(); c++) {
            const Car &car = all.cars()[c];
            const Vec2 now = car.route().poseAt(car.state().position).position;
            const Vec2 moved = now - before[c];
            // Any jump in position would show here, a reroute's included.
            CHECK(std::sqrt(dot(moved, moved)) < 21.0 * kDt + 1.0 + 2.0 * city.laneOffset());
            before[c] = now;
        }
        previousReroutes = all.stats().reroutes;
    }
    CHECK(previousReroutes > 0);
}

TEST(traffic_conserves_cars_and_keeps_them_on_their_routes) {
    std::mt19937 rng(11);
    City city(randomCityConfig(rng));
    TrafficConfig config;
    config.cars = 8;
    Traffic traffic(city, tripPlannerConfig(), config, rng);
    while (traffic.time() < 300.0) traffic.step(kDt);
    CHECK(traffic.cars().size() == 8);
    CHECK(traffic.stats().tripsFinished >= 8);
    for (const Car &car : traffic.cars()) {
        CHECK(car.state().speed <= tripPlannerConfig().vMax + 1e-9);
        CHECK(car.state().speed >= 0.0);
    }
}

TEST(too_many_cars_are_refused) {
    std::mt19937 rng(13);
    CityConfig config;
    config.cols = 3;
    config.rows = 3;
    City city(config);
    TrafficConfig traffic;
    traffic.cars = 1000;
    CHECK_THROWS(std::invalid_argument, Traffic(city, tripPlannerConfig(), traffic, rng));
}

TEST(hotspot_trips_end_next_to_the_center) {
    std::mt19937 rng(17);
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    const Node center{2, 2};
    for (int i = 0; i < 500; i++) {
        const Trip trip = randomTrip(city, rng, 1.0);
        CHECK(trip.end.from == center || neighbor(trip.end.from, trip.end.heading) == center);
    }
}

TEST(a_car_arriving_behind_a_queue_waits_for_a_later_green) {
    // The same approach twice, with and without knowing about the queue.
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    const Trip trip{{{0, 2}, East, 100.0}, {{3, 2}, East, 100.0}};

    TrafficConfig traffic;
    traffic.rerouting = false;
    Car aware = loneCar(city, trip, traffic);
    traffic.queueAwareSpeed = false;
    Car blind = loneCar(city, trip, traffic);

    double time = 0.0;
    bool slower = false;
    while (time < 3.0) {
        aware.step(standingQueue(aware, 100.0, time), time, kDt);
        blind.step(standingQueue(blind, 100.0, time), time, kDt);
        time += kDt;
        slower |= aware.plan().targetSpeed < blind.plan().targetSpeed - 0.5;
    }
    CHECK(slower);
}

TEST(a_route_finds_only_cars_in_its_own_lane_going_its_way) {
    City city;
    Route route(city, {{0, 1}, {1, 1}, {2, 1}});
    const Vec2 east = dirVector(East);
    const Vec2 ahead = route.poseAt(100.0).position;
    CHECK_NEAR(*route.locate(ahead, east * 10.0, 50.0), 100.0, 1e-9);
    CHECK_NEAR(*route.locate(ahead, {0.0, 0.0}, 50.0), 100.0, 1e-9);  // Standing.
    CHECK(!route.locate(ahead, east * -10.0, 50.0));                  // Going the other way.
    CHECK(!route.locate(ahead, east * 10.0, 150.0));                  // Behind.

    const Vec2 otherLane = city.nodeCenter({0, 1}) + east * 100.0 + rightOf(West) * city.laneOffset();
    CHECK(!route.locate(otherLane, {0.0, 0.0}, 50.0));
    CHECK(!route.locate(city.nodeCenter({1, 0}) + rightOf(South) * city.laneOffset(),
                        dirVector(South) * 5.0, 0.0));
    // The next street counts: the car does not have to be on it yet.
    CHECK_NEAR(*route.locate(route.poseAt(300.0).position, east * 5.0, 50.0), 300.0, 1e-9);
}

TEST(the_map_tells_a_street_from_a_position) {
    City city;
    Route route(city, {{0, 1}, {1, 1}, {1, 2}});
    const double second = route.streetEnd(0) + 50.0;
    CHECK(city.streetAt(route.poseAt(50.0).position, {0.0, 0.0}) == city.streetIndex({0, 1}, East));
    CHECK(city.streetAt(route.poseAt(second).position, {0.0, 0.0}) ==
          city.streetIndex({1, 1}, South));
    CHECK(city.streetAt(route.poseAt(second).position, dirVector(South) * 3.0) ==
          city.streetIndex({1, 1}, South));
    CHECK(!city.streetAt({50.0, 57.0}, {0.0, 0.0}));
}

TEST(radio_reaches_every_car_in_range_and_none_beyond) {
    std::mt19937 rng(19);
    CityConfig cityConfig;
    cityConfig.cols = 6;
    cityConfig.rows = 6;
    City city(cityConfig);
    TrafficConfig config;
    config.cars = 60;
    config.radioRange = 250.0;
    Traffic traffic(city, tripPlannerConfig(), config, rng);
    for (int i = 0; i < 30; i++) traffic.step(kDt);

    for (std::size_t i = 0; i < traffic.cars().size(); i++) {
        const Vec2 here = traffic.cars()[i].message().position;
        std::size_t inRange = 0;
        for (std::size_t j = 0; j < traffic.cars().size(); j++) {
            const Vec2 apart = traffic.cars()[j].message().position - here;
            inRange += j != i && dot(apart, apart) <= 250.0 * 250.0 ? 1 : 0;
        }
        CHECK(traffic.inbox(i).size() == inRange);
        for (const CarMessage &m : traffic.inbox(i)) {
            const Vec2 apart = m.position - here;
            CHECK(dot(apart, apart) <= 250.0 * 250.0);
        }
    }
}

TEST(a_message_carries_position_and_velocity) {
    City city;
    Car car = loneCar(city, {{{0, 1}, East, 100.0}, {{2, 1}, East, 100.0}});
    for (int i = 0; i < 90; i++) car.step({}, i * kDt, kDt);
    const CarMessage message = car.message();
    CHECK_NEAR(message.position.x, car.state().position, 1e-9);
    CHECK_NEAR(message.velocity.x, car.state().speed, 1e-9);
    CHECK_NEAR(message.velocity.y, 0.0, 1e-9);
    CHECK(car.state().speed > 1.0);
}

TEST(a_car_goes_around_a_jam_it_only_heard_about) {
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    const Trip commute{{{0, 2}, East, 30.0}, {{3, 2}, East, 100.0}};
    Car car = loneCar(city, commute);
    CHECK(passesThrough(car.route().nodes(), {1, 2}, {2, 2}));

    // Eight cars standing on the street after the next crossing.
    Route jammed(city, {{1, 2}, {2, 2}, {3, 2}});
    Inbox inbox;
    for (int i = 0; i < 8; i++) {
        inbox.push_back({jammed.poseAt(170.0 - i * 6.5).position, {0.0, 0.0}});
    }
    double time = 0.0;
    while (time < 3.0) {
        car.step(inbox, time, kDt);
        time += kDt;
    }
    CHECK(car.reroutes() > 0);
    CHECK(!passesThrough(car.route().nodes(), {1, 2}, {2, 2}));
}

TEST(a_car_out_of_range_of_a_jam_does_not_know_of_it) {
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    Car car = loneCar(city, {{{0, 2}, East, 30.0}, {{3, 2}, East, 100.0}});
    for (int i = 0; i < 90; i++) car.step({}, i * kDt, kDt);
    CHECK(car.reroutes() == 0);
    CHECK(passesThrough(car.route().nodes(), {1, 2}, {2, 2}));
}

TEST(a_car_stays_out_of_a_crossing_with_no_room_beyond) {
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    TrafficConfig traffic;
    traffic.rerouting = false;
    Car car = loneCar(city, {{{0, 2}, East, 100.0}, {{2, 2}, East, 100.0}}, traffic);
    const double line = car.route().lights().front().stopLine;
    const Inbox blocked = {{car.route().poseAt(line + 44.0).position, {0.0, 0.0}}};
    for (int i = 0; i < 1200; i++) car.step(blocked, i * kDt, kDt);
    CHECK(car.state().position < line);
    CHECK(car.state().speed < 0.5);

    double time = 1200 * kDt;
    while (car.state().position < line && time < 100.0) {
        car.step({}, time, kDt);
        time += kDt;
    }
    CHECK(car.state().position >= line);
}

TEST(a_left_turn_waits_for_an_oncoming_car) {
    CityConfig config;
    config.cols = 5;
    config.rows = 5;
    City city(config);
    TrafficConfig traffic;
    traffic.rerouting = false;
    const Trip left{{{0, 2}, East, 100.0}, {{1, 2}, North, 100.0}};
    Car waiting = loneCar(city, left, traffic);
    Car free = loneCar(city, left, traffic);
    CHECK(waiting.route().turnsLeftAfter(0));

    const Vec2 entry = city.turnPoint({1, 2}, West, West);
    const Inbox oncoming = {{entry - dirVector(West) * 15.0, dirVector(West) * 15.0}};
    const double line = waiting.route().lights().front().stopLine;
    for (int i = 0; i < 1200; i++) {
        waiting.step(oncoming, i * kDt, kDt);
        free.step({}, i * kDt, kDt);
    }
    CHECK(waiting.state().position < line);
    CHECK(free.state().position > line);
}

TEST(a_busy_count_grows_with_the_city) {
    CityConfig small;
    small.cols = 3;
    small.rows = 3;
    CityConfig large;
    large.cols = 9;
    large.rows = 7;
    CHECK(busyCarCount(City(large)) > 3 * busyCarCount(City(small)));
}

int main() { return testing::runAll(); }
