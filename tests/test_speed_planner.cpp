#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "Kinematics.h"
#include "Simulation.h"
#include "SpeedPlanner.h"
#include "TestSupport.h"

namespace {

constexpr double kTolerance = 1e-6;

// Red and green of 10 s, with green starting at `greenStart`.
UpcomingLight lightAt(double distance, double greenStart) {
    UpcomingLight light;
    light.distance = distance;
    light.schedule = GreenSchedule{20.0, 10.0, greenStart};
    return light;
}

PlannerConfig idealConfig() {
    PlannerConfig config;
    config.safetyMargin = 0.0;
    config.minSafetyMargin = 0.0;
    config.accountForTransition = false;
    return config;
}

// Checks a plan from scratch, without trusting the planner's own intermediate sets.
void checkStopFree(const PlannerConfig &config, const PlanRequest &request, const Plan &plan) {
    CHECK(plan.status == PlanStatus::StopFree || plan.status == PlanStatus::ReducedMargin);
    CHECK(plan.lights.size() == request.lights.size());
    double previousTime = request.now;
    double previousDistance = 0.0;
    for (std::size_t i = 0; i < plan.lights.size(); i++) {
        const LightPlan &light = plan.lights[i];
        const UpcomingLight &input = request.lights[i];
        CHECK(!light.fallback);
        CHECK(light.reachable.contains(light.arrivalTime));

        double t = light.arrivalTime;
        CHECK(input.schedule.isGreen(t - light.margin));
        CHECK(input.schedule.isGreen(t + light.margin - 1e-9));

        double duration = t - previousTime;
        double speed = (input.distance - previousDistance) / duration;
        if (i > 0 || !config.accountForTransition) {
            CHECK(speed >= config.vMin - kTolerance);
            CHECK(speed <= config.vMax + kTolerance);
            CHECK_NEAR(light.segmentSpeed, speed, kTolerance);
        }
        previousTime = t;
        previousDistance = input.distance;
    }
}

// Whether a given arrival time at the last light is reachable, by brute force
// over the arrival time at the first.
bool reachableByBruteForce(double t2, double x1, double x2, const GreenSchedule &s1,
                           const GreenSchedule &s2, double vMin, double vMax) {
    if (!s2.isGreen(t2)) return false;
    for (double t1 = x1 / vMax; t1 <= x1 / vMin; t1 += 0.002) {
        double dt = t2 - t1;
        if (s1.isGreen(t1) && dt >= (x2 - x1) / vMax && dt <= (x2 - x1) / vMin) return true;
    }
    return false;
}

}  // namespace

// --- The specified scenarios ---

TEST(single_light_reachable_set) {
    SpeedPlanner planner(idealConfig());
    PlanRequest request{0.0, 13.9, {lightAt(200.0, 12.0)}};
    Plan plan = planner.plan(request);

    const IntervalSet &r = plan.lights[0].reachable;
    CHECK(r.size() == 2);
    CHECK_NEAR(r.intervals()[0].lo, 200.0 / 13.9, 1e-12);
    CHECK_NEAR(r.intervals()[0].lo, 14.4, 0.02);  // The specified, rounded value.
    CHECK_NEAR(r.intervals()[0].hi, 22.0, 1e-12);
    CHECK_NEAR(r.intervals()[1].lo, 32.0, 1e-12);
    CHECK_NEAR(r.intervals()[1].hi, 40.0, 1e-12);
    checkStopFree(planner.config(), request, plan);
}

TEST(several_lights_with_different_offsets) {
    PlannerConfig config;
    config.accountForTransition = false;
    SpeedPlanner planner(config);
    PlanRequest request{3.0, 9.0, {lightAt(120.0, 4.0), lightAt(260.0, 17.0), lightAt(430.0, 9.0)}};
    Plan plan = planner.plan(request);
    checkStopFree(config, request, plan);
    CHECK(plan.status == PlanStatus::StopFree);
    for (const LightPlan &light : plan.lights) {
        CHECK(light.arrivalTime >= light.gate.lo && light.arrivalTime <= light.gate.hi);
    }
    CHECK_NEAR(plan.targetSpeed, plan.lights[0].segmentSpeed, 1e-12);
}

TEST(no_stop_free_plan_falls_back_to_the_next_green) {
    PlannerConfig config;
    config.accountForTransition = false;
    SpeedPlanner planner(config);
    // 30 m away, reachable in [2.16, 6] s, with red until t = 8.
    PlanRequest request{0.0, 10.0, {lightAt(30.0, 8.0)}};
    Plan plan = planner.plan(request);

    CHECK(plan.status == PlanStatus::Fallback);
    CHECK(plan.lights[0].fallback);
    CHECK(plan.lights[0].reachable.empty());
    CHECK_NEAR(plan.lights[0].arrivalTime, 9.0, 1e-9);  // Green start plus the margin.
    CHECK_NEAR(plan.targetSpeed, 30.0 / 9.0, 1e-9);
    CHECK(plan.targetSpeed < config.vMin);
    CHECK(!plan.redCrossingUnavoidable);
}

TEST(all_lights_in_phase_allow_a_constant_speed) {
    PlannerConfig config;
    config.accountForTransition = false;
    SpeedPlanner planner(config);
    // Green at [15, 25] and every 20 s: 10 m/s from t = 0 hits each one mid-green.
    PlanRequest request{0.0, 10.0, {lightAt(200.0, 15.0), lightAt(400.0, 15.0)}};
    Plan plan = planner.plan(request);

    checkStopFree(config, request, plan);
    CHECK_NEAR(plan.lights[0].arrivalTime, 20.0, 1e-9);
    CHECK_NEAR(plan.lights[1].arrivalTime, 40.0, 1e-9);
    for (const LightPlan &light : plan.lights) CHECK_NEAR(light.segmentSpeed, 10.0, 1e-9);
}

TEST(green_wave_is_kept_when_arriving_off_center) {
    // The same wave, but here a constant speed cannot arrive mid-green. Weighting
    // smoothness over centering must recover the constant speed.
    PlannerConfig config;
    config.accountForTransition = false;
    RobustSmoothCost::Weights weights;
    weights.greenCenter = 0.0;
    weights.speedChange = 1.0;
    weights.time = 0.0;
    SpeedPlanner planner(config, std::make_shared<RobustSmoothCost>(8.9, weights));
    PlanRequest request{0.0, 9.0, {lightAt(180.0, 15.0), lightAt(380.0, 15.0)}};
    Plan plan = planner.plan(request);

    checkStopFree(config, request, plan);
    double spread = std::abs(plan.lights[0].segmentSpeed - plan.lights[1].segmentSpeed);
    CHECK(spread < 0.15);
}

// --- Reachable sets ---

TEST(margin_shrinks_green_windows_at_both_ends) {
    PlannerConfig config = idealConfig();
    config.safetyMargin = 1.0;
    config.minSafetyMargin = 1.0;
    SpeedPlanner planner(config);
    Plan plan = planner.plan({0.0, 13.9, {lightAt(200.0, 12.0)}});
    const IntervalSet &r = plan.lights[0].reachable;
    CHECK_NEAR(r.intervals()[0].lo, 200.0 / 13.9, 1e-12);
    CHECK_NEAR(r.intervals()[0].hi, 21.0, 1e-12);
    CHECK_NEAR(r.intervals()[1].lo, 33.0, 1e-12);
    CHECK_NEAR(r.intervals()[1].hi, 40.0, 1e-12);  // Limited by vMin, not by the margin.
}

TEST(a_window_open_at_the_start_still_loses_its_margin_at_its_true_end) {
    PlannerConfig config = idealConfig();
    config.safetyMargin = 1.0;
    config.minSafetyMargin = 1.0;
    SpeedPlanner planner(config);
    // Green over [-3, 7]; reachable from 10 / 13.9 s on.
    Plan plan = planner.plan({0.0, 13.9, {lightAt(10.0, -3.0)}});
    const IntervalSet &r = plan.lights[0].reachable;
    CHECK_NEAR(r.intervals()[0].lo, 10.0 / 13.9, 1e-12);
    CHECK_NEAR(r.min(), 10.0 / 13.9, 1e-12);
    CHECK(r.max() <= 6.0);
}

TEST(windows_cover_the_whole_slow_arrival_range) {
    SpeedPlanner planner(idealConfig());
    Plan plan = planner.plan({0.0, 13.9, {lightAt(500.0, 0.0)}});
    const IntervalSet &r = plan.lights[0].reachable;
    CHECK_NEAR(r.min(), 40.0, 1e-9);  // 500 / 13.9 = 35.97, in red until 40.
    CHECK_NEAR(r.max(), 100.0, 1e-9);  // 500 / 5, and a green starts exactly then.
    CHECK(r.size() == 4);
}

TEST(two_light_set_matches_brute_force) {
    SpeedPlanner planner(idealConfig());
    GreenSchedule s1{20.0, 10.0, 6.0};
    GreenSchedule s2{20.0, 10.0, 19.0};
    double x1 = 90.0;
    double x2 = 260.0;
    UpcomingLight a;
    a.distance = x1;
    a.schedule = s1;
    UpcomingLight b;
    b.distance = x2;
    b.schedule = s2;
    Plan plan = planner.plan({0.0, 13.9, {a, b}});
    const IntervalSet &r = plan.lights[1].reachable;

    // Skip points right next to an edge, where the brute-force grid is too coarse.
    for (double t = 10.0; t < 60.0; t += 0.17) {
        bool nearEdge = false;
        for (const Interval &i : r.intervals()) {
            if (std::abs(t - i.lo) < 0.01 || std::abs(t - i.hi) < 0.01) nearEdge = true;
        }
        if (nearEdge) continue;
        CHECK(r.contains(t) == reachableByBruteForce(t, x1, x2, s1, s2, 5.0, 13.9));
    }
}

// --- Backtracking and the choice of arrival times ---

TEST(window_preference_does_not_move_an_arrival_inside_a_window) {
    // However heavily earliness is weighted, mid-window stays the best arrival.
    PlannerConfig config = idealConfig();
    RobustSmoothCost::Weights weights;
    weights.speedChange = 0.0;
    weights.time = 100.0;
    SpeedPlanner planner(config, std::make_shared<RobustSmoothCost>(8.9, weights));
    Plan plan = planner.plan({0.0, 10.0, {lightAt(200.0, 12.0)}});
    CHECK_NEAR(plan.lights[0].arrivalTime, 17.0, 1e-9);
}

TEST(default_cost_prefers_the_earlier_of_two_equal_windows) {
    SpeedPlanner planner{idealConfig()};
    // Both windows' middles are reachable, and 8.0 m/s and 5.7 m/s differ from the
    // current speed about equally.
    Plan plan = planner.plan({0.0, 6.85, {lightAt(200.0, 20.0)}});
    CHECK_NEAR(plan.lights[0].arrivalTime, 25.0, 1e-9);
}

TEST(default_cost_arrives_mid_green_for_a_lone_light) {
    PlannerConfig config = idealConfig();
    config.safetyMargin = 1.0;
    config.minSafetyMargin = 1.0;
    SpeedPlanner planner(config);
    Plan plan = planner.plan({0.0, 12.0, {lightAt(200.0, 12.0)}});
    CHECK_NEAR(plan.lights[0].arrivalTime, 17.0, 0.5);
    CHECK_NEAR(plan.lights[0].gate.mid(), 17.0, 1e-9);
}

TEST(earliest_arrival_option) {
    PlannerConfig config = idealConfig();
    config.safetyMargin = 1.0;
    config.minSafetyMargin = 1.0;
    SpeedPlanner planner(config, std::make_shared<EarliestArrivalCost>());
    PlanRequest request{0.0, 10.0, {lightAt(200.0, 12.0), lightAt(330.0, 4.0)}};
    Plan plan = planner.plan(request);
    checkStopFree(config, request, plan);
    CHECK_NEAR(plan.lights[0].arrivalTime, 200.0 / 13.9, 1e-9);
    CHECK_NEAR(plan.lights[1].arrivalTime, plan.lights[1].reachable.min(), 1e-9);
}

namespace {
class LatestArrivalCost : public PlanCost {
public:
    double arrival(const ArrivalOption &option) const override { return -option.time; }
    double speedChange(double, double) const override { return 0.0; }
};
}  // namespace

TEST(cost_is_pluggable) {
    SpeedPlanner planner(idealConfig(), std::make_shared<LatestArrivalCost>());
    Plan plan = planner.plan({0.0, 10.0, {lightAt(200.0, 12.0)}});
    CHECK_NEAR(plan.lights[0].arrivalTime, 40.0, 1e-9);
}

TEST(backtracking_avoids_times_that_dead_end) {
    // The first light has two windows but only the later one leads to the second
    // light, which is 13 m further on and open only around t = 41..50.
    PlannerConfig config = idealConfig();
    config.safetyMargin = 1.0;
    config.minSafetyMargin = 1.0;
    SpeedPlanner planner(config, std::make_shared<EarliestArrivalCost>());
    UpcomingLight first = lightAt(200.0, 12.0);
    UpcomingLight second;
    second.distance = 213.0;
    second.schedule = GreenSchedule{60.0, 10.0, 40.0};
    PlanRequest request{0.0, 10.0, {first, second}};
    Plan plan = planner.plan(request);

    checkStopFree(config, request, plan);
    CHECK(plan.lights[0].reachable.size() == 2);
    CHECK(plan.lights[0].arrivalTime > 30.0);
}

TEST(degenerate_single_point_chain_is_found) {
    // Only one exact chain works: 10 s at vMax to the last instant of the first gate,
    // then 10 s more to reach the second gate. It must survive backward-pass rounding.
    PlannerConfig config = idealConfig();
    SpeedPlanner planner(config);
    UpcomingLight first;
    first.distance = 139.0;
    first.schedule = GreenSchedule{1000.0, 5.0, 5.0};  // [5, 10] closes as vMax arrives.
    UpcomingLight second;
    second.distance = 278.0;
    second.schedule = GreenSchedule{1000.0, 5.0, 15.0};
    Plan plan = planner.plan({0.0, 13.9, {first, second}});
    CHECK(plan.status == PlanStatus::StopFree);
    CHECK_NEAR(plan.lights[0].arrivalTime, 10.0, 1e-9);
    CHECK_NEAR(plan.lights[1].arrivalTime, 20.0, 5e-9);
}

// --- Fallback ---

TEST(fallback_at_a_later_light_keeps_the_earlier_ones_valid_and_continues) {
    PlannerConfig config;
    config.accountForTransition = false;
    SpeedPlanner planner(config);
    // Light 1 is easy. Light 2 sits 20 m on and is closed all through the 2.6 s that
    // segment can take, for every feasible arrival at light 1.
    UpcomingLight first;
    first.distance = 100.0;
    first.schedule = GreenSchedule{60.0, 11.0, 8.0};  // Open [8, 19].
    UpcomingLight second;
    second.distance = 120.0;
    second.schedule = GreenSchedule{60.0, 10.0, 30.0};  // Next open at t = 30.
    UpcomingLight third = lightAt(280.0, 45.0);
    Plan plan = planner.plan({0.0, 10.0, {first, second, third}});

    CHECK(plan.status == PlanStatus::Fallback);
    CHECK(!plan.lights[0].fallback);
    CHECK(plan.lights[0].reachable.contains(plan.lights[0].arrivalTime));
    CHECK(plan.lights[1].fallback);
    CHECK_NEAR(plan.lights[1].arrivalTime, 31.0, 1e-9);
    CHECK(plan.lights[1].segmentSpeed < config.vMin);
    CHECK(!plan.lights[2].fallback);
    CHECK(plan.lights[2].reachable.min() >= 31.0);
    // Only the slow segment is allowed below vMin.
    CHECK(plan.targetSpeed >= config.vMin);
}

TEST(waiting_at_the_line_goes_when_green_comes) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    Plan waiting = planner.plan({5.0, 0.0, {lightAt(0.0, 12.0)}});
    CHECK(waiting.status == PlanStatus::Fallback);
    CHECK(waiting.targetSpeed == 0.0);

    Plan going = planner.plan({12.5, 0.0, {lightAt(0.0, 12.0)}});
    CHECK(going.status != PlanStatus::Fallback);
    CHECK_NEAR(going.targetSpeed, config.vMax, 1e-9);
}

TEST(unstoppable_car_at_a_red_light_is_reported) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    // 13.9 m/s needs 48 m to stop at 2 m/s^2. Only 20 m left, red for 8 s.
    Plan plan = planner.plan({0.0, 13.9, {lightAt(20.0, 8.0)}});
    CHECK(plan.status == PlanStatus::Fallback);
    CHECK(plan.redCrossingUnavoidable);
    CHECK(plan.targetSpeed == 0.0);  // Brakes as hard as allowed regardless.
}

// --- Margin retry ---

TEST(a_car_committed_to_a_green_line_is_not_sent_into_fallback) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    // 3 m from the line at 10 m/s, 0.5 s into green. Even braking at a_max the car
    // arrives by t = 12.31, inside the 1 s margin, but the light is plainly green.
    Plan plan = planner.plan({12.0, 10.0, {lightAt(3.0, 11.5)}});
    CHECK(plan.status == PlanStatus::ReducedMargin);
    CHECK_NEAR(plan.lights[0].margin, config.minSafetyMargin, 1e-12);
    CHECK(plan.targetSpeed >= config.vMin);
}

TEST(margin_retry_is_per_light) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    Plan plan = planner.plan({12.0, 10.0, {lightAt(3.0, 11.5), lightAt(300.0, 20.0)}});
    CHECK(plan.status == PlanStatus::ReducedMargin);
    CHECK_NEAR(plan.lights[0].margin, config.minSafetyMargin, 1e-12);
    CHECK_NEAR(plan.lights[1].margin, config.safetyMargin, 1e-12);
}

TEST(margin_retry_can_be_turned_off) {
    PlannerConfig config;
    config.minSafetyMargin = config.safetyMargin;
    SpeedPlanner planner(config);
    Plan plan = planner.plan({12.0, 10.0, {lightAt(3.0, 11.5)}});
    CHECK(plan.status == PlanStatus::Fallback);
}

// --- Acceleration ---

TEST(transition_narrows_the_first_leg) {
    PlannerConfig config = idealConfig();
    config.accountForTransition = true;
    SpeedPlanner planner(config);
    // From rest, 13.9 m/s takes 6.95 s and 48.3 m, so arrival costs 3.48 s extra.
    Plan plan = planner.plan({0.0, 0.0, {lightAt(200.0, 12.0)}});
    double earliest = 6.95 + (200.0 - 13.9 * 13.9 / 4.0) / 13.9;
    CHECK_NEAR(plan.lights[0].reachable.min(), earliest, 1e-9);
}

TEST(following_the_target_speed_under_a_max_hits_the_planned_time) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    for (double v0 : {0.0, 5.0, 9.0, 13.9}) {
        Plan plan = planner.plan({0.0, v0, {lightAt(150.0, 8.0)}});
        CHECK(plan.status == PlanStatus::StopFree);

        // No replanning: a single target speed, held to the line.
        CarState car{0.0, v0};
        double t = 0.0;
        const double dt = 0.001;
        while (car.position < 150.0) {
            stepCar(car, plan.targetSpeed, config.aMax, dt);
            t += dt;
        }
        CHECK_NEAR(t, plan.lights[0].arrivalTime, 0.01);
    }
}

TEST(a_plan_stays_put_when_followed) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    std::vector<UpcomingLight> lights = {lightAt(150.0, 8.0), lightAt(340.0, 3.0)};
    Plan first = planner.plan({0.0, 9.0, lights});

    CarState car{0.0, 9.0};
    double t = 0.0;
    const double dt = 0.05;
    while (t < 5.0) {
        stepCar(car, first.targetSpeed, config.aMax, dt);
        t += dt;
    }
    std::vector<UpcomingLight> ahead = lights;
    for (UpcomingLight &light : ahead) light.distance -= car.position;
    Plan later = planner.plan({t, car.speed, ahead});
    CHECK_NEAR(later.lights[0].arrivalTime, first.lights[0].arrivalTime, 0.3);
}

TEST(choice_avoids_chains_that_acceleration_cannot_follow) {
    // Light 2 is 30 m after light 1 and opens 5 s after it closes. Crossing light 1
    // at speed looks fine with instant speed changes, but no car can then shed
    // enough speed in 30 m. The plan must cross light 1 slowly enough to make it.
    PlannerConfig config;
    SpeedPlanner planner(config, std::make_shared<EarliestArrivalCost>());
    UpcomingLight first;
    first.distance = 150.0;
    first.schedule = GreenSchedule{100.0, 15.0, 5.0};  // Open [5, 20].
    UpcomingLight second;
    second.distance = 180.0;
    second.schedule = GreenSchedule{100.0, 15.0, 20.0};  // Open [20, 35].
    Plan plan = planner.plan({0.0, 13.9, {first, second}});

    CHECK(plan.status == PlanStatus::StopFree);
    double duration = plan.lights[1].arrivalTime - plan.lights[0].arrivalTime;
    double speedIn = plan.lights[0].segmentSpeed;
    CHECK(travelTime(30.0, speedIn, config.vMin, config.aMax) >= duration - 1e-6);
    CHECK(travelTime(30.0, speedIn, config.vMax, config.aMax) <= duration + 1e-6);
}

TEST(a_planned_wait_at_a_later_light_caps_speed_to_a_braking_curve) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    // Light 1 is right here and green; light 2 is 25 m on and shut for a long while.
    UpcomingLight first;
    first.distance = 5.0;
    first.schedule = GreenSchedule{100.0, 50.0, -10.0};
    UpcomingLight second;
    second.distance = 30.0;
    second.schedule = GreenSchedule{100.0, 10.0, 40.0};
    Plan plan = planner.plan({0.0, 13.9, {first, second}});

    CHECK(plan.status == PlanStatus::Fallback);
    CHECK(plan.lights[1].fallback);
    CHECK(plan.targetSpeed <= std::sqrt(2.0 * config.aMax * 30.0) + 1e-9);
}

TEST(advised_speed_does_not_jitter_near_the_line) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    // The second light rewards a change of speed before the first line, which is
    // what used to flip between coarse arrival times in the last few metres.
    double previous = -1.0;
    double worst = 0.0;
    CarState car{0.0, 8.0};
    const double dt = 0.05;
    const double firstLine = 120.0;
    double t = 4.0;
    while (car.position < firstLine - 1.0) {
        Plan plan = planner.plan({t, car.speed,
                                  {lightAt(firstLine - car.position, 6.0),
                                   lightAt(firstLine + 200.0 - car.position, 12.0)}});
        if (previous >= 0.0) worst = std::max(worst, std::abs(plan.targetSpeed - previous));
        previous = plan.targetSpeed;
        stepCar(car, plan.targetSpeed, config.aMax, dt);
        t += dt;
    }
    CHECK(worst < 1.0);
}

// --- Car length and extension points ---

TEST(whole_car_rule_closes_the_gate_early) {
    PlannerConfig config = idealConfig();
    config.crossingRule = CrossingRule::WholeCarPastStopLine;
    config.carLength = 5.0;
    SpeedPlanner planner(config);
    Plan plan = planner.plan({0.0, 13.9, {lightAt(200.0, 12.0)}});
    const IntervalSet &r = plan.lights[0].reachable;
    CHECK_NEAR(r.intervals()[0].hi, 21.0, 1e-12);  // 5 m at the slowest 5 m/s: 1 s.
    CHECK_NEAR(r.intervals()[1].lo, 32.0, 1e-12);  // Opening time is unaffected.
}

TEST(gate_open_delay_moves_only_the_opening) {
    SpeedPlanner planner(idealConfig());
    UpcomingLight light = lightAt(200.0, 12.0);
    light.gateOpenDelay = 3.0;
    Plan plan = planner.plan({0.0, 13.9, {light}});
    const IntervalSet &r = plan.lights[0].reachable;
    CHECK_NEAR(r.intervals()[0].lo, 15.0, 1e-12);
    CHECK_NEAR(r.intervals()[0].hi, 22.0, 1e-12);
    CHECK_NEAR(r.intervals()[1].lo, 35.0, 1e-12);
}

// --- Inputs ---

TEST(no_lights_means_cruising) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    Plan plan = planner.plan({0.0, 5.0, {}});
    CHECK(plan.status == PlanStatus::NoLightsAhead);
    CHECK(plan.targetSpeed == config.vMax);
}

TEST(lights_past_the_horizon_are_ignored_and_order_does_not_matter) {
    SpeedPlanner planner(PlannerConfig{});
    Plan plan = planner.plan({0.0, 10.0, {lightAt(600.0, 3.0), lightAt(300.0, 3.0),
                                          lightAt(100.0, 7.0)}});
    CHECK(plan.lights.size() == 2);
    CHECK(plan.lights[0].distance == 100.0);
    CHECK(plan.lights[1].distance == 300.0);
}

TEST(a_light_exactly_at_the_horizon_counts) {
    SpeedPlanner planner(PlannerConfig{});
    CHECK(planner.plan({0.0, 10.0, {lightAt(500.0, 3.0)}}).lights.size() == 1);
}

TEST(a_gate_that_never_opens_is_an_error) {
    PlannerConfig config;
    SpeedPlanner planner(config);
    UpcomingLight light = lightAt(30.0, 8.0);
    light.gateOpenDelay = 50.0;
    CHECK_THROWS(std::invalid_argument, planner.plan({0.0, 10.0, {light}}));
}

TEST(invalid_configurations_are_rejected) {
    auto build = [](PlannerConfig config) { return SpeedPlanner(config); };
    PlannerConfig config;
    config.vMin = 20.0;
    CHECK_THROWS(std::invalid_argument, build(config));
    config = {};
    config.vMin = 0.0;
    CHECK_THROWS(std::invalid_argument, build(config));
    config = {};
    config.aMax = 0.0;
    CHECK_THROWS(std::invalid_argument, build(config));
    config = {};
    config.minSafetyMargin = 2.0;
    CHECK_THROWS(std::invalid_argument, build(config));
    config = {};
    config.candidateStep = 0.0;
    CHECK_THROWS(std::invalid_argument, build(config));
    config = {};
    config.candidateSpeedStep = 0.0;
    CHECK_THROWS(std::invalid_argument, build(config));
}

int main() { return testing::runAll(); }
