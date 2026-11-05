#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "Route.h"
#include "SpeedPlanner.h"

struct CarState {
    double position = 0.0;  // Of the front bumper, metres along the route.
    double speed = 0.0;
};

// Moves toward the target speed, limited by aMax, with exact constant-acceleration
// integration over dt.
void stepCar(CarState &car, double targetSpeed, double aMax, double dt);

// The same with a hard ceiling on top: `targetSpeed` is approached within aMax as
// usual, but the car brakes up to brakeMax to stay under `speedLimit`.
void stepCar(CarState &car, double targetSpeed, double speedLimit, double aMax, double brakeMax,
             double dt);

// The share of aMax a car plans to brake with on arrival. The rest is what lets
// it catch up with the braking curve after entering it one time step late.
constexpr double kArrivalBrakingShare = 0.95;

// The highest speed from which the car still stops at the end of the route.
double arrivalSpeedLimit(const Route &route, const CarState &car, double aMax);

// The receding-horizon loop. It replans at a fixed rate, and also whenever a
// light is passed, and only ever applies the current segment's target speed.
class GlosaController {
public:
    // replanInterval <= 0 replans on every call.
    GlosaController(const PlannerConfig &config, double replanInterval,
                    std::shared_ptr<const PlanCost> cost = {});

    // `notBefore` holds back the arrival at the next light. See UpcomingLight.
    double targetSpeed(const Route &route, const CarState &car, double time,
                       double notBefore = -std::numeric_limits<double>::infinity());
    const Plan &lastPlan() const { return plan_; }

private:
    SpeedPlanner planner_;
    double replanInterval_;
    double lastPlanTime_ = 0.0;
    std::size_t lastNextLight_ = 0;
    bool hasPlan_ = false;
    Plan plan_;
};

using SpeedPolicy = std::function<double(const Route &, const CarState &, double time)>;

struct SimulationOptions {
    double dt = 0.05;
    double maxTime = 600.0;
    double aMax = 2.0;
    double carLength = 4.5;
};

struct Sample {
    double time = 0.0;
    double position = 0.0;
    double speed = 0.0;
    double targetSpeed = 0.0;
};

// Crossing times are solved exactly within a step and judged against the lights
// themselves, so a crossing between two samples cannot slip past the check.
struct Crossing {
    std::size_t routeLight = 0;  // Index into Route::lights().
    double frontTime = 0.0;
    bool frontGreen = false;
    double rearTime = -1.0;  // Negative if the rear never crossed.
    bool rearGreen = false;
    double speed = 0.0;  // At the front's crossing.
};

struct SimulationResult {
    std::vector<Sample> samples;
    std::vector<Crossing> crossings;
    bool finished = false;  // The car reached the end of the route.
    double endTime = 0.0;
};

SimulationResult simulate(const Route &route, CarState car, const SimulationOptions &options,
                          const SpeedPolicy &policy);
