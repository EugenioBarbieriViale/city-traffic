#include "Simulation.h"

#include <algorithm>
#include <cmath>

void stepCar(CarState &car, double targetSpeed, double aMax, double dt) {
    const double change = std::clamp(targetSpeed - car.speed, -aMax * dt, aMax * dt);
    const double newSpeed = std::max(car.speed + change, 0.0);
    car.position += (car.speed + newSpeed) / 2.0 * dt;
    car.speed = newSpeed;
}

void stepCar(CarState &car, double targetSpeed, double speedLimit, double aMax, double brakeMax,
             double dt) {
    double acceleration = std::clamp((targetSpeed - car.speed) / dt, -aMax, aMax);
    acceleration = std::max(std::min(acceleration, (speedLimit - car.speed) / dt), -brakeMax);
    const double newSpeed = std::max(car.speed + acceleration * dt, 0.0);
    car.position += (car.speed + newSpeed) / 2.0 * dt;
    car.speed = newSpeed;
}

double arrivalSpeedLimit(const Route &route, const CarState &car, double aMax) {
    // The slack keeps the speed above zero at the end itself. Without it the
    // car would creep toward the end ever more slowly and never get there.
    constexpr double kSlack = 0.1;
    const double remaining = std::max(route.length() - car.position, 0.0) + kSlack;
    return std::sqrt(2.0 * kArrivalBrakingShare * aMax * remaining);
}

GlosaController::GlosaController(const PlannerConfig &config, double replanInterval,
                                 std::shared_ptr<const PlanCost> cost)
    : planner_(config, std::move(cost)), replanInterval_(replanInterval) {}

double GlosaController::targetSpeed(const Route &route, const CarState &car, double time,
                                    double notBefore) {
    const std::size_t nextLight = route.nextLightIndex(car.position);
    const bool due = !hasPlan_ || time - lastPlanTime_ >= replanInterval_;
    if (due || nextLight != lastNextLight_) {
        PlanRequest request;
        request.now = time;
        request.speed = car.speed;
        request.lights = route.lightsAhead(car.position, planner_.config().horizon);
        if (!request.lights.empty()) request.lights.front().notBefore = notBefore;
        plan_ = planner_.plan(request);
        hasPlan_ = true;
        lastPlanTime_ = time;
        lastNextLight_ = nextLight;
    }
    return plan_.targetSpeed;
}

namespace {

// Seconds into a step until the position has advanced by `distance`, given
// constant acceleration from `speed`.
double timeToCover(double distance, double speed, double acceleration) {
    if (std::abs(acceleration) < 1e-12) return speed > 0.0 ? distance / speed : 0.0;
    const double root = std::sqrt(std::max(speed * speed + 2.0 * acceleration * distance, 0.0));
    return (root - speed) / acceleration;
}

}  // namespace

SimulationResult simulate(const Route &route, CarState car, const SimulationOptions &options,
                          const SpeedPolicy &policy) {
    SimulationResult result;
    const City &city = route.city();
    const std::vector<RouteLight> &lights = route.lights();
    std::size_t nextFront = route.nextLightIndex(car.position);
    std::size_t nextRear = nextFront;

    const auto greenAt = [&](const RouteLight &light, double t) {
        return city.light(light.lightIndex).isGreen(light.axis, t);
    };

    for (double time = 0.0; time < options.maxTime; time += options.dt) {
        const double target = policy(route, car, time);
        result.samples.push_back({time, car.position, car.speed, target});

        const CarState before = car;
        stepCar(car, target, options.aMax, options.dt);
        const double acceleration = (car.speed - before.speed) / options.dt;

        // A car resting exactly on a line has not crossed it yet.
        while (nextFront < lights.size() && lights[nextFront].stopLine < car.position) {
            const RouteLight &light = lights[nextFront];
            const double tau = timeToCover(light.stopLine - before.position, before.speed,
                                           acceleration);
            Crossing crossing;
            crossing.routeLight = nextFront;
            crossing.frontTime = time + tau;
            crossing.frontGreen = greenAt(light, crossing.frontTime);
            crossing.speed = before.speed + acceleration * tau;
            result.crossings.push_back(crossing);
            nextFront++;
        }

        const double rear = car.position - options.carLength;
        while (nextRear < nextFront && lights[nextRear].stopLine < rear) {
            const RouteLight &light = lights[nextRear];
            const double rearBefore = before.position - options.carLength;
            const double tau = timeToCover(light.stopLine - rearBefore, before.speed, acceleration);
            for (Crossing &crossing : result.crossings) {
                if (crossing.routeLight != nextRear) continue;
                crossing.rearTime = time + tau;
                crossing.rearGreen = greenAt(light, crossing.rearTime);
            }
            nextRear++;
        }

        result.endTime = time + options.dt;
        if (car.position >= route.length()) {
            result.finished = true;
            break;
        }
    }
    return result;
}
