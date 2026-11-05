#include "SpeedPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "Kinematics.h"

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
// Absorbs rounding when checking that one chosen time follows from another.
constexpr double kTimeTolerance = 1e-9;
// Added for a segment a_max cannot follow. Dominates any real cost.
constexpr double kUnfollowablePenalty = 1e6;

// Everything known about one light once the forward pass has run.
struct Layer {
    const UpcomingLight *light = nullptr;
    double segment = 0.0;  // Distance from the previous light, or from the car.
    double distance = 0.0;  // From the car.
    IntervalSet gates;      // Green_i, at `margin`.
    IntervalSet reachable;  // R_i
    double margin = 0.0;
    bool fallback = false;
    IntervalSet arrivals;  // The times to continue from: R_i, or the fallback time.
};

struct Candidate {
    double time = 0.0;
    Interval gate;
    double cost = kInfinity;
    double speedIn = 0.0;
    int parent = -1;
};

double endOfGateTrim(const PlannerConfig &config) {
    if (config.crossingRule == CrossingRule::FrontAtStopLine) return 0.0;
    return config.carLength / config.vMin;
}

// The usable windows of a light over [from, to]. Whole green windows are shrunk,
// so a window already open at `from` still loses its margin at its true ends.
IntervalSet gateWindows(const UpcomingLight &light, double margin, double endTrim, double from,
                        double to) {
    const double startTrim = margin + light.gateOpenDelay;
    const IntervalSet greens = light.schedule.windows(from, to);
    std::vector<Interval> gates;
    for (const Interval &green : greens.intervals()) {
        gates.push_back({std::max(green.lo + startTrim, light.notBefore),
                         green.hi - margin - endTrim});
    }
    return IntervalSet(std::move(gates));
}

// The earliest time at or after t that a gate opens.
double nextGateStart(const UpcomingLight &light, double margin, double endTrim, double t) {
    const double cycle = light.schedule.cycle;
    t = std::max(t, light.notBefore);
    IntervalSet gates = gateWindows(light, margin, endTrim, t, t + 2.0 * cycle);
    for (const Interval &gate : gates.intervals()) {
        if (gate.lo >= t) return gate.lo;
    }
    throw std::invalid_argument("SpeedPlanner: a light's gate never opens");
}

Interval firstLegTimes(const PlannerConfig &config, double distance, double speed) {
    if (!config.accountForTransition) return {distance / config.vMax, distance / config.vMin};
    double earliest = travelTime(distance, speed, config.vMax, config.aMax);
    double latest = travelTime(distance, speed, config.vMin, config.aMax);
    return {earliest, std::max(earliest, latest)};
}

std::vector<Layer> propagateForward(const PlannerConfig &config, const PlanRequest &request,
                                    const std::vector<UpcomingLight> &lights) {
    const double endTrim = endOfGateTrim(config);
    std::vector<Layer> layers;
    IntervalSet previous = IntervalSet::point(request.now);
    double previousDistance = 0.0;

    for (const UpcomingLight &light : lights) {
        Layer layer;
        layer.light = &light;
        layer.distance = light.distance;
        layer.segment = light.distance - previousDistance;

        Interval leg = layers.empty()
                           ? firstLegTimes(config, layer.segment, request.speed)
                           : Interval{layer.segment / config.vMax, layer.segment / config.vMin};
        IntervalSet shifted = previous.shift(leg);

        for (double margin : {config.safetyMargin, config.minSafetyMargin}) {
            layer.margin = margin;
            layer.gates = gateWindows(light, margin, endTrim, shifted.min(), shifted.max());
            layer.reachable = shifted.intersect(layer.gates);
            if (!layer.reachable.empty()) break;
        }

        if (layer.reachable.empty()) {
            layer.margin = config.safetyMargin;
            layer.fallback = true;
            double arrival = nextGateStart(light, layer.margin, endTrim, shifted.min());
            layer.gates = gateWindows(light, layer.margin, endTrim, arrival, arrival);
            layer.arrivals = IntervalSet::point(arrival);
        } else {
            layer.arrivals = layer.reachable;
        }

        previous = layer.arrivals;
        previousDistance = light.distance;
        layers.push_back(std::move(layer));
    }
    return layers;
}

// B_i: the times at each light from which the last light is still reachable.
std::vector<IntervalSet> propagateBackward(const PlannerConfig &config,
                                           const std::vector<Layer> &layers) {
    std::vector<IntervalSet> usable(layers.size());
    usable.back() = layers.back().arrivals;
    for (std::size_t i = layers.size() - 1; i-- > 0;) {
        const Layer &next = layers[i + 1];
        const double fastest = next.segment / config.vMax;
        const double slowest = next.segment / config.vMin;
        if (next.fallback) {
            // The segment into a fallback has no lower speed bound.
            usable[i] = layers[i].arrivals.intersect(
                Interval{-kInfinity, usable[i + 1].min() - fastest + kTimeTolerance});
        } else {
            usable[i] = layers[i].arrivals.intersect(usable[i + 1].shift(
                -slowest - kTimeTolerance, -fastest + kTimeTolerance));
        }
    }
    return usable;
}

std::vector<double> sampleTimes(const PlannerConfig &config, const PlanRequest &request,
                                const Layer &layer, const IntervalSet &usable, bool firstLight) {
    std::vector<double> times;
    for (const Interval &interval : usable.intervals()) {
        times.push_back(interval.lo);
        times.push_back(interval.hi);
        if (auto gate = layer.gates.intervalContaining(interval.mid())) {
            times.push_back(std::clamp(gate->mid(), interval.lo, interval.hi));
        }
        // On a grid of absolute times, so that repeated plans sample the same
        // times and arrivals at in-phase lights line up.
        const double step = config.candidateStep;
        for (double k = std::ceil(interval.lo / step); k * step <= interval.hi; k += 1.0) {
            times.push_back(k * step);
        }
    }
    // Carrying on at the current speed, which keeps replanning from drifting.
    if (request.speed > 0.0) {
        double steady = request.now + layer.distance / request.speed;
        if (usable.contains(steady)) times.push_back(steady);
    }
    if (firstLight) {
        const double aMax = config.accountForTransition
                                ? config.aMax
                                : std::numeric_limits<double>::infinity();
        for (double v = config.vMin; v <= config.vMax; v += config.candidateSpeedStep) {
            double time = request.now + travelTime(layer.segment, request.speed, v, aMax);
            if (usable.contains(time)) times.push_back(time);
        }
    }
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
    return times;
}

// Whether a car entering a segment at `speedIn` can take `duration` over it.
bool canFollow(const PlannerConfig &config, const Layer &layer, double speedIn, double duration) {
    if (!config.accountForTransition) return true;
    if (duration < travelTime(layer.segment, speedIn, config.vMax, config.aMax)) return false;
    // Waiting out a red has no lower speed bound, so only stopping in time matters.
    const double slowest = layer.fallback ? 0.0 : config.vMin;
    return duration <= travelTime(layer.segment, speedIn, slowest, config.aMax);
}

double firstSegmentSpeed(const PlannerConfig &config, const PlanRequest &request,
                         const Layer &layer, double arrival) {
    const double duration = arrival - request.now;
    // Below vMin is only allowed when the plan is to wait out a red.
    const double lowest = layer.fallback ? 0.0 : config.vMin;
    if (!config.accountForTransition) {
        if (duration <= 0.0) return config.vMax;
        return std::clamp(layer.segment / duration, lowest, config.vMax);
    }
    return cruiseSpeedForDuration(layer.segment, duration, request.speed, lowest, config.vMax,
                                  config.aMax);
}

// Dynamic programming over the sampled times. The cost of a speed change depends
// on the two segments around a light, and only the best path into each candidate
// is kept, so the optimum is over chains of that shape rather than exhaustive.
std::vector<double> chooseArrivals(const PlannerConfig &config, const PlanCost &cost,
                                   const PlanRequest &request, const std::vector<Layer> &layers,
                                   const std::vector<IntervalSet> &usable) {
    std::vector<std::vector<Candidate>> grid(layers.size());

    for (std::size_t i = 0; i < layers.size(); i++) {
        const Layer &layer = layers[i];
        for (double time : sampleTimes(config, request, layer, usable[i], i == 0)) {
            Candidate c;
            c.time = time;
            auto gate = layer.gates.intervalContaining(time);
            c.gate = gate ? *gate : Interval{time, time};
            const double arrivalCost = cost.arrival({i, request.now, time, c.gate});

            if (i == 0) {
                c.speedIn = firstSegmentSpeed(config, request, layer, time);
                c.cost = arrivalCost + cost.speedChange(request.speed, c.speedIn);
            } else {
                for (std::size_t p = 0; p < grid[i - 1].size(); p++) {
                    const Candidate &previous = grid[i - 1][p];
                    const double duration = time - previous.time;
                    if (duration <= 0.0) continue;
                    const double fastest = layer.segment / config.vMax;
                    const double slowest =
                        layer.fallback ? kInfinity : layer.segment / config.vMin;
                    if (duration < fastest - 2.0 * kTimeTolerance) continue;
                    if (duration > slowest + 2.0 * kTimeTolerance) continue;

                    const double speed = layer.segment / duration;
                    double total = previous.cost + arrivalCost +
                                   cost.speedChange(previous.speedIn, speed);
                    if (!canFollow(config, layer, previous.speedIn, duration)) {
                        total += kUnfollowablePenalty;
                    }
                    if (total < c.cost) {
                        c.cost = total;
                        c.speedIn = speed;
                        c.parent = static_cast<int>(p);
                    }
                }
            }
            grid[i].push_back(c);
        }
    }

    const std::vector<Candidate> &last = grid.back();
    auto best = std::min_element(last.begin(), last.end(), [](const auto &a, const auto &b) {
        return a.cost < b.cost;
    });
    if (best == last.end() || !std::isfinite(best->cost)) {
        throw std::logic_error("SpeedPlanner: no chain through the feasible arrival times");
    }

    std::vector<double> arrivals(layers.size());
    int index = static_cast<int>(best - last.begin());
    for (std::size_t i = layers.size(); i-- > 0;) {
        const Candidate &c = grid[i][static_cast<std::size_t>(index)];
        arrivals[i] = c.time;
        index = c.parent;
    }
    return arrivals;
}

}  // namespace

void PlannerConfig::validate() const {
    if (!(vMin > 0.0) || !(vMax >= vMin)) throw std::invalid_argument("need 0 < vMin <= vMax");
    if (!(aMax > 0.0)) throw std::invalid_argument("need aMax > 0");
    if (!(horizon > 0.0)) throw std::invalid_argument("need horizon > 0");
    if (!(safetyMargin >= minSafetyMargin) || !(minSafetyMargin >= 0.0)) {
        throw std::invalid_argument("need safetyMargin >= minSafetyMargin >= 0");
    }
    if (!(carLength >= 0.0)) throw std::invalid_argument("need carLength >= 0");
    if (!(candidateStep > 0.0) || !(candidateSpeedStep > 0.0)) {
        throw std::invalid_argument("need candidate steps > 0");
    }
}

RobustSmoothCost::RobustSmoothCost(double speedRange, Weights weights)
    : speedRange_(speedRange > 0.0 ? speedRange : 1.0), weights_(weights) {}

double RobustSmoothCost::arrival(const ArrivalOption &option) const {
    const double half = option.gate.width() / 2.0;
    const double offCenter = half > 0.0 ? (option.time - option.gate.mid()) / half : 0.0;
    return weights_.greenCenter * offCenter * offCenter +
           weights_.time * (option.gate.mid() - option.now);
}

double RobustSmoothCost::speedChange(double vBefore, double vAfter) const {
    const double change = (vAfter - vBefore) / speedRange_;
    return weights_.speedChange * change * change;
}

double EarliestArrivalCost::arrival(const ArrivalOption &option) const {
    return option.time - option.now;
}

SpeedPlanner::SpeedPlanner(const PlannerConfig &config, std::shared_ptr<const PlanCost> cost)
    : config_(config), cost_(std::move(cost)) {
    config_.validate();
    if (!cost_) cost_ = std::make_shared<RobustSmoothCost>(config_.vMax - config_.vMin);
}

Plan SpeedPlanner::plan(const PlanRequest &request) const {
    std::vector<UpcomingLight> lights;
    for (const UpcomingLight &light : request.lights) {
        if (light.distance >= 0.0 && light.distance <= config_.horizon) lights.push_back(light);
    }
    std::sort(lights.begin(), lights.end(), [](const auto &a, const auto &b) {
        return a.distance < b.distance;
    });

    Plan plan;
    if (lights.empty()) {
        plan.targetSpeed = config_.vMax;
        return plan;
    }

    std::vector<Layer> layers = propagateForward(config_, request, lights);
    std::vector<IntervalSet> usable = propagateBackward(config_, layers);
    std::vector<double> arrivals = chooseArrivals(config_, *cost_, request, layers, usable);

    plan.status = PlanStatus::StopFree;
    for (std::size_t i = 0; i < layers.size(); i++) {
        const Layer &layer = layers[i];
        LightPlan light;
        light.distance = layer.distance;
        light.reachable = layer.reachable;
        light.arrivalTime = arrivals[i];
        light.segmentSpeed = i == 0 ? firstSegmentSpeed(config_, request, layer, arrivals[i])
                                    : layer.segment / (arrivals[i] - arrivals[i - 1]);
        auto gate = layer.gates.intervalContaining(arrivals[i]);
        light.gate = gate ? *gate : Interval{arrivals[i], arrivals[i]};
        light.margin = layer.margin;
        light.fallback = layer.fallback;
        plan.lights.push_back(std::move(light));

        if (layer.fallback) {
            plan.status = PlanStatus::Fallback;
        } else if (layer.margin < config_.safetyMargin && plan.status == PlanStatus::StopFree) {
            plan.status = PlanStatus::ReducedMargin;
        }
    }
    plan.targetSpeed = plan.lights.front().segmentSpeed;
    if (std::isfinite(config_.aMax)) {
        for (const Layer &layer : layers) {
            if (!layer.fallback) continue;
            const double brakingLimit = std::sqrt(2.0 * config_.aMax * layer.distance);
            plan.targetSpeed = std::min(plan.targetSpeed, brakingLimit);
        }
    }

    if (layers.front().fallback) {
        const double stoppingTime = travelTime(layers.front().distance, request.speed, 0.0,
                                               config_.aMax);
        plan.redCrossingUnavoidable =
            std::isfinite(stoppingTime) &&
            !layers.front().light->schedule.isGreen(request.now + stoppingTime);
    }
    return plan;
}
