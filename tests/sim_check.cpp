// Drives cars along routes through the full receding-horizon loop and judges each
// crossing against the lights themselves. Writes the main run's log to
// sim_trace.csv in the working directory. Exits non-zero on any violation.
//
// Usage: sim_check [stress runs per configuration, default 6]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "City.h"
#include "Route.h"
#include "Simulation.h"
#include "SpeedPlanner.h"

namespace {

int gFailures = 0;

void require(bool condition, const std::string &message) {
    if (condition) return;
    std::printf("  VIOLATION: %s\n", message.c_str());
    gFailures++;
}

const char *statusName(PlanStatus status) {
    switch (status) {
        case PlanStatus::NoLightsAhead: return "no-lights";
        case PlanStatus::StopFree: return "stop-free";
        case PlanStatus::ReducedMargin: return "reduced-margin";
        case PlanStatus::Fallback: return "fallback";
    }
    return "?";
}

// One simulated drive, with the planner's status recorded for every step.
struct Drive {
    SimulationResult result;
    std::vector<PlanStatus> statuses;
    bool everUnavoidable = false;
};

Drive drive(const Route &route, CarState start, const PlannerConfig &planner,
            const SimulationOptions &options, double replanInterval,
            std::shared_ptr<const PlanCost> cost = {}) {
    Drive d;
    GlosaController controller(planner, replanInterval, std::move(cost));
    d.result = simulate(route, start, options,
                        [&](const Route &r, const CarState &car, double time) {
                            double target = controller.targetSpeed(r, car, time);
                            d.statuses.push_back(controller.lastPlan().status);
                            d.everUnavoidable |= controller.lastPlan().redCrossingUnavoidable;
                            return target;
                        });
    return d;
}

// The universal rules of a drive. Anything broken is reported with `label`.
void checkDrive(const Drive &d, const Route &route, const PlannerConfig &planner,
                const SimulationOptions &options, const std::string &label) {
    const SimulationResult &r = d.result;
    require(r.finished, label + ": did not reach the end of the route");
    require(r.crossings.size() == route.lights().size(), label + ": missed a crossing");

    for (const Crossing &c : r.crossings) {
        require(c.frontGreen, label + ": front crossed on red at t=" + std::to_string(c.frontTime));
        if (planner.crossingRule == CrossingRule::WholeCarPastStopLine) {
            require(c.rearTime >= 0.0 && c.rearGreen,
                    label + ": rear crossed on red at t=" + std::to_string(c.rearTime));
        }
    }

    for (std::size_t i = 0; i + 1 < r.samples.size(); i++) {
        double accel = (r.samples[i + 1].speed - r.samples[i].speed) / options.dt;
        require(std::abs(accel) <= planner.aMax + 1e-9,
                label + ": a_max exceeded at t=" + std::to_string(r.samples[i].time));
        require(r.samples[i].speed <= planner.vMax + 1e-9, label + ": v_max exceeded");
        require(r.samples[i].speed >= 0.0, label + ": reversing");
    }
}

// Counts dithering: a large change of advice undone within a few seconds. Passing
// a light starts a new segment, where a jump is expected. The last metres before
// a line are also left out: the time left is too short for a_max to act on them.
int adviceReversals(const SimulationResult &r, const Route &route) {
    const double kLargeChange = 1.0;
    const double kWindow = 5.0;
    const double kNearLine = 20.0;

    int reversals = 0;
    double lastChange = 0.0;
    double lastTime = -kWindow;
    for (std::size_t i = 0; i + 1 < r.samples.size(); i++) {
        const std::size_t next = route.nextLightIndex(r.samples[i].position);
        if (next != route.nextLightIndex(r.samples[i + 1].position)) continue;
        if (next < route.lights().size() &&
            route.lights()[next].stopLine - r.samples[i].position < kNearLine) {
            continue;
        }
        const double change = r.samples[i + 1].targetSpeed - r.samples[i].targetSpeed;
        if (std::abs(change) < kLargeChange) continue;
        if (change * lastChange < 0.0 && r.samples[i].time - lastTime < kWindow) reversals++;
        lastChange = change;
        lastTime = r.samples[i].time;
    }
    return reversals;
}

// Seconds between a crossing and the nearest change of its light.
double clearance(const City &city, const Route &route, const Crossing &c) {
    const RouteLight &rl = route.lights()[c.routeLight];
    GreenSchedule s = city.light(rl.lightIndex).schedule(rl.axis);
    double untilRed = s.timeToNextChange(c.frontTime);
    return std::min(untilRed, s.greenDuration - untilRed);
}

// Whether the car stopped at any point after first moving.
bool everStopped(const SimulationResult &r) {
    bool moving = false;
    for (const Sample &s : r.samples) {
        if (s.speed > 1.0) moving = true;
        if (moving && s.speed < 0.1) return true;
    }
    return false;
}

void mainRun() {
    std::printf("== Main run: snake through the default 4x3 city ==\n");
    City city;
    Route route(city, {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}, {2, 1}, {1, 1}, {0, 1}, {0, 2},
                       {1, 2}, {2, 2}, {3, 2}});
    PlannerConfig planner;
    SimulationOptions options;
    Drive d = drive(route, {0.0, 0.0}, planner, options, 0.0);
    checkDrive(d, route, planner, options, "main");
    require(!everStopped(d.result), "main: the car stopped");
    const int reversals = adviceReversals(d.result, route);
    require(reversals == 0, "main: the advice dithered");

    std::FILE *trace = std::fopen("sim_trace.csv", "w");
    require(trace != nullptr, "could not write sim_trace.csv");
    if (trace) {
        std::fprintf(trace, "time,position,speed,target,status\n");
        for (std::size_t i = 0; i < d.result.samples.size(); i++) {
            const Sample &s = d.result.samples[i];
            std::fprintf(trace, "%.2f,%.3f,%.3f,%.3f,%s\n", s.time, s.position, s.speed,
                         s.targetSpeed, statusName(d.statuses[i]));
        }
        std::fclose(trace);
    }

    std::printf("%8s %10s %7s  %-7s %-5s %8s %8s\n", "time[s]", "pos[m]", "v[m/s]", "light", "state",
                "greenFor", "untilRed");
    for (const Crossing &c : d.result.crossings) {
        const RouteLight &rl = route.lights()[c.routeLight];
        const TrafficLight &light = city.light(rl.lightIndex);
        GreenSchedule s = light.schedule(rl.axis);
        double untilRed = s.timeToNextChange(c.frontTime);
        std::printf("%8.2f %10.1f %7.2f  (%d,%d)%-2s %-5s %7.2fs %7.2fs\n", c.frontTime,
                    rl.stopLine, c.speed, light.node.col, light.node.row,
                    rl.axis == Axis::EastWest ? "EW" : "NS", c.frontGreen ? "GREEN" : "RED",
                    s.greenDuration - untilRed, untilRed);
    }
    double minSpeed = 1e9;
    double t = 0.0;
    for (const Sample &sample : d.result.samples) {
        if (sample.time > 8.0) minSpeed = std::min(minSpeed, sample.speed);
        t = sample.time;
    }
    std::printf("finished in %.1f s; lowest speed after the launch %.2f m/s; advice reversals "
                "%d; log: sim_trace.csv\n\n",
                t, minSpeed, reversals);
}

void controlRun() {
    std::printf("== Control: the same route with no planner must run reds ==\n");
    City city;
    Route route(city, {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}, {2, 1}, {1, 1}, {0, 1}, {0, 2},
                       {1, 2}, {2, 2}, {3, 2}});
    SimulationOptions options;
    SimulationResult r = simulate(route, {0.0, 0.0}, options,
                                  [](const Route &, const CarState &, double) { return 13.9; });
    int reds = 0;
    for (const Crossing &c : r.crossings) reds += c.frontGreen ? 0 : 1;
    std::printf("constant 13.9 m/s: %d of %zu crossings on red\n\n", reds, r.crossings.size());
    require(reds > 0, "control: the red-light check caught nothing");
}

void greenWaveRun() {
    std::printf("== Green wave: all lights in phase, 200 m apart ==\n");
    CityConfig config;
    config.cols = 6;
    config.rows = 3;
    City city(config);
    // A straight run along the middle street, where every node is a crossing.
    std::vector<Node> nodes;
    for (int col = 0; col < config.cols; col++) nodes.push_back({col, 1});
    for (const TrafficLight &light : city.lights()) city.setLightOffset(light.node, 0.0);
    Route route(city, nodes);

    PlannerConfig planner;
    SimulationOptions options;

    const auto crossingSpeeds = [](const Drive &d) {
        std::vector<double> speeds;
        for (const Crossing &c : d.result.crossings) speeds.push_back(c.speed);
        return speeds;
    };
    const auto spread = [](const std::vector<double> &v, std::size_t from) {
        auto range = std::minmax_element(v.begin() + static_cast<std::ptrdiff_t>(from), v.end());
        return *range.second - *range.first;
    };

    // Starting in step with the wave, 10 m/s meets every light mid-green.
    const double firstLine = route.lights()[0].stopLine;
    Drive inStep = drive(route, {firstLine - 150.0, 10.0}, planner, options, 0.0);
    checkDrive(inStep, route, planner, options, "green wave, in step");
    std::vector<double> speeds = crossingSpeeds(inStep);
    std::printf("in step: speed at crossings spans %.4f m/s\n", spread(speeds, 0));
    require(spread(speeds, 0) < 0.01, "green wave: an in-step car changed speed");

    // Out of step, the default cost moves toward mid-green over the first segments.
    // The speed must then converge on the wave's 10 m/s.
    Drive offStep = drive(route, {0.0, 10.0}, planner, options, 0.0);
    checkDrive(offStep, route, planner, options, "green wave, off step");
    speeds = crossingSpeeds(offStep);
    std::printf("off step: crossings at");
    for (double speed : speeds) std::printf(" %.2f", speed);
    std::printf(" m/s\n\n");
    require(speeds.size() >= 4 && spread(speeds, 2) < 0.05, "green wave: speed did not settle");
    require(std::abs(speeds.back() - 10.0) < 0.01, "green wave: settled on the wrong speed");
}

Route randomRoute(const City &city, std::mt19937 &rng, int streets) {
    std::vector<Node> nodes = {{0, 0}, {1, 0}};
    while (static_cast<int>(nodes.size()) <= streets) {
        Node end = nodes.back();
        Dir heading = headingBetween(nodes[nodes.size() - 2], end);
        std::vector<Node> options;
        for (int i = 0; i < DirCount; i++) {
            Dir d = static_cast<Dir>(i);
            if (d != opposite(heading) && city.nodeExists(neighbor(end, d))) {
                options.push_back(neighbor(end, d));
            }
        }
        nodes.push_back(options[std::uniform_int_distribution<size_t>(0, options.size() - 1)(rng)]);
    }
    return Route(city, nodes);
}

// Random cities, routes and offsets under every combination of the knobs that
// could plausibly break the loop.
void stressRun(int runs) {
    std::printf("== Stress: random offsets and routes ==\n");
    std::printf("%6s %6s %6s %-6s %-8s | %5s %9s %5s %8s %9s %9s %9s\n", "block", "dt", "replan",
                "rule", "cost", "runs", "crossings", "red", "stopped", "fallback%", "closest",
                "reversals");

    std::mt19937 rng(20261104);
    std::uniform_real_distribution<double> anyOffset(0.0, 20.0);
    std::uniform_real_distribution<double> anySpeed(0.0, 13.9);

    for (double block : {200.0, 100.0, 60.0}) {
        for (double dt : {0.05, 0.2}) {
            for (double replan : {0.0, 1.0}) {
                for (bool wholeCar : {false, true}) {
                    for (bool earliest : {false, true}) {
                        CityConfig cityConfig;
                        cityConfig.cols = 8;
                        cityConfig.rows = 6;
                        cityConfig.blockLength = block;
                        cityConfig.roadWidth = 12.0;

                        PlannerConfig planner;
                        if (wholeCar) planner.crossingRule = CrossingRule::WholeCarPastStopLine;
                        std::shared_ptr<const PlanCost> cost;
                        if (earliest) cost = std::make_shared<EarliestArrivalCost>();

                        SimulationOptions options;
                        options.dt = dt;
                        options.aMax = planner.aMax;
                        options.carLength = planner.carLength;

                        int crossings = 0;
                        int reds = 0;
                        int stopped = 0;
                        std::size_t fallbackSteps = 0;
                        std::size_t steps = 0;
                        double closest = 1e9;
                        int reversals = 0;
                        const int failuresBefore = gFailures;

                        for (int run = 0; run < runs; run++) {
                            City city(cityConfig);
                            for (const TrafficLight &light : city.lights()) {
                                city.setLightOffset(light.node, anyOffset(rng));
                            }
                            Route route = randomRoute(city, rng, 14);
                            Drive d = drive(route, {0.0, anySpeed(rng)}, planner, options, replan,
                                            cost);
                            std::string label = "block " + std::to_string(block) + " run " +
                                                std::to_string(run);
                            if (gFailures - failuresBefore < 8) {
                                checkDrive(d, route, planner, options, label);
                            }
                            crossings += static_cast<int>(d.result.crossings.size());
                            for (const Crossing &c : d.result.crossings) {
                                reds += c.frontGreen ? 0 : 1;
                                closest = std::min(closest, clearance(city, route, c));
                            }
                            reversals += adviceReversals(d.result, route);
                            stopped += everStopped(d.result) ? 1 : 0;
                            for (PlanStatus s : d.statuses) {
                                fallbackSteps += s == PlanStatus::Fallback ? 1 : 0;
                            }
                            steps += d.statuses.size();
                        }
                        require(reds == 0, "stress: red crossings");
                        std::printf(
                            "%6.0f %6.2f %6.1f %-6s %-8s | %5d %9d %5d %8d %8.1f%% %8.2fs %9d\n",
                            block, dt, replan, wholeCar ? "whole" : "front",
                            earliest ? "earliest" : "default", runs, crossings, reds, stopped,
                            100.0 * fallbackSteps / steps, closest, reversals);
                    }
                }
            }
        }
    }
    std::printf("\n");
}

// Lights closer together than the stopping distance from vMax are outside what a
// single a_max can guarantee. Reds can happen there, but never unannounced.
void shortBlockRun(int runs) {
    std::printf("== Short blocks: below the stopping distance, every red must be flagged ==\n");
    std::mt19937 rng(20261105);
    std::uniform_real_distribution<double> anyOffset(0.0, 20.0);

    CityConfig cityConfig;
    cityConfig.cols = 8;
    cityConfig.rows = 6;
    cityConfig.blockLength = 40.0;
    cityConfig.roadWidth = 12.0;
    PlannerConfig planner;
    SimulationOptions options;

    int crossings = 0;
    int reds = 0;
    int silent = 0;
    for (int run = 0; run < runs * 10; run++) {
        City city(cityConfig);
        for (const TrafficLight &light : city.lights()) {
            city.setLightOffset(light.node, anyOffset(rng));
        }
        Route route = randomRoute(city, rng, 14);
        Drive d = drive(route, {0.0, 0.0}, planner, options, 0.0);
        int redsHere = 0;
        for (const Crossing &c : d.result.crossings) redsHere += c.frontGreen ? 0 : 1;
        crossings += static_cast<int>(d.result.crossings.size());
        reds += redsHere;
        if (redsHere > 0 && !d.everUnavoidable) silent++;
    }
    std::printf("40 m blocks: %d of %d crossings on red, %d drive(s) with an unflagged red\n\n",
                reds, crossings, silent);
    require(silent == 0, "short blocks: a red crossing was not flagged");
}

}  // namespace

int main(int argc, char **argv) {
    const int runs = argc > 1 ? std::stoi(argv[1]) : 6;
    mainRun();
    controlRun();
    greenWaveRun();
    stressRun(runs);
    shortBlockRun(runs);
    std::printf("%s: %d violation(s)\n", gFailures == 0 ? "PASS" : "FAIL", gFailures);
    return gFailures == 0 ? 0 : 1;
}
