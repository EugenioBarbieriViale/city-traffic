#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "City.h"
#include "IntervalSet.h"

// A Green Light Optimal Speed Advisory planner. It works in the time-distance
// plane: red phases are walls and green phases are gates at each stop line.
// See plan() for the algorithm.

// A traffic light ahead of the car.
struct UpcomingLight {
    double distance = 0.0;  // From the car's front bumper to the stop line, metres.
    GreenSchedule schedule;
    // Seconds after each green starts before arrival is allowed. This is the
    // hook for a later queue model: a queue simply makes the gate open later.
    double gateOpenDelay = 0.0;
    // No arrival before this absolute time, however green the light is: the cars
    // queued ahead are still crossing. A queue longer than one green closes whole
    // windows this way, which a delay per window could not express.
    double notBefore = -std::numeric_limits<double>::infinity();
};

enum class CrossingRule {
    FrontAtStopLine,  // The front bumper crosses the line during green.
    // The rear bumper is past the line before green ends too. The end of each
    // gate moves earlier by carLength / vMin, the slowest planned crossing.
    WholeCarPastStopLine,
};

struct PlannerConfig {
    double vMin = 5.0;   // m/s
    double vMax = 13.9;  // m/s, about 50 km/h.
    double aMax = 2.0;   // m/s^2, for both speeding up and braking.
    double horizon = 500.0;  // Lights further along the route than this are ignored.

    double safetyMargin = 1.0;  // Shrinks each green window at both ends, seconds.
    // A light is retried with this smaller margin before being declared
    // impossible. A car about to reach a green line has nowhere else to go, and
    // the alternative is braking for a light it could legally cross.
    // Set equal to safetyMargin to turn the retry off.
    double minSafetyMargin = 0.25;

    // True: the first segment accounts for the ramp from the current speed to
    // the cruise speed, for both reachable times and the target speed.
    // False: speed changes are treated as instantaneous.
    // Reachable sets for later segments always use constant speed, as they are
    // replanned exactly once they become the first segment. When this is true the
    // choice among them also avoids chains that a_max cannot follow.
    bool accountForTransition = true;

    CrossingRule crossingRule = CrossingRule::FrontAtStopLine;
    double carLength = 4.5;  // Metres.

    double candidateStep = 0.5;  // Spacing of the arrival times compared, seconds.
    // Spacing of the first segment's cruise speeds compared, m/s. A fixed time
    // step gets ever coarser in speed as the car nears the line, which would make
    // the advised speed jitter between plans.
    double candidateSpeedStep = 0.25;

    // Throws std::invalid_argument if the values are inconsistent.
    void validate() const;
};

// One possible arrival, as seen by a PlanCost.
struct ArrivalOption {
    std::size_t lightIndex = 0;
    double now = 0.0;
    double time = 0.0;
    Interval gate;  // The usable green window holding `time`.
};

// Decides among feasible plans. A plan's total cost is the sum of arrival() over
// its lights and speedChange() between consecutive speeds, starting from the
// car's current speed. Lower is better. Implementations must be stateless.
class PlanCost {
public:
    virtual ~PlanCost() = default;
    virtual double arrival(const ArrivalOption &option) const = 0;
    virtual double speedChange(double vBefore, double vAfter) const = 0;
};

// The default: arrive mid-green for robustness, then keep the speed steady.
// Earlier green windows are preferred too, which keeps the car from crawling to a
// later one. That reward goes by the window, so it never moves an arrival inside one.
class RobustSmoothCost : public PlanCost {
public:
    struct Weights {
        double greenCenter = 1.0;  // For an arrival at the very edge of the gate.
        double speedChange = 0.5;  // For a change of the whole speed range.
        double time = 0.01;        // Per second until the window's middle.
    };

    explicit RobustSmoothCost(double speedRange) : RobustSmoothCost(speedRange, Weights{}) {}
    RobustSmoothCost(double speedRange, Weights weights);

    double arrival(const ArrivalOption &option) const override;
    double speedChange(double vBefore, double vAfter) const override;

private:
    double speedRange_;
    Weights weights_;
};

// Get through every light as soon as possible, whatever the speed changes.
class EarliestArrivalCost : public PlanCost {
public:
    double arrival(const ArrivalOption &option) const override;
    double speedChange(double, double) const override { return 0.0; }
};

struct PlanRequest {
    double now = 0.0;
    double speed = 0.0;  // Current speed.
    std::vector<UpcomingLight> lights;
};

enum class PlanStatus {
    NoLightsAhead,    // Nothing in range: cruise at vMax.
    StopFree,         // All lights crossed with the full safety margin.
    ReducedMargin,    // Stop-free, but only using minSafetyMargin somewhere.
    Fallback,         // Some light cannot be crossed without waiting.
};

struct LightPlan {
    double distance = 0.0;  // From the car, metres.
    // R_i: feasible arrival times. Empty when `fallback` is set, in which case
    // the search continues from the arrival time chosen here.
    IntervalSet reachable;
    double arrivalTime = 0.0;  // T_i
    double segmentSpeed = 0.0;  // v_i: cruise speed on the segment ending here.
    Interval gate;  // The usable green window chosen.
    double margin = 0.0;  // The safety margin in force at this light.
    bool fallback = false;
};

struct Plan {
    PlanStatus status = PlanStatus::NoLightsAhead;
    // Speed to apply now: the first segment's, capped by a braking curve to the
    // stop line of any light the plan has to wait at.
    double targetSpeed = 0.0;
    // The nearest light will be red when the car gets there, and a_max cannot
    // stop the car first. Needs a start too close to a red, or lights spaced
    // closer than the stopping distance from vMax (48 m by default).
    bool redCrossingUnavoidable = false;
    std::vector<LightPlan> lights;  // Nearest first.
};

class SpeedPlanner {
public:
    // A null cost selects RobustSmoothCost.
    explicit SpeedPlanner(const PlannerConfig &config, std::shared_ptr<const PlanCost> cost = {});

    const PlannerConfig &config() const { return config_; }

    // Plans arrival times at every light in range:
    //
    //  1. Forward: from R_0 = {now}, S_i = R_{i-1} + [dx/vMax, dx/vMin] gives the
    //     times the car can reach light i, and R_i = S_i n Green_i the ones it may
    //     cross at. If R_i is empty, the plan is to arrive as the next green
    //     starts, and propagation continues from that single time.
    //  2. Backward: B_i keeps the times in R_i that can still reach the last
    //     light, so each one has a valid predecessor and successor.
    //  3. Choice: the cheapest chain T_1 .. T_n through sampled times in the B_i,
    //     found by dynamic programming and backtracked from the last light.
    //     Samples include each B_i's lower end, and those always form a chain,
    //     so the choice cannot fail. Chains that a_max cannot follow are heavily
    //     penalized rather than excluded, for the same reason.
    Plan plan(const PlanRequest &request) const;

private:
    PlannerConfig config_;
    std::shared_ptr<const PlanCost> cost_;
};
