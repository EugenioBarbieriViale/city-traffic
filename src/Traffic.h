#pragma once

#include <cstddef>
#include <optional>
#include <random>
#include <vector>

#include "City.h"
#include "Route.h"
#include "Simulation.h"
#include "SpeedPlanner.h"
#include "Trip.h"

// Many cars on one city. Every car broadcasts its position and velocity, and hears
// the cars within radio range. Traffic hands a Car that inbox and nothing else, so
// a car never learns another car's route: which lane a car is in, and so whether
// it matters, is for the receiver to work out from where it is and where it goes.

struct TrafficConfig {
    int cars = 8;
    double radioRange = 300.0;  // Metres. Cars further apart do not hear each other.

    // The share of trips that end next to the city's center.
    double hotspotShare = 0.25;
    // Seconds a car stands in its lane at the end of a trip, blocking the cars behind.
    double dwellSeconds = 5.0;

    double brakeMax = 8.0;   // m/s^2. Emergency braking, beyond the planner's aMax.
    double minGap = 2.0;     // Metres kept to the car ahead when stopped.
    double reaction = 0.5;   // Seconds of headway kept on top of the stopping distance.

    double replanInterval = 0.2;  // Seconds between speed plans.
    double routeInterval = 1.0;   // Seconds between route searches.
    // How fast a street's remembered delay fades, seconds.
    double memoryHalfLife = 120.0;

    // The parts of the traffic model, which can be turned off to measure them.
    bool queueAwareSpeed = true;  // Speed plans allow for the queue ahead.
    bool rerouting = true;        // Routes allow for delays, and change.
    bool giveWay = true;          // Turns give way to the cars they could meet.
    bool keepCrossingsClear = true;  // No entering a crossing with no room beyond.

    void validate() const;
};

// All one car tells the others. Both refer to the front bumper.
struct CarMessage {
    Vec2 position;
    Vec2 velocity;
};

using Inbox = std::vector<CarMessage>;

// A car the receiver found to be ahead of it on its route.
struct CarAhead {
    double position = 0.0;  // Along the receiver's route.
    double speed = 0.0;
};

// A car ahead, as far as the queue model is concerned.
struct QueuedCar {
    double distance = 0.0;  // To the stop line.
    double speed = 0.0;
};

// When a car at `self` can cross behind the cars ahead of it, each of which needs
// its turn in a green window. `ahead` is ordered nearest the line first. Cars
// leave a standing queue one at a time, so the time between crossings is long for
// a car starting close to the line and short for one arriving at speed.
// Without any car ahead this is -infinity.
double queueClearTime(const GreenSchedule &schedule, const std::vector<QueuedCar> &ahead,
                      QueuedCar self, double now, const PlannerConfig &planner,
                      const TrafficConfig &traffic);

// The highest speed from which the car can stop behind a leader braking at
// `brake`, with `reaction` seconds to spare. `gap` is free road to its rear bumper.
double followingSpeed(double gap, double leaderSpeed, double brake, double reaction);

// How much later than an empty street a car expects each street to make it.
class StreetMemory {
public:
    StreetMemory(const City &city, double halfLife);

    void remember(int street, double delay, double time);
    // The delay last seen or heard of there, faded by how long ago that was.
    double expectedDelay(int street, double time) const;

private:
    struct Entry {
        double delay = 0.0;
        double time = 0.0;
    };

    std::vector<Entry> entries_;
    double halfLife_;
};

class Car {
public:
    Car(const City &city, const PlannerConfig &planner, const TrafficConfig &traffic,
        const Trip &trip, double time);

    // One time step. `inbox` is all the car learns about the other cars.
    void step(const Inbox &inbox, double time, double dt);

    CarMessage message() const;
    bool arrived() const { return state_.position >= route_->length(); }
    // Where the car is now, as the start of a new trip. It keeps what it remembers.
    StreetPoint location() const;
    void startTrip(const Trip &trip, double time);
    void park() { state_.speed = 0.0; }

    const Route &route() const { return *route_; }
    const CarState &state() const { return state_; }
    const Plan &plan() const { return controller_.lastPlan(); }
    const StreetMemory &memory() const { return memory_; }
    double tripStart() const { return tripStart_; }
    std::size_t carsAhead() const { return ahead_.size(); }
    int reroutes() const { return reroutes_; }

private:
    PathOptions pathOptions(double time, double notBefore) const;
    void readInbox(const Inbox &inbox);
    void hearStreets(const Inbox &inbox, double time);
    void noteStreet(double delay, double time);
    void reroute(const Inbox &inbox, double notBefore, double time);
    bool exitBlocked(double stopLine) const;
    bool carDueAt(const Inbox &inbox, Vec2 entry, Dir heading, double seconds,
                  double distance) const;
    bool mustGiveWay(const Inbox &inbox, std::size_t street) const;

    const City *city_;
    PlannerConfig planner_;
    TrafficConfig traffic_;
    GlosaController controller_;
    StreetMemory memory_;
    Trip trip_;
    std::optional<Route> route_;
    CarState state_;

    std::vector<CarAhead> ahead_;  // Nearest first.

    int currentStreet_ = -1;
    double streetDelay_ = 0.0;  // The worst seen during this visit to the street.
    double streetEntered_ = 0.0;
    double nextRouteTime_ = 0.0;
    double tripStart_ = 0.0;
    int reroutes_ = 0;
};

struct TrafficStats {
    int tripsFinished = 0;
    double tripSeconds = 0.0;    // Over the finished trips.
    double distance = 0.0;       // Metres driven by all cars.
    int stops = 0;               // A car came to a halt, other than at a trip's start.
    int redCrossings = 0;
    int overlaps = 0;            // Times one car ran into the rear of another.
    double deepestOverlap = 0.0;  // Metres.
    int reroutes = 0;
};

class Traffic {
public:
    // Throws std::invalid_argument if the cars do not fit on the city's streets.
    Traffic(const City &city, const PlannerConfig &planner, const TrafficConfig &config,
            std::mt19937 &rng);

    void step(double dt);

    double time() const { return time_; }
    const std::vector<Car> &cars() const { return cars_; }
    const TrafficStats &stats() const { return stats_; }
    // What car `index` heard on the last step.
    const Inbox &inbox(std::size_t index) const { return inboxes_[index]; }

private:
    void deliverMessages();
    void countOverlaps();

    const City *city_;
    PlannerConfig planner_;
    TrafficConfig config_;
    std::mt19937 *rng_;
    std::vector<Car> cars_;
    std::vector<Inbox> inboxes_;  // By car.
    std::vector<char> touching_;  // By car: rammed the one ahead on the last step.
    std::vector<double> parkedUntil_;  // By car. In the past for a car that is driving.
    double time_ = 0.0;
    TrafficStats stats_;
};

// A number of cars that keeps a city busy without filling it, whatever its size.
int busyCarCount(const City &city);
