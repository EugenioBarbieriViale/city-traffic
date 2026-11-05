#include "Traffic.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

#include "Kinematics.h"

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Least distance between two cars placed on one street at the start.
constexpr double kPlacementGap = 8.0;
constexpr int kPlacementAttempts = 2000;

// The share of brakeMax a car plans with to stop at a line. The rest is what lets
// it catch up with the curve after a late start.
constexpr double kStopBrakingShare = 0.75;

// Metres short of the line such a stop aims for: the end of the curve is where
// a time step's rounding could otherwise carry the car across.
constexpr double kStopShort = 0.5;

// Below this a car's arrival time at a stop line says nothing.
constexpr double kCreepSpeed = 0.1;  // m/s

// Below this the time between crossings stops following the speed at the line.
constexpr double kSlowestCrossing = 1.0;  // m/s

// A car this slow is taken to be standing in a queue, m/s.
constexpr double kQueuedSpeed = 2.0;
// What one car queued on a street adds to the wait there: a green passes about two
// cars of a stream, and a cycle lasts two phases.
constexpr double kPhasesPerQueuedCar = 1.0;

// A turn waits for cars due at the crossing within this, seconds.
constexpr double kYieldSeconds = 2.0;
constexpr double kOnLane = 0.5;  // Metres.

// How much sooner another exit must promise to arrive for a car to change to it.
constexpr double kSwitchSeconds = 5.0;

// Cars per lane busyCarCount() aims for.
constexpr double kBusyCarsPerLane = 2.5;

double length(Vec2 v) { return std::sqrt(dot(v, v)); }

// Runs body(i) for every i below `count`, over all cores for a large count. The
// calls must not touch each other's data.
template <typename Body>
void parallelFor(std::size_t count, const Body &body) {
    constexpr std::size_t kWorthAThread = 64;
    const std::size_t workers = std::min<std::size_t>(
        std::max(std::thread::hardware_concurrency(), 1u), count / kWorthAThread + 1);
    if (workers <= 1) {
        for (std::size_t i = 0; i < count; i++) body(i);
        return;
    }
    std::atomic<std::size_t> next{0};
    const auto work = [&] {
        constexpr std::size_t kChunk = 16;
        for (;;) {
            const std::size_t start = next.fetch_add(kChunk);
            if (start >= count) return;
            const std::size_t end = std::min(start + kChunk, count);
            for (std::size_t i = start; i < end; i++) body(i);
        }
    };
    std::vector<std::thread> threads;
    for (std::size_t i = 1; i < workers; i++) threads.emplace_back(work);
    work();
    for (std::thread &thread : threads) thread.join();
}

std::shared_ptr<const PlanCost> tripCost() { return std::make_shared<EarliestArrivalCost>(); }

// Time the car needs behind the one crossing before it.
double crossingHeadway(QueuedCar car, const PlannerConfig &planner, const TrafficConfig &traffic) {
    const double reach = car.speed * car.speed + 2.0 * planner.aMax * car.distance;
    const double speed = std::clamp(std::sqrt(reach), kSlowestCrossing, planner.vMax);
    return traffic.reaction + (planner.carLength + traffic.minGap) / speed;
}

double freeArrival(QueuedCar car, double now, const PlannerConfig &planner) {
    return now + travelTime(car.distance, car.speed, planner.vMax, planner.aMax);
}

}  // namespace

void TrafficConfig::validate() const {
    if (cars < 0) throw std::invalid_argument("need cars >= 0");
    if (!(radioRange >= 0.0)) throw std::invalid_argument("need radioRange >= 0");
    if (!(dwellSeconds >= 0.0)) throw std::invalid_argument("need dwellSeconds >= 0");
    if (!(hotspotShare >= 0.0) || !(hotspotShare <= 1.0)) {
        throw std::invalid_argument("need hotspotShare in [0, 1]");
    }
    if (!(brakeMax > 0.0)) throw std::invalid_argument("need brakeMax > 0");
    if (!(minGap >= 0.0) || !(reaction >= 0.0)) {
        throw std::invalid_argument("need minGap >= 0 and reaction >= 0");
    }
    if (!(replanInterval > 0.0) || !(routeInterval > 0.0) || !(memoryHalfLife > 0.0)) {
        throw std::invalid_argument("need positive intervals and half-life");
    }
}

double queueClearTime(const GreenSchedule &schedule, const std::vector<QueuedCar> &ahead,
                      QueuedCar self, double now, const PlannerConfig &planner,
                      const TrafficConfig &traffic) {
    if (ahead.empty()) return -kInfinity;
    double previous = -kInfinity;
    for (const QueuedCar &car : ahead) {
        const double earliest = previous + crossingHeadway(car, planner, traffic);
        const double arrival = std::max(freeArrival(car, now, planner), earliest);
        // Cars ahead use a green to its very end, as the planner lets them.
        previous = schedule.nextOpening(arrival, planner.minSafetyMargin);
    }
    return previous + crossingHeadway(self, planner, traffic);
}

double followingSpeed(double gap, double leaderSpeed, double brake, double reaction) {
    const double room = brake * reaction;
    const double speed = -room + std::sqrt(room * room + leaderSpeed * leaderSpeed +
                                           2.0 * brake * std::max(gap, 0.0));
    // Too close already, as after a merge: at gap 0 the speed above still matches
    // the leader's, which would never open the gap again.
    const double fallBack = reaction > 0.0 ? std::min(gap, 0.0) / reaction : 0.0;
    return std::max(speed + fallBack, 0.0);
}

StreetMemory::StreetMemory(const City &city, double halfLife)
    : entries_(static_cast<std::size_t>(city.streetIndexCount())), halfLife_(halfLife) {}

void StreetMemory::remember(int street, double delay, double time) {
    entries_.at(static_cast<std::size_t>(street)) = {delay, time};
}

double StreetMemory::expectedDelay(int street, double time) const {
    const Entry &entry = entries_.at(static_cast<std::size_t>(street));
    return entry.delay * std::pow(0.5, (time - entry.time) / halfLife_);
}

Car::Car(const City &city, const PlannerConfig &planner, const TrafficConfig &traffic,
         const Trip &trip, double time)
    : city_(&city),
      planner_(planner),
      traffic_(traffic),
      controller_(planner, traffic.replanInterval, tripCost()),
      memory_(city, traffic.memoryHalfLife) {
    startTrip(trip, time);
}

PathOptions Car::pathOptions(double time, double notBefore) const {
    PathOptions options;
    options.startTime = time;
    options.startSpeed = state_.speed;
    options.notBefore = notBefore;
    if (traffic_.rerouting) {
        options.streetDelay = [this, time](Node from, Dir heading) {
            return memory_.expectedDelay(city_->streetIndex(from, heading), time);
        };
    }
    return options;
}

void Car::startTrip(const Trip &trip, double time) {
    trip_ = trip;
    state_.speed = std::min(state_.speed, planner_.vMax);
    const TripPath path =
        fastestPath(*city_, planner_, trip, pathOptions(time, -kInfinity));
    route_.emplace(*city_, path.nodes, path.endTrim);
    state_.position = path.startPosition;
    controller_ = GlosaController(planner_, traffic_.replanInterval, tripCost());
    ahead_.clear();
    tripStart_ = time;
    nextRouteTime_ = time + traffic_.routeInterval;
    currentStreet_ = route_->streetIndex(0);
    streetDelay_ = 0.0;
    streetEntered_ = time;
}

StreetPoint Car::location() const {
    const std::vector<Node> &nodes = route_->nodes();
    const std::size_t street = route_->streetAt(state_.position);
    return {nodes[street], headingBetween(nodes[street], nodes[street + 1]),
            route_->streetCoordinate(street, state_.position)};
}

CarMessage Car::message() const {
    const RoutePose pose = route_->poseAt(state_.position);
    return {pose.position, dirVector(pose.heading) * state_.speed};
}

void Car::readInbox(const Inbox &inbox) {
    ahead_.clear();
    for (const CarMessage &message : inbox) {
        const auto at = route_->locate(message.position, message.velocity, state_.position);
        if (at) ahead_.push_back({*at, length(message.velocity)});
    }
    std::sort(ahead_.begin(), ahead_.end(), [](const CarAhead &a, const CarAhead &b) {
        return a.position < b.position;
    });
}

// Cars standing on a street say how long its queue is. A street wholly in range
// that nobody is queueing on is known to be clear. One partly in range can only
// show itself to be worse than thought.
void Car::hearStreets(const Inbox &inbox, double time) {
    std::vector<int> queued(static_cast<std::size_t>(city_->streetIndexCount()), 0);
    for (const CarMessage &message : inbox) {
        if (length(message.velocity) >= kQueuedSpeed) continue;
        const auto street = city_->streetAt(message.position, message.velocity);
        if (street) queued[static_cast<std::size_t>(*street)]++;
    }

    const Vec2 here = route_->poseAt(state_.position).position;
    const auto inRange = [&](Node n) {
        return length(city_->nodeCenter(n) - here) <= traffic_.radioRange;
    };
    const double perCar = kPhasesPerQueuedCar * city_->config().lightPhaseSeconds;
    const CityConfig &grid = city_->config();
    for (int col = 0; col < grid.cols; col++) {
        for (int row = 0; row < grid.rows; row++) {
            const Node from{col, row};
            for (int d = 0; d < DirCount; d++) {
                const Dir heading = static_cast<Dir>(d);
                const Node to = neighbor(from, heading);
                if (!city_->nodeExists(to)) continue;
                const int street = city_->streetIndex(from, heading);
                const double delay = queued[static_cast<std::size_t>(street)] * perCar;
                const bool wholly = inRange(from) && inRange(to);
                if (wholly || delay > memory_.expectedDelay(street, time)) {
                    memory_.remember(street, delay, time);
                }
            }
        }
    }
}

void Car::noteStreet(double delay, double time) {
    const int street = route_->streetIndex(route_->streetAt(state_.position));
    if (street != currentStreet_) {
        // The queue model only foresees lights. What held the car up beyond a free
        // run and a full red came from something else, like a car stopped ahead.
        const CityConfig &grid = city_->config();
        const double expected = grid.blockLength / planner_.vMax + 2.0 * grid.lightPhaseSeconds;
        const double suffered = time - streetEntered_ - expected;
        memory_.remember(currentStreet_, std::max(streetDelay_, suffered), time);
        currentStreet_ = street;
        streetDelay_ = 0.0;
        streetEntered_ = time;
    }
    streetDelay_ = std::max(streetDelay_, delay);
}

void Car::reroute(const Inbox &inbox, double notBefore, double time) {
    if (time < nextRouteTime_) return;
    nextRouteTime_ = time + traffic_.routeInterval;
    hearStreets(inbox, time);

    // Only the exit from this street is still open. Once past the stop line it is
    // taken, and the last street leads straight to the end anyway.
    const std::size_t street = route_->streetAt(state_.position);
    if (street + 1 >= route_->streetCount()) return;
    const double coordinate = route_->streetCoordinate(street, state_.position);
    const double lastChance = city_->config().blockLength - city_->stopLineDistance();
    // The rear has to be on the street too: the new route starts at its first node.
    if (coordinate < planner_.carLength || coordinate >= lastChance) return;

    const std::vector<Node> &nodes = route_->nodes();
    const Trip fromHere{{nodes[street], headingBetween(nodes[street], nodes[street + 1]), coordinate},
                        trip_.end};
    PathOptions options = pathOptions(time, notBefore);
    const StreetDelay delay = options.streetDelay;
    const Node exit = nodes[street + 1];
    const Dir exitHeading = headingBetween(exit, nodes[street + 2]);
    options.streetDelay = [&](Node from, Dir heading) {
        const bool otherExit = from == exit && heading != exitHeading;
        return delay(from, heading) + (otherExit ? kSwitchSeconds : 0.0);
    };
    const TripPath path = fastestPath(*city_, planner_, fromHere, options);
    // Queues reported around the car come and go with every light, so only a clear
    // gain is worth a new way, and only the exit from this street needs deciding now.
    if (path.nodes[2] == nodes[street + 2]) return;

    route_.emplace(*city_, path.nodes, path.endTrim);
    state_.position = path.startPosition;
    // The next plan starts over on the new route.
    controller_ = GlosaController(planner_, traffic_.replanInterval, tripCost());
    reroutes_++;
}

// Whether the first car past the line is standing so close that this car, once
// across, would be left in the crossing.
bool Car::exitBlocked(double stopLine) const {
    const double farSide = stopLine + 2.0 * city_->stopLineDistance();
    for (const CarAhead &car : ahead_) {
        if (car.position <= stopLine) continue;
        if (car.speed >= kQueuedSpeed) return false;
        return car.position - planner_.carLength - planner_.carLength - traffic_.minGap < farSide;
    }
    return false;
}

// Whether a car is coming along `heading` to the lane point `entry`, within `seconds`
// and `distance` of it.
bool Car::carDueAt(const Inbox &inbox, Vec2 entry, Dir heading, double seconds,
                   double distance) const {
    const Vec2 forward = dirVector(heading);
    for (const CarMessage &message : inbox) {
        const double speed = dot(message.velocity, forward);
        if (speed < kQueuedSpeed) continue;
        const Vec2 toEntry = entry - message.position;
        if (std::abs(dot(toEntry, rightOf(heading))) > kOnLane) continue;
        const double away = dot(toEntry, forward);
        if (away > 0.0 && away <= distance && away / speed < seconds) return true;
    }
    return false;
}

// A turn gives way to whatever could meet it. Where the others are going cannot be
// heard, so that is judged from their lane alone: a car coming straight along the
// street being turned into, and for a left turn anything oncoming. A right turn
// waits only for an oncoming car already in the crossing, which may be turning
// left into the same street and can no longer stop for it.
bool Car::mustGiveWay(const Inbox &inbox, std::size_t street) const {
    if (street + 1 >= route_->streetCount()) return false;
    const Dir in = route_->streetHeading(street);
    const Dir out = route_->streetHeading(street + 1);
    if (in == out) return false;

    const Node node = route_->nodes()[street + 1];
    if (carDueAt(inbox, city_->turnPoint(node, out, out), out, kYieldSeconds, kInfinity)) {
        return true;
    }
    const Dir against = opposite(in);
    const bool left = route_->turnsLeftAfter(street);
    return carDueAt(inbox, city_->turnPoint(node, against, against), against, kYieldSeconds,
                    left ? kInfinity : city_->stopLineDistance());
}

void Car::step(const Inbox &inbox, double time, double dt) {
    readInbox(inbox);

    const std::vector<RouteLight> &lights = route_->lights();
    const std::size_t nextLight = route_->nextLightIndex(state_.position);
    const bool lightAhead = nextLight < lights.size();
    const double stopLine = lightAhead ? lights[nextLight].stopLine : kInfinity;

    double notBefore = -kInfinity;
    double trafficDelay = 0.0;
    if (lightAhead) {
        const RouteLight &light = lights[nextLight];
        const GreenSchedule schedule = city_->light(light.lightIndex).schedule(light.axis);

        std::vector<QueuedCar> queue;
        for (std::size_t i = ahead_.size(); i-- > 0;) {
            if (ahead_[i].position < stopLine) {
                queue.push_back({stopLine - ahead_[i].position, ahead_[i].speed});
            }
        }
        const QueuedCar self{stopLine - state_.position, state_.speed};
        notBefore = queueClearTime(schedule, queue, self, time, planner_, traffic_);

        const double arrival = freeArrival(self, time, planner_);
        trafficDelay =
            schedule.nextOpening(std::max(arrival, notBefore), planner_.minSafetyMargin) -
            schedule.nextOpening(arrival, planner_.minSafetyMargin);
        if (!traffic_.queueAwareSpeed) notBefore = -kInfinity;
    }
    noteStreet(trafficDelay, time);

    const double planned = controller_.targetSpeed(*route_, state_, time, notBefore);
    const double target = std::min(planned, arrivalSpeedLimit(*route_, state_, planner_.aMax));

    // What the plan may not override: the car ahead, and anything that holds the
    // car at the line.
    double limit = kInfinity;
    if (!ahead_.empty()) {
        const double gap =
            ahead_.front().position - planner_.carLength - state_.position - traffic_.minGap;
        limit = followingSpeed(gap, ahead_.front().speed, planner_.aMax, traffic_.reaction);
    }
    if (lightAhead) {
        // The plan assumes it could still speed up, which the car ahead may not allow.
        const RouteLight &light = lights[nextLight];
        const double toLine = stopLine - state_.position;
        const TrafficLight &signal = city_->light(light.lightIndex);
        const double arrival = time + toLine / std::max(state_.speed, kCreepSpeed);
        // Twice the margin the planner runs at, as the car ahead can still slow it.
        const double latest = arrival + 2.0 * planner_.minSafetyMargin;
        const bool redOnArrival =
            state_.speed > 0.0 &&
            (!signal.isGreen(light.axis, arrival) || !signal.isGreen(light.axis, latest));
        const Plan &plan = controller_.lastPlan();
        const bool planWaits = !plan.lights.empty() && plan.lights.front().fallback;

        const std::size_t street = route_->streetAt(state_.position);
        const bool lineEndsThisStreet = stopLine <= route_->streetEnd(street);
        const bool blocked = traffic_.keepCrossingsClear && exitBlocked(stopLine);
        const bool yielding =
            traffic_.giveWay && lineEndsThisStreet && mustGiveWay(inbox, street);

        if (redOnArrival || planWaits || blocked || yielding) {
            const double room = std::max(toLine - kStopShort, 0.0);
            limit = std::min(limit, std::sqrt(2.0 * kStopBrakingShare * traffic_.brakeMax * room));
        }
    }

    stepCar(state_, target, limit, planner_.aMax, traffic_.brakeMax, dt);

    if (traffic_.rerouting && !arrived()) reroute(inbox, notBefore, time + dt);
}

Traffic::Traffic(const City &city, const PlannerConfig &planner, const TrafficConfig &config,
                 std::mt19937 &rng)
    : city_(&city),
      planner_(planner),
      config_(config),
      rng_(&rng) {
    config_.validate();

    const auto clashes = [&](const Trip &trip) {
        for (const Car &car : cars_) {
            const StreetPoint other = car.location();
            if (other.from == trip.start.from && other.heading == trip.start.heading &&
                std::abs(other.offset - trip.start.offset) < kPlacementGap) {
                return true;
            }
        }
        return false;
    };

    // Anywhere along a lane, clear of the stop line and of the street's start.
    std::uniform_real_distribution<double> anywhere(
        planner_.carLength + 1.0, city.config().blockLength - city.stopLineDistance() - 2.0);

    cars_.reserve(static_cast<std::size_t>(config_.cars));
    for (int i = 0; i < config_.cars; i++) {
        const auto place = [&] {
            Trip trip = randomTrip(city, rng, config_.hotspotShare);
            trip.start.offset = anywhere(rng);
            return trip;
        };
        Trip trip = place();
        int attempts = 0;
        while (clashes(trip)) {
            if (++attempts == kPlacementAttempts) {
                throw std::invalid_argument("Traffic: too many cars for this city");
            }
            trip = place();
        }
        cars_.emplace_back(city, planner_, config_, trip, 0.0);
    }
    touching_.assign(cars_.size(), 0);
    parkedUntil_.assign(cars_.size(), -kInfinity);
    inboxes_.resize(cars_.size());
    deliverMessages();
}

void Traffic::deliverMessages() {
    std::vector<CarMessage> messages;
    messages.reserve(cars_.size());
    for (const Car &car : cars_) messages.push_back(car.message());

    const double range = config_.radioRange * config_.radioRange;
    parallelFor(cars_.size(), [&](std::size_t i) {
        Inbox &inbox = inboxes_[i];
        inbox.clear();
        for (std::size_t j = 0; j < messages.size(); j++) {
            const Vec2 apart = messages[j].position - messages[i].position;
            if (j != i && dot(apart, apart) <= range) inbox.push_back(messages[j]);
        }
    });
}

void Traffic::countOverlaps() {
    // Judged only between cars that are both really in the lane. Inside a crossing
    // they are still on their own ways in.
    struct Occupant {
        std::size_t car = 0;
        double coordinate = 0.0;
    };
    std::vector<std::vector<Occupant>> inLane(static_cast<std::size_t>(city_->streetIndexCount()));
    for (std::size_t i = 0; i < cars_.size(); i++) {
        const Route &route = cars_[i].route();
        const double position = cars_[i].state().position;
        const std::size_t k = route.streetAt(position);
        inLane[static_cast<std::size_t>(route.streetIndex(k))].push_back(
            {i, route.streetCoordinate(k, position)});
    }

    std::vector<char> touching(cars_.size(), 0);
    for (std::vector<Occupant> &lane : inLane) {
        std::sort(lane.begin(), lane.end(), [](const Occupant &a, const Occupant &b) {
            return a.coordinate < b.coordinate;
        });
        for (std::size_t i = 0; i + 1 < lane.size(); i++) {
            const double depth = planner_.carLength - (lane[i + 1].coordinate - lane[i].coordinate);
            if (depth <= 0.0) continue;
            touching[lane[i].car] = 1;
            stats_.overlaps += touching_[lane[i].car] ? 0 : 1;
            stats_.deepestOverlap = std::max(stats_.deepestOverlap, depth);
        }
    }
    touching_ = std::move(touching);
}

void Traffic::step(double dt) {
    // What the referee needs from before the step, to see what it changed.
    struct Before {
        double toLine = kInfinity;
        RouteLight light;
        int reroutes = 0;
        bool moving = false;
        bool stepped = false;
    };
    std::vector<Before> before(cars_.size());

    // Every car acts on the same messages, so the order of the cars does not matter.
    parallelFor(cars_.size(), [&](std::size_t i) {
        Car &car = cars_[i];
        if (car.arrived()) return;  // Parked.
        Before &b = before[i];
        b.stepped = true;
        const std::size_t next = car.route().nextLightIndex(car.state().position);
        const std::vector<RouteLight> &lights = car.route().lights();
        if (next < lights.size()) {
            b.light = lights[next];
            b.toLine = b.light.stopLine - car.state().position;
        }
        b.reroutes = car.reroutes();
        b.moving = car.state().speed >= kCreepSpeed;
        car.step(inboxes_[i], time_, dt);
    });

    for (std::size_t i = 0; i < cars_.size(); i++) {
        const Before &b = before[i];
        const Car &car = cars_[i];
        if (!b.stepped) continue;
        const double moved = car.state().speed * dt;
        stats_.stops += b.moving && car.state().speed < kCreepSpeed ? 1 : 0;
        stats_.distance += moved;
        stats_.reroutes += car.reroutes() - b.reroutes;
        // A reroute moves the car's coordinates, so distances are compared instead.
        if (b.toLine < moved &&
            !city_->light(b.light.lightIndex).isGreen(b.light.axis, time_ + dt)) {
            stats_.redCrossings++;
        }
    }

    time_ += dt;
    for (std::size_t i = 0; i < cars_.size(); i++) {
        Car &car = cars_[i];
        if (!car.arrived()) continue;
        if (parkedUntil_[i] < car.tripStart()) {
            stats_.tripsFinished++;
            stats_.tripSeconds += time_ - car.tripStart();
            parkedUntil_[i] = time_ + config_.dwellSeconds;
            car.park();
        }
        if (time_ >= parkedUntil_[i]) {
            car.startTrip(nextTrip(*city_, car.location(), *rng_, config_.hotspotShare), time_);
        }
    }
    deliverMessages();
    countOverlaps();
}

int busyCarCount(const City &city) {
    int lanes = 0;
    const CityConfig &grid = city.config();
    for (int col = 0; col < grid.cols; col++) {
        for (int row = 0; row < grid.rows; row++) {
            for (int d = 0; d < DirCount; d++) {
                lanes += city.nodeExists(neighbor({col, row}, static_cast<Dir>(d))) ? 1 : 0;
            }
        }
    }
    return static_cast<int>(kBusyCarsPerLane * lanes);
}
