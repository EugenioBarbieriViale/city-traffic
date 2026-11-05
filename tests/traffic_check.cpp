// Runs whole cities of cars and compares them with parts of the traffic model
// turned off. Exits non-zero if a car runs a red under the full model. The other
// rows are baselines, which do not know about queues and can clip a red.
//
// Usage: traffic_check [cities per row, default 6] [simulated seconds, default 300]

#include <algorithm>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "City.h"
#include "Traffic.h"
#include "Trip.h"

namespace {

constexpr double kStep = 1.0 / 30.0;

struct Mode {
    const char *name;
    bool queueAware;
    bool rerouting;
    bool giveWay;
};

struct Totals {
    int trips = 0;
    double tripSeconds = 0.0;
    double distance = 0.0;
    int stops = 0;
    int reds = 0;
    int overlaps = 0;
    double deepest = 0.0;
    int reroutes = 0;
};

Totals runCity(unsigned seed, int cars, const Mode &mode, double duration) {
    std::mt19937 rng(seed);
    CityConfig cityConfig = randomCityConfig(rng);
    // Small enough for this many cars to get in each other's way.
    cityConfig.cols = 4;
    cityConfig.rows = 4;
    const City city(cityConfig);
    TrafficConfig config;
    config.cars = cars;
    config.queueAwareSpeed = mode.queueAware;
    config.rerouting = mode.rerouting;
    config.giveWay = mode.giveWay;
    Traffic traffic(city, tripPlannerConfig(), config, rng);
    while (traffic.time() < duration) traffic.step(kStep);

    const TrafficStats &s = traffic.stats();
    return {s.tripsFinished, s.tripSeconds,  s.distance,       s.stops,
            s.redCrossings,  s.overlaps,     s.deepestOverlap, s.reroutes};
}

}  // namespace

int main(int argc, char **argv) {
    const int seeds = argc > 1 ? std::stoi(argv[1]) : 6;
    const double duration = argc > 2 ? std::stod(argv[2]) : 300.0;
    const std::vector<Mode> modes = {{"no queue speed", false, true, true},
                                     {"no rerouting", true, false, true},
                                     {"no give-way", true, true, false},
                                     {"full model", true, true, true}};

    std::printf("%d cities of %.0f s each\n", seeds, duration);
    std::printf("%5s %-18s | %7s %9s %10s | %5s %8s %8s %9s\n", "cars", "mode", "trips", "s/trip",
                "stops/trip", "reds", "overlaps", "deepest", "reroutes");
    int reds = 0;
    for (int cars : {60, 120, 200}) {
        for (const Mode &mode : modes) {
            Totals all;
            for (int i = 0; i < seeds; i++) {
                const Totals t = runCity(1000u + static_cast<unsigned>(i), cars, mode, duration);
                all.trips += t.trips;
                all.tripSeconds += t.tripSeconds;
                all.distance += t.distance;
                all.stops += t.stops;
                all.reds += t.reds;
                all.overlaps += t.overlaps;
                all.deepest = std::max(all.deepest, t.deepest);
                all.reroutes += t.reroutes;
            }
            std::printf("%5d %-18s | %7d %9.1f %10.2f | %5d %8d %7.2fm %9d\n", cars, mode.name,
                        all.trips, all.tripSeconds / all.trips,
                        static_cast<double>(all.stops) / all.trips, all.reds, all.overlaps,
                        all.deepest, all.reroutes);
            if (mode.queueAware && mode.rerouting && mode.giveWay) reds += all.reds;
        }
    }
    std::printf("%s: %d red crossing(s)\n", reds == 0 ? "PASS" : "FAIL", reds);
    return reds == 0 ? 0 : 1;
}
