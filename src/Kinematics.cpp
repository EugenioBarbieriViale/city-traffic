#include "Kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

double rampDistance(double v0, double v, double aMax) {
    if (std::isinf(aMax)) return 0.0;
    return std::abs(v * v - v0 * v0) / (2.0 * aMax);
}

double travelTime(double distance, double v0, double vCruise, double aMax) {
    if (distance <= 0.0) return 0.0;

    const double ramp = rampDistance(v0, vCruise, aMax);
    if (ramp >= distance) {
        // The whole distance is covered during the ramp.
        const double sign = vCruise >= v0 ? 1.0 : -1.0;
        const double vEnd = std::sqrt(std::max(v0 * v0 + sign * 2.0 * aMax * distance, 0.0));
        return std::abs(vEnd - v0) / aMax;
    }

    const double rampTime = std::isinf(aMax) ? 0.0 : std::abs(vCruise - v0) / aMax;
    if (vCruise <= 0.0) return std::numeric_limits<double>::infinity();
    return rampTime + (distance - ramp) / vCruise;
}

double cruiseSpeedForDuration(double distance, double duration, double v0, double vLow,
                              double vHigh, double aMax) {
    if (distance <= 0.0) return duration > 0.0 ? vLow : vHigh;
    if (travelTime(distance, v0, vHigh, aMax) >= duration) return vHigh;
    if (travelTime(distance, v0, vLow, aMax) <= duration) return vLow;

    // travelTime falls monotonically as the cruise speed rises.
    double low = vLow;
    double high = vHigh;
    for (int i = 0; i < 60; i++) {
        double mid = (low + high) / 2.0;
        if (travelTime(distance, v0, mid, aMax) > duration) low = mid;
        else high = mid;
    }
    return (low + high) / 2.0;
}
