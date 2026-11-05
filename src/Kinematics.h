#pragma once

// Time and speed along a straight line under the car's motion model: a ramp at
// constant acceleration from `v0` to a cruise speed, then cruising.
//
// `aMax` may be infinity, which gives an instantaneous speed change.

// Distance needed to change speed from v0 to v.
double rampDistance(double v0, double v, double aMax);

// Time to cover `distance` starting at v0 and heading for cruise speed `vCruise`.
// When the distance ends mid-ramp, this is the time at which the ramp reaches
// it. Returns infinity when the car stops short of the distance.
double travelTime(double distance, double v0, double vCruise, double aMax);

// Cruise speed in [vLow, vHigh] whose travelTime() comes closest to `duration`.
// Arriving later than vLow allows returns vLow; earlier than vHigh returns vHigh.
double cruiseSpeedForDuration(double distance, double duration, double v0, double vLow,
                              double vHigh, double aMax);
