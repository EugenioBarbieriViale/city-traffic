#pragma once

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <vector>

// A closed interval [lo, hi]. A single instant is the degenerate case lo == hi.
struct Interval {
    double lo = 0.0;
    double hi = 0.0;

    double width() const { return hi - lo; }
    double mid() const { return lo + (hi - lo) / 2.0; }
    bool contains(double t) const { return lo <= t && t <= hi; }

    friend bool operator==(const Interval &a, const Interval &b) {
        return a.lo == b.lo && a.hi == b.hi;
    }
};

// A union of closed intervals, always kept sorted, disjoint and non-touching:
// intervals that overlap or share an endpoint are merged on the way in.
class IntervalSet {
public:
    IntervalSet() = default;
    IntervalSet(std::initializer_list<Interval> intervals);
    explicit IntervalSet(std::vector<Interval> intervals);

    // A set holding one instant: the start of a propagation, R_0 = {t0}.
    static IntervalSet point(double t);

    bool empty() const { return intervals_.empty(); }
    std::size_t size() const { return intervals_.size(); }
    const std::vector<Interval> &intervals() const { return intervals_; }

    // Smallest and largest element. Both throw std::logic_error on an empty set.
    double min() const;
    double max() const;

    bool contains(double t) const;
    // The member interval holding t, if there is one.
    std::optional<Interval> intervalContaining(double t) const;

    // Minkowski sum with [lo, hi]: every element t becomes [t + lo, t + hi].
    IntervalSet shift(double lo, double hi) const;
    IntervalSet shift(Interval by) const { return shift(by.lo, by.hi); }

    IntervalSet intersect(const IntervalSet &other) const;
    IntervalSet intersect(Interval other) const;
    IntervalSet unite(const IntervalSet &other) const;

    friend bool operator==(const IntervalSet &a, const IntervalSet &b) {
        return a.intervals_ == b.intervals_;
    }

private:
    void normalize();

    std::vector<Interval> intervals_;
};
