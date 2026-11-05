#include "IntervalSet.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

IntervalSet::IntervalSet(std::initializer_list<Interval> intervals) : intervals_(intervals) {
    normalize();
}

IntervalSet::IntervalSet(std::vector<Interval> intervals) : intervals_(std::move(intervals)) {
    normalize();
}

IntervalSet IntervalSet::point(double t) { return IntervalSet{{t, t}}; }

// Intervals with lo > hi are empty, which is what shrinking a short window
// produces, so they are dropped rather than rejected.
void IntervalSet::normalize() {
    for (const Interval &i : intervals_) {
        if (std::isnan(i.lo) || std::isnan(i.hi)) {
            throw std::invalid_argument("IntervalSet: NaN bound");
        }
    }
    intervals_.erase(std::remove_if(intervals_.begin(), intervals_.end(),
                                    [](const Interval &i) { return i.lo > i.hi; }),
                     intervals_.end());
    std::sort(intervals_.begin(), intervals_.end(),
              [](const Interval &a, const Interval &b) { return a.lo < b.lo; });

    std::vector<Interval> merged;
    merged.reserve(intervals_.size());
    for (const Interval &i : intervals_) {
        if (!merged.empty() && i.lo <= merged.back().hi) {
            merged.back().hi = std::max(merged.back().hi, i.hi);
        } else {
            merged.push_back(i);
        }
    }
    intervals_ = std::move(merged);
}

double IntervalSet::min() const {
    if (empty()) throw std::logic_error("IntervalSet::min on an empty set");
    return intervals_.front().lo;
}

double IntervalSet::max() const {
    if (empty()) throw std::logic_error("IntervalSet::max on an empty set");
    return intervals_.back().hi;
}

bool IntervalSet::contains(double t) const { return intervalContaining(t).has_value(); }

std::optional<Interval> IntervalSet::intervalContaining(double t) const {
    auto after = std::upper_bound(intervals_.begin(), intervals_.end(), t,
                                  [](double value, const Interval &i) { return value < i.lo; });
    if (after == intervals_.begin()) return std::nullopt;
    const Interval &candidate = *std::prev(after);
    if (!candidate.contains(t)) return std::nullopt;
    return candidate;
}

IntervalSet IntervalSet::shift(double lo, double hi) const {
    if (std::isnan(lo) || std::isnan(hi) || lo > hi) {
        throw std::invalid_argument("IntervalSet::shift: need lo <= hi");
    }
    std::vector<Interval> shifted;
    shifted.reserve(intervals_.size());
    for (const Interval &i : intervals_) shifted.push_back({i.lo + lo, i.hi + hi});
    return IntervalSet(std::move(shifted));  // Widening can make neighbors overlap.
}

IntervalSet IntervalSet::intersect(const IntervalSet &other) const {
    std::vector<Interval> result;
    std::size_t a = 0;
    std::size_t b = 0;
    while (a < intervals_.size() && b < other.intervals_.size()) {
        const Interval &x = intervals_[a];
        const Interval &y = other.intervals_[b];
        double lo = std::max(x.lo, y.lo);
        double hi = std::min(x.hi, y.hi);
        if (lo <= hi) result.push_back({lo, hi});
        // The interval ending first cannot meet anything further on.
        if (x.hi < y.hi) a++;
        else b++;
    }
    return IntervalSet(std::move(result));
}

IntervalSet IntervalSet::intersect(Interval other) const {
    return intersect(IntervalSet{other});
}

IntervalSet IntervalSet::unite(const IntervalSet &other) const {
    std::vector<Interval> all = intervals_;
    all.insert(all.end(), other.intervals_.begin(), other.intervals_.end());
    return IntervalSet(std::move(all));
}
