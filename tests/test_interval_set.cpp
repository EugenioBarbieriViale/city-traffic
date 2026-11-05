#include <limits>
#include <stdexcept>

#include "IntervalSet.h"
#include "TestSupport.h"

TEST(default_set_is_empty) {
    IntervalSet s;
    CHECK(s.empty());
    CHECK(s.size() == 0);
    CHECK(!s.contains(0.0));
    CHECK_THROWS(std::logic_error, s.min());
    CHECK_THROWS(std::logic_error, s.max());
}

TEST(construction_sorts_and_merges_overlaps) {
    IntervalSet s{{5, 7}, {1, 3}, {2, 4}};
    CHECK((s == IntervalSet{{1, 4}, {5, 7}}));
}

TEST(touching_closed_intervals_merge) {
    IntervalSet s{{1, 2}, {2, 3}};
    CHECK((s == IntervalSet{{1, 3}}));
    CHECK(s.size() == 1);
}

TEST(nested_interval_is_absorbed) {
    IntervalSet s{{0, 10}, {2, 3}, {4, 5}};
    CHECK((s == IntervalSet{{0, 10}}));
}

TEST(a_nearly_touching_gap_stays_a_gap) {
    IntervalSet s{{0, 1}, {1.0000001, 2}};
    CHECK(s.size() == 2);
    CHECK(!s.contains(1.00000005));
}

TEST(reversed_interval_counts_as_empty) {
    IntervalSet s{{3, 1}, {5, 6}};
    CHECK((s == IntervalSet{{5, 6}}));
    CHECK(IntervalSet{{3, 1}}.empty());
}

TEST(nan_is_rejected) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(std::invalid_argument, IntervalSet({{nan, 1}}));
    CHECK_THROWS(std::invalid_argument, IntervalSet{{0, 1}}.shift(nan, 1));
}

TEST(point_is_a_degenerate_interval) {
    IntervalSet s = IntervalSet::point(4.0);
    CHECK(!s.empty());
    CHECK(s.contains(4.0));
    CHECK(!s.contains(4.0001));
    CHECK(s.min() == 4.0 && s.max() == 4.0);
}

TEST(contains_includes_both_ends_and_excludes_gaps) {
    IntervalSet s{{1, 2}, {4, 5}};
    CHECK(s.contains(1) && s.contains(2) && s.contains(4) && s.contains(5));
    CHECK(!s.contains(0.999) && !s.contains(3) && !s.contains(5.001));
}

TEST(interval_containing_finds_the_member) {
    IntervalSet s{{1, 2}, {4, 5}};
    auto found = s.intervalContaining(4.5);
    CHECK(found.has_value());
    CHECK((*found == Interval{4, 5}));
    CHECK(!s.intervalContaining(3).has_value());
}

TEST(shift_is_a_minkowski_sum) {
    IntervalSet s{{10, 12}};
    CHECK((s.shift(1, 3) == IntervalSet{{11, 15}}));
}

TEST(shift_of_a_point_gives_the_interval) {
    CHECK((IntervalSet::point(2).shift(3, 8) == IntervalSet{{5, 10}}));
}

TEST(shift_merges_intervals_that_grow_into_each_other) {
    IntervalSet s{{0, 1}, {3, 4}, {20, 21}};
    CHECK((s.shift(0, 2) == IntervalSet{{0, 6}, {20, 23}}));
}

TEST(shift_with_negative_bounds_moves_backward) {
    CHECK((IntervalSet{{10, 12}}.shift(-5, -2) == IntervalSet{{5, 10}}));
}

TEST(shift_of_the_empty_set_stays_empty) { CHECK(IntervalSet{}.shift(1, 2).empty()); }

TEST(shift_needs_ordered_bounds) {
    CHECK_THROWS(std::invalid_argument, IntervalSet{{0, 1}}.shift(2, 1));
}

TEST(intersect_across_several_intervals) {
    IntervalSet a{{0, 5}, {10, 15}};
    IntervalSet b{{3, 12}, {14, 20}};
    CHECK((a.intersect(b) == IntervalSet{{3, 5}, {10, 12}, {14, 15}}));
    CHECK((b.intersect(a) == a.intersect(b)));
}

TEST(intersect_of_touching_closed_intervals_is_a_point) {
    IntervalSet result = IntervalSet{{1, 2}}.intersect(IntervalSet{{2, 3}});
    CHECK((result == IntervalSet::point(2)));
}

TEST(intersect_of_disjoint_sets_is_empty) {
    CHECK(IntervalSet{{1, 2}}.intersect(IntervalSet{{3, 4}}).empty());
    CHECK(IntervalSet{{1, 2}}.intersect(IntervalSet{}).empty());
    CHECK(IntervalSet{}.intersect(IntervalSet{{1, 2}}).empty());
}

TEST(intersect_when_one_interval_spans_many) {
    IntervalSet a{{0, 100}};
    IntervalSet b{{1, 2}, {3, 4}, {5, 6}};
    CHECK((a.intersect(b) == b));
    CHECK((b.intersect(a) == b));
}

TEST(intersect_with_a_single_interval) {
    IntervalSet a{{0, 2}, {4, 6}};
    CHECK((a.intersect(Interval{1, 5}) == IntervalSet{{1, 2}, {4, 5}}));
}

TEST(infinite_bounds_work_as_half_lines) {
    double inf = std::numeric_limits<double>::infinity();
    IntervalSet a{{0, 2}, {4, 6}};
    CHECK((a.intersect(Interval{-inf, 5}) == IntervalSet{{0, 2}, {4, 5}}));
}

TEST(unite_merges_two_sets) {
    IntervalSet a{{0, 2}, {6, 8}};
    IntervalSet b{{1, 3}, {10, 11}};
    CHECK((a.unite(b) == IntervalSet{{0, 3}, {6, 8}, {10, 11}}));
    CHECK((a.unite(IntervalSet{}) == a));
}

int main() { return testing::runAll(); }
