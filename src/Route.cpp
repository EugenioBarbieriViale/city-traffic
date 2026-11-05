#include "Route.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Route::Route(const City &city, std::vector<Node> nodes, double endTrim)
    : city_(&city), nodes_(std::move(nodes)), endTrim_(endTrim) {
    if (nodes_.size() < 2) throw std::invalid_argument("Route: needs at least two nodes");
    rebuildWaypoints();
}

void Route::extend(Node next) {
    nodes_.push_back(next);
    rebuildWaypoints();
}

Dir Route::finalHeading() const { return headings_.back(); }

void Route::rebuildWaypoints() {
    headings_.clear();
    for (std::size_t i = 0; i < nodes_.size(); i++) {
        if (!city_->nodeExists(nodes_[i])) throw std::invalid_argument("Route: unknown node");
        if (i == 0) continue;
        Dir heading = headingBetween(nodes_[i - 1], nodes_[i]);
        if (!headings_.empty() && heading == opposite(headings_.back())) {
            throw std::invalid_argument("Route: U-turns are not supported");
        }
        headings_.push_back(heading);
    }

    waypoints_.clear();
    waypoints_.push_back(city_->turnPoint(nodes_.front(), headings_.front(), headings_.front()));
    for (std::size_t i = 1; i < nodes_.size(); i++) {
        Dir in = headings_[i - 1];
        Dir out = i < headings_.size() ? headings_[i] : in;
        waypoints_.push_back(city_->turnPoint(nodes_[i], in, out));
    }

    cumulative_.assign(1, 0.0);
    lights_.clear();
    for (std::size_t i = 0; i + 1 < waypoints_.size(); i++) {
        Vec2 forward = dirVector(headings_[i]);
        double start = cumulative_.back();
        cumulative_.push_back(start + dot(waypoints_[i + 1] - waypoints_[i], forward));

        Node target = nodes_[i + 1];
        auto lightIndex = city_->lightIndexAt(target);
        if (!lightIndex) continue;
        Vec2 stopLine = city_->nodeCenter(target) - forward * city_->stopLineDistance();
        double along = dot(stopLine - waypoints_[i], forward);
        lights_.push_back({*lightIndex, axisOf(headings_[i]), start + along});
    }

    const double lastStreet = cumulative_.back() - cumulative_[cumulative_.size() - 2];
    if (endTrim_ < 0.0 || endTrim_ >= lastStreet) {
        throw std::invalid_argument("Route: endTrim must lie within the last street");
    }
    const double end = length();
    lights_.erase(std::remove_if(lights_.begin(), lights_.end(),
                                 [end](const RouteLight &l) { return l.stopLine >= end; }),
                  lights_.end());
}

RoutePose Route::poseAt(double position) const {
    position = std::clamp(position, 0.0, length());
    const std::size_t segment = streetAt(position);
    Dir heading = headings_[segment];
    return {waypoints_[segment] + dirVector(heading) * (position - cumulative_[segment]), heading};
}

std::size_t Route::streetAt(double position) const {
    auto it = std::upper_bound(cumulative_.begin(), cumulative_.end(), position);
    std::size_t street = static_cast<std::size_t>(it - cumulative_.begin());
    return std::min(street == 0 ? 0 : street - 1, headings_.size() - 1);
}

int Route::streetIndex(std::size_t street) const {
    return city_->streetIndex(nodes_[street], headings_[street]);
}

bool Route::turnsLeftAfter(std::size_t street) const {
    if (street + 1 >= headings_.size()) return false;
    return dot(rightOf(headings_[street]), dirVector(headings_[street + 1])) < 0.0;
}

std::optional<double> Route::locate(Vec2 point, Vec2 velocity, double from) const {
    constexpr double kOnLane = 0.5;  // Metres.
    const bool moving = dot(velocity, velocity) > 0.0;
    for (std::size_t k = streetAt(from); k < headings_.size(); k++) {
        const Vec2 forward = dirVector(headings_[k]);
        if (moving && dot(velocity, forward) <= 0.0) continue;
        const Vec2 offset = point - waypoints_[k];
        if (std::abs(dot(offset, rightOf(headings_[k]))) > kOnLane) continue;
        const double along = cumulative_[k] + dot(offset, forward);
        if (along >= from && along >= cumulative_[k] && along <= cumulative_[k + 1]) return along;
    }
    return std::nullopt;
}

double Route::streetCoordinate(std::size_t street, double position) const {
    const Vec2 pastCenter = waypoints_[street] - city_->nodeCenter(nodes_[street]);
    return position - cumulative_[street] + dot(pastCenter, dirVector(headings_[street]));
}

std::vector<Vec2> Route::polyline(double from, double to) const {
    std::vector<Vec2> points{poseAt(from).position};
    for (std::size_t i = 0; i < waypoints_.size(); i++) {
        if (cumulative_[i] > from && cumulative_[i] < to) points.push_back(waypoints_[i]);
    }
    points.push_back(poseAt(to).position);
    return points;
}

std::size_t Route::nextLightIndex(double position) const {
    auto it = std::lower_bound(lights_.begin(), lights_.end(), position,
                               [](const RouteLight &l, double p) { return l.stopLine < p; });
    return static_cast<std::size_t>(it - lights_.begin());
}

std::vector<UpcomingLight> Route::lightsAhead(double position, double horizon) const {
    std::vector<UpcomingLight> ahead;
    for (std::size_t i = nextLightIndex(position); i < lights_.size(); i++) {
        const RouteLight &l = lights_[i];
        double distance = l.stopLine - position;
        if (distance > horizon) break;
        UpcomingLight upcoming;
        upcoming.distance = distance;
        upcoming.schedule = city_->light(l.lightIndex).schedule(l.axis);
        ahead.push_back(upcoming);
    }
    return ahead;
}
