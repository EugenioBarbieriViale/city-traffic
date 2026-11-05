#include "raylib.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#include "City.h"
#include "Route.h"
#include "Simulation.h"
#include "SpeedPlanner.h"
#include "Traffic.h"
#include "Trip.h"

namespace {

constexpr int kWindowWidth = 1600;   // Shrunk to fit the monitor. The window can be resized.
constexpr int kWindowHeight = 1100;
constexpr float kScreenShare = 0.95f;  // Of the monitor, at most.

constexpr float kMargin = 30.0f;      // Pixels around the grid.
constexpr float kStatusHeight = 76.0f;  // Pixels kept free for the text above it.
constexpr int kFontSize = 28;

constexpr float kCarPixels = 11.0f;   // Least size. To scale once the city is drawn larger.
constexpr float kPathThickness = 5.0f;
constexpr float kEndMarkerRadius = 12.0f;
constexpr float kStopLineThickness = 7.0f;
constexpr float kCenterLineThickness = 2.0f;

constexpr double kStep = 1.0 / 30.0;   // Seconds. Fixed, so that a seed replays exactly.
constexpr double kMaxFrameTime = 0.1;  // Avoids jumps after the window stalls.

// The world is in metres and the window in pixels. Every city fills the window.
struct View {
    float pixelsPerMetre = 1.0f;
    Vector2 origin;

    Vector2 toScreen(Vec2 world) const {
        return {origin.x + static_cast<float>(world.x) * pixelsPerMetre,
                origin.y + static_cast<float>(world.y) * pixelsPerMetre};
    }
    float toPixels(double metres) const { return static_cast<float>(metres) * pixelsPerMetre; }
};

View fitToWindow(const City &city, float windowWidth, float windowHeight) {
    const CityConfig &config = city.config();
    const float road = static_cast<float>(config.roadWidth);
    const float width = static_cast<float>((config.cols - 1) * config.blockLength) + road;
    const float height = static_cast<float>((config.rows - 1) * config.blockLength) + road;
    const float freeWidth = windowWidth - 2 * kMargin;
    const float freeHeight = windowHeight - 2 * kMargin - kStatusHeight;
    View view;
    view.pixelsPerMetre = std::min(freeWidth / width, freeHeight / height);
    const float drawnRoad = road * view.pixelsPerMetre;
    view.origin = {(windowWidth - width * view.pixelsPerMetre + drawnRoad) / 2,
                   kStatusHeight + (freeHeight + 2 * kMargin - height * view.pixelsPerMetre + drawnRoad) / 2};
    return view;
}

void drawRoadSegment(const City &city, const View &view, Node from, Node to) {
    Vector2 a = view.toScreen(city.nodeCenter(from));
    Vector2 b = view.toScreen(city.nodeCenter(to));
    float width = view.toPixels(city.config().roadWidth);
    Rectangle road = {a.x - width / 2, a.y - width / 2, b.x - a.x + width, b.y - a.y + width};
    DrawRectangleRec(road, DARKGRAY);
}

void drawCenterLine(const City &city, const View &view, Node from, Node to, Dir d) {
    Vec2 inset = dirVector(d) * (city.config().roadWidth / 2.0);
    DrawLineEx(view.toScreen(city.nodeCenter(from) + inset),
               view.toScreen(city.nodeCenter(to) - inset), kCenterLineThickness, LIGHTGRAY);
}

void drawStreets(const City &city, const View &view) {
    const CityConfig &config = city.config();
    for (int pass = 0; pass < 2; pass++) {
        for (int col = 0; col < config.cols; col++) {
            for (int row = 0; row < config.rows; row++) {
                Node n{col, row};
                for (Dir d : {East, South}) {
                    Node next = neighbor(n, d);
                    if (!city.nodeExists(next)) continue;
                    if (pass == 0) drawRoadSegment(city, view, n, next);
                    else drawCenterLine(city, view, n, next, d);
                }
            }
        }
    }
}

// Each light is a colored stop line across the lane that approaches the crossing.
void drawTrafficLights(const City &city, const View &view, double time) {
    float laneWidth = view.toPixels(city.config().roadWidth / 2.0) - 4.0f;
    for (const TrafficLight &light : city.lights()) {
        for (int i = 0; i < DirCount; i++) {
            Dir heading = static_cast<Dir>(i);
            if (!city.nodeExists(neighbor(light.node, opposite(heading)))) continue;

            Vec2 center = city.nodeCenter(light.node) -
                          dirVector(heading) * city.stopLineDistance() +
                          rightOf(heading) * city.laneOffset();
            Vector2 screen = view.toScreen(center);
            bool vertical = heading == North || heading == South;
            Vector2 size = vertical ? Vector2{laneWidth, kStopLineThickness}
                                    : Vector2{kStopLineThickness, laneWidth};
            Vector2 corner = {screen.x - size.x / 2, screen.y - size.y / 2};
            DrawRectangleV(corner, size, light.isGreen(axisOf(heading), time) ? GREEN : RED);
        }
    }
}

// The part of the route still ahead of the car.
void drawPath(const Route &route, const View &view, double position) {
    std::vector<Vec2> points = route.polyline(position, route.length());
    for (std::size_t i = 0; i + 1 < points.size(); i++) {
        Vector2 a = view.toScreen(points[i]);
        Vector2 b = view.toScreen(points[i + 1]);
        DrawLineEx(a, b, kPathThickness, SKYBLUE);
        DrawCircleV(b, kPathThickness / 2, SKYBLUE);  // Fills the outside of the corner.
    }
}

void drawEndMarker(const Route &route, const View &view) {
    Vector2 end = view.toScreen(route.poseAt(route.length()).position);
    DrawCircleV(end, kEndMarkerRadius, SKYBLUE);
    DrawCircleV(end, kEndMarkerRadius / 2, RAYWHITE);
}

void drawCar(const City &city, const View &view, const Car &car, double carLength, Color color) {
    const RoutePose pose = car.route().poseAt(car.state().position);
    const Vec2 middle = pose.position - dirVector(pose.heading) * (carLength / 2.0);
    const Vector2 center = view.toScreen(middle);
    // No longer than a queue's spacing, so that queued cars stay apart on screen.
    const float spacing = view.toPixels(carLength + TrafficConfig{}.minGap) * 0.9f;
    const float length = std::max(view.toPixels(carLength), std::min(kCarPixels, spacing));
    const float width = std::max(view.toPixels(city.laneOffset()), kCarPixels);
    const bool vertical = axisOf(pose.heading) == Axis::NorthSouth;
    const Vector2 size = vertical ? Vector2{width, length} : Vector2{length, width};
    DrawRectangleV({center.x - size.x / 2, center.y - size.y / 2}, size, color);
}

// How far the followed car's radio reaches. Cars inside it are drawn yellow.
void drawRadioRange(const View &view, const Car &car, double range) {
    const Vector2 center = view.toScreen(car.message().position);
    DrawCircleLinesV(center, view.toPixels(range), SKYBLUE);
}

void drawStatus(const Car &followed, const Traffic &traffic, int carCount, std::size_t heard) {
    const CarState &car = followed.state();
    const Plan &plan = followed.plan();
    const char *status = "cruising";
    if (plan.status == PlanStatus::StopFree) status = "stop-free";
    if (plan.status == PlanStatus::ReducedMargin) status = "stop-free (tight)";
    if (plan.status == PlanStatus::Fallback) status = "waiting for a green";
    if (followed.arrived()) status = "parked";
    DrawText(TextFormat("%.1f m/s  %s  hears %d, %d on its route, %d reroutes", car.speed, status,
                        static_cast<int>(heard), static_cast<int>(followed.carsAhead()),
                        followed.reroutes()),
             14, 8, kFontSize, DARKGRAY);
    const TrafficStats &stats = traffic.stats();
    const double average = stats.tripsFinished > 0 ? stats.tripSeconds / stats.tripsFinished : 0.0;
    DrawText(TextFormat("%d cars, %d trips done, %.1f s per trip, %d stops", carCount,
                        stats.tripsFinished, average, stats.stops),
             14, 8 + kFontSize + 4, kFontSize, DARKGRAY);
}

}  // namespace

// Usage: city [seed] [cars]
// Without a seed every run is a new city; the seed is printed to replay it.
// Without a number of cars the city gets a busy number for its size.
// The red car is the one whose route and radio range are drawn.
int main(int argc, char **argv) {
    const unsigned seed = argc > 1 ? static_cast<unsigned>(std::strtoul(argv[1], nullptr, 10))
                                   : std::random_device{}();
    std::mt19937 rng(seed);

    const City city(randomCityConfig(rng));
    const PlannerConfig planner = tripPlannerConfig();
    TrafficConfig config;
    config.cars = argc > 2 ? std::max(std::atoi(argv[2]), 1) : busyCarCount(city);
    std::printf("seed %u: %d x %d streets, %d cars\n", seed, city.config().cols, city.config().rows,
                config.cars);
    std::optional<Traffic> created;
    try {
        created.emplace(city, planner, config, rng);
    } catch (const std::invalid_argument &error) {
        std::fprintf(stderr, "city: %s\n", error.what());
        return EXIT_FAILURE;
    }
    Traffic &traffic = *created;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(kWindowWidth, kWindowHeight, "City Traffic");
    if (!IsWindowReady()) {
        std::fprintf(stderr, "city: could not open a window. Is there a display?\n");
        return EXIT_FAILURE;
    }
    const int monitor = GetCurrentMonitor();
    SetWindowSize(std::min(kWindowWidth, static_cast<int>(GetMonitorWidth(monitor) * kScreenShare)),
                  std::min(kWindowHeight, static_cast<int>(GetMonitorHeight(monitor) * kScreenShare)));
    SetTargetFPS(60);

    double pending = 0.0;
    while (!WindowShouldClose()) {
        pending += std::min(static_cast<double>(GetFrameTime()), kMaxFrameTime);
        for (; pending >= kStep; pending -= kStep) traffic.step(kStep);

        const View view = fitToWindow(city, static_cast<float>(GetScreenWidth()),
                                      static_cast<float>(GetScreenHeight()));
        const Car &followed = traffic.cars().front();
        const Vec2 here = followed.message().position;
        BeginDrawing();
        ClearBackground(RAYWHITE);
        drawStreets(city, view);
        if (!followed.arrived()) drawPath(followed.route(), view, followed.state().position);
        drawEndMarker(followed.route(), view);
        drawTrafficLights(city, view, traffic.time());
        for (std::size_t i = traffic.cars().size(); i-- > 1;) {
            const Car &car = traffic.cars()[i];
            const Vec2 apart = car.message().position - here;
            const bool heard = dot(apart, apart) <= config.radioRange * config.radioRange;
            drawCar(city, view, car, planner.carLength, heard ? YELLOW : ORANGE);
        }
        drawCar(city, view, followed, planner.carLength, RED);
        drawRadioRange(view, followed, config.radioRange);
        drawStatus(followed, traffic, config.cars, traffic.inbox(0).size());
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
