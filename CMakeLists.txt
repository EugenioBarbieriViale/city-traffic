cmake_minimum_required(VERSION 3.16)
project(CitySimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# Headless core: no graphics dependency, so tests and checks run anywhere.
add_library(citycore STATIC
    src/City.cpp
    src/IntervalSet.cpp
    src/Kinematics.cpp
    src/Route.cpp
    src/Simulation.cpp
    src/SpeedPlanner.cpp
    src/Traffic.cpp
    src/Trip.cpp
)
target_include_directories(citycore PUBLIC src)
find_package(Threads REQUIRED)
target_link_libraries(citycore PUBLIC Threads::Threads)

# The window is optional: without raylib everything else still builds.
find_package(raylib QUIET)
if(raylib_FOUND)
    add_executable(city app/main.cpp)
    target_link_libraries(city PRIVATE citycore raylib)
else()
    message(STATUS "raylib not found: skipping the 'city' GUI target")
endif()

enable_testing()

add_executable(test_interval_set tests/test_interval_set.cpp)
target_link_libraries(test_interval_set PRIVATE citycore)
add_test(NAME interval_set COMMAND test_interval_set)

add_executable(test_environment tests/test_environment.cpp)
target_link_libraries(test_environment PRIVATE citycore)
add_test(NAME environment COMMAND test_environment)

add_executable(test_speed_planner tests/test_speed_planner.cpp)
target_link_libraries(test_speed_planner PRIVATE citycore)
add_test(NAME speed_planner COMMAND test_speed_planner)

add_executable(test_trip tests/test_trip.cpp)
target_link_libraries(test_trip PRIVATE citycore)
add_test(NAME trip COMMAND test_trip)

add_executable(test_traffic tests/test_traffic.cpp)
target_link_libraries(test_traffic PRIVATE citycore)
add_test(NAME traffic COMMAND test_traffic)

add_executable(traffic_check tests/traffic_check.cpp)
target_link_libraries(traffic_check PRIVATE citycore)
add_test(NAME traffic_check COMMAND traffic_check)

add_executable(sim_check tests/sim_check.cpp)
target_link_libraries(sim_check PRIVATE citycore)
add_test(NAME simulation_check COMMAND sim_check)
