---
name: run-city
description: Build, run, and drive the city traffic simulation. Use when asked to start or run the city app, build it, run its tests, take a screenshot of its window, or check that a change to the cars, lights, planner or graphics works in the real app.
---

A raylib desktop app that simulates cars in a grid city, plus a headless core
(`citycore`) with tests. Drive the window with
`.claude/skills/run-city/driver.sh`, which runs it under xvfb and screenshots it.
The app has no controls: it is watched, not clicked, so a change shows up in the
screenshots and in the two status lines at the top of the window.

All paths below are relative to the project root.

## Prerequisites

Verified on Arch Linux, where the packages are `raylib`, `xorg-server-xvfb` and
`imagemagick` (for `import`), plus `cmake` and `g++`. Check they are there:

```bash
pkg-config --modversion raylib && which xvfb-run import cmake g++
```

Without raylib the `city` target is skipped silently and everything else builds.

## Build

```bash
cmake -S . -B build && cmake --build build -j8
```

## Run (agent path)

```bash
.claude/skills/run-city/driver.sh --seed 7 --shots 3 --every 6 --out /tmp/city-shots
```

It prints the city's size and car count, and fails if the app dies or if every
screenshot is identical. Screenshots land in `<out>/shot-N.png` and the app's log in
`<out>/city.log`. **Read the screenshots**: the driver only proves that something
moved. Options: `--seed N`, `--cars N` (default is a busy number for the city),
`--shots N`, `--every SECONDS`, `--out DIR`.

What to look for in a frame:
- Red car: the one followed, with its route in light blue and its radio range as a
  circle. Yellow cars are in range, orange ones are not.
- First status line: speed, plan, cars heard, cars on its route, reroutes.
- Second status line: cars, trips done, seconds per trip, stops.

To see queues rather than free-flowing traffic, use a small city with many cars:

```bash
.claude/skills/run-city/driver.sh --seed 7 --cars 200 --shots 2 --every 15 --out /tmp/city-jam
```

## Direct invocation (no window)

Most changes are to the traffic core and can be judged without the app.
`traffic_check [cities per row] [simulated seconds]` compares the model with parts
turned off, and exits non-zero if the full model runs a red:

```bash
cd build && ./test_traffic && ./traffic_check 1 30
```

## Run (human path)

```bash
./build/city 7 200
```

Arguments are `[seed] [cars]`. Without a seed, each run is a new city and the seed
is printed. Close the window to quit. Verified on a physical 1920 x 1080 display.

## Test

```bash
cd build && ctest
```

Takes about 12 minutes, nearly all of it `traffic_check` at its defaults. While
working, run the fast suites instead:

```bash
cd build && ctest -E traffic_check
```

## Gotchas

- With no display the app cannot open a window and exits with code 1. Always use the
  driver when headless.
- Screenshots are taken of the xvfb screen, which is 1600x1100, and the window takes
  95% of that, so black bands at the right and bottom are not a rendering bug.
- PNGs of a few KB are normal, as the frames are flat colors. Size says nothing about
  whether a frame is blank, so look at it.
- `import` has to run on the same display as the app. Two separate `xvfb-run` calls
  get two different displays, which is why the driver does both in one.
- `sim_check` writes `sim_trace.csv` into the current directory. Run it from `build/`.

## Troubleshooting

- `city: Traffic: too many cars for this city`, exit 1: the cars do not fit the
  city. Seed 7 is 3 x 4 streets and refuses 5000. Lower `--cars`.
- `city: could not open a window. Is there a display?`, exit 1: use the driver.
- `traffic_check` taking many minutes under `ctest`: it runs its defaults there. Run
  it directly with small arguments, as above.
