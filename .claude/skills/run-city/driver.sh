#!/usr/bin/env bash
# Launches the city app under xvfb, screenshots it over time, and fails unless the
# app stayed alive and the picture changed. Run from the project root.
#
# Usage: driver.sh [--seed N] [--cars N] [--shots N] [--every SECONDS] [--out DIR]
set -euo pipefail

seed=7 cars="" shots=3 every=6 out=/tmp/city-shots
while [ $# -gt 0 ]; do
    case "$1" in
        --seed) seed=$2 ;;
        --cars) cars=$2 ;;
        --shots) shots=$2 ;;
        --every) every=$2 ;;
        --out) out=$2 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift 2
done

[ -x build/city ] || { echo "build/city is missing: build first, from the project root" >&2; exit 2; }
mkdir -p "$out"
rm -f "$out"/shot-*.png "$out"/city.log

# Everything has to happen inside one display, so it is all in this inner script.
xvfb-run -a -s "-screen 0 1600x1100x24" bash -s -- "$seed" "$cars" "$shots" "$every" "$out" <<'INNER'
seed=$1 cars=$2 shots=$3 every=$4 out=$5
build/city "$seed" $cars > "$out/city.log" 2>&1 &
app=$!
for i in $(seq 1 "$shots"); do
    sleep "$every"
    if ! kill -0 "$app" 2>/dev/null; then
        wait "$app"; echo "FAIL: the app exited early with code $? before shot $i" >&2
        tail -5 "$out/city.log" >&2
        exit 1
    fi
    import -window root "$out/shot-$i.png"
done
kill "$app"
wait "$app" 2>/dev/null || true
INNER

distinct=$(sha256sum "$out"/shot-*.png | cut -d' ' -f1 | sort -u | wc -l)
if [ "$distinct" -lt 2 ] && [ "$shots" -ge 2 ]; then
    echo "FAIL: all $shots screenshots are identical, so nothing moved" >&2
    exit 1
fi
head -1 "$out/city.log"
echo "OK: $shots shots, $distinct distinct, in $out"
