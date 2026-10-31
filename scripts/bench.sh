#!/bin/sh
# Timing of the simulation core on a few representative mechanisms, plus a chain of N links to show how the
# cost grows with size. Run from the repository root after `make`:
#
#   sh scripts/bench.sh            # prints seconds of computing per mechanism
#
# It also checks that the results are unchanged when you compare against a saved baseline:
#   sh scripts/bench.sh save base  # writes bench-base/*.csv
#   sh scripts/bench.sh check base # runs again and compares byte for byte
set -e
M=./mech
mode=${1:-time}; tag=${2:-base}
dir=bench-$tag
chain() {   # chain N : an N-link pendulum, dynamic
  cat <<EOT
mechanism chain {
  world { gravity: 9.81  step: 0.001 }
  ground frame { anchor pivot at (0, 0) }
  for i = 0 to $1 - 1 { rod link[i] { length: 0.05  mass: 0.05 } }
  pin link[0].A -> frame.pivot
  for i = 1 to $1 - 1 { pin link[i].A -> link[i - 1].B }
}
EOT
}
tmp=$(mktemp -d)
for n in 20 40 80; do chain $n > $tmp/chain$n.mech; done
case "$mode" in save|check) mkdir -p "$dir" ;; esac
run() {   # run name file seconds
  name=$1; file=$2; secs=$3
  start=$(date +%s.%N)
  $M run "$file" --headless "$secs" --csv "$tmp/$name.csv" > /dev/null 2>&1
  end=$(date +%s.%N)
  printf '%-14s %6.2f s for %s s simulated\n' "$name" "$(echo "$end - $start" | bc)" "$secs"
  case "$mode" in
    save) cp "$tmp/$name.csv" "$dir/$name.csv" ;;
    check) cmp -s "$tmp/$name.csv" "$dir/$name.csv" && echo "               identical to the baseline" || echo "               DIFFERS from the baseline" ;;
  esac
}
run geneva    examples/geneva.mech    3
run ratchet   examples/ratchet.mech   3
run jumpcam   examples/jumpcam.mech   1
run clock     examples/clock.mech     20
run chain20   $tmp/chain20.mech       1
run chain40   $tmp/chain40.mech       1
run chain80   $tmp/chain80.mech       1
rm -rf "$tmp"
