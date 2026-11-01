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
chain() {   # chain N [shuffled]: an N-link pendulum, dynamic, written out (a loop is limited to 1000 iterations); shuffled
            # declares the links and the pins in random order, which the solver has to see through
  echo "mechanism chain {"
  echo "  world { gravity: 9.81  step: 0.001 }"
  echo "  ground frame { anchor pivot at (0, 0) }"
  if [ -n "$2" ]; then order="shuf --random-source=/dev/zero"; else order="cat"; fi
  seq 0 $(($1 - 1)) | $order | while read i; do echo "  rod link_$i { length: 0.05  mass: 0.05 }"; done
  { echo "  pin link_0.A -> frame.pivot"; seq 1 $(($1 - 1)) | while read i; do echo "  pin link_$i.A -> link_$(($i - 1)).B"; done; } | $order
  echo "}"
}
tmp=$(mktemp -d)
for n in 20 40 80 320 1280; do chain $n > $tmp/chain$n.mech; done
chain 320 shuffled > $tmp/shuf320.mech
case "$mode" in save|check) mkdir -p "$dir" ;; esac
run() {   # run name file seconds
  name=$1; file=$2; secs=$3
  start=$(date +%s.%N)
  $M run "$file" --headless "$secs" --csv "$tmp/$name.csv" > /dev/null 2>&1
  end=$(date +%s.%N)
  printf '%-14s %6.2f s for %s s simulated\n' "$name" "$(echo "$end - $start" | bc)" "$secs"
  # a zero that is exactly -0 in one build and +0 in another is the same number: compare with the sign of zero removed
  norm() { sed -e 's/,-0\(,\|$\)/,0\1/g' -e 's/,-0\(,\|$\)/,0\1/g' "$1"; }
  case "$mode" in
    save) norm "$tmp/$name.csv" > "$dir/$name.csv" ;;
    check) norm "$tmp/$name.csv" > "$tmp/$name.norm"; cmp -s "$tmp/$name.norm" "$dir/$name.csv" && echo "               identical to the baseline" || echo "               DIFFERS from the baseline" ;;
  esac
}
run geneva    examples/geneva.mech    3
run ratchet   examples/ratchet.mech   3
run jumpcam   examples/jumpcam.mech   1
run clock     examples/clock.mech     20
run chain20   $tmp/chain20.mech       1
run chain40   $tmp/chain40.mech       1
run chain80   $tmp/chain80.mech       1
run chain320  $tmp/chain320.mech      0.3
run shuf320   $tmp/shuf320.mech       0.3
run chain1280 $tmp/chain1280.mech     0.1
rm -rf "$tmp"
