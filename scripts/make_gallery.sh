#!/bin/sh
# Regenerate the drawings in docs/img from the examples. Run from the repository root after `make`:
#
#   sh scripts/make_gallery.sh
#
# Every file is produced by `mech render`; nothing is drawn by hand.
set -e
M=./mech
O=docs/img
mkdir -p "$O"
$M render examples/fourbar.mech   --frames 48 --span 10 --speed 2 --trace coupler.B -o $O/fourbar.svg
$M render examples/camstar.mech   --at 0.4  -o $O/camstar.svg
$M render examples/ratchet.mech   --at 0.6  -o $O/ratchet.svg
$M render examples/jumpcam.mech   --at 0.09 -o $O/jumpcam.svg
$M render examples/clock.mech     --at 20   -o $O/clock.svg
$M render examples/planetary.mech --at 1    -o $O/planetary.svg
$M render examples/ellipgears.mech --frames 24 --span 6 -o $O/ellipgears.svg
$M render examples/boom.mech        --frames 36 --span 6 --trace boom.tip -o $O/boom.svg
$M render examples/quickreturn.mech --frames 36 --span 6 --trace lever.slot -o $O/quickreturn.svg
$M render examples/scotchyoke.mech  --frames 36 --span 4 -o $O/scotchyoke.svg
$M render examples/slidingbead.mech --at 0.8 --span 0.8 --trace bead.g -o $O/slidingbead.svg
$M render examples/rackpinion.mech  --frames 36 --span 3 -o $O/rackpinion.svg
$M render examples/beltdrive.mech   --at 1 -o $O/beltdrive.svg
$M render examples/rollingwheel.mech --frames 24 --span 0.6 -o $O/rollingwheel.svg
