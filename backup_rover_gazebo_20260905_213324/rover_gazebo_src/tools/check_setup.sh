#!/usr/bin/env bash
# Sixty-second acceptance check. Run this first, on the machine that will run the sim.
#
# It answers one question: can this Gazebo enforce the suspension coupling in its
# solver, or do you need the fallback? Everything else about the rover depends on
# getting that right, so it is worth the minute.

set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(dirname "$HERE")"
if [ ! -d "$PKG/urdf" ]; then
  # Installed layout: the script is in lib/<pkg>/, the data in share/<pkg>/. Ask the index.
  PKG="$(python3 -c 'from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory("rover_gazebo"))' 2>/dev/null || echo "$PKG")"
fi
fail=0

say() { printf '%-34s %s\n' "$1" "$2"; }

echo "== tools =="
for t in xacro ros2; do
  if command -v "$t" >/dev/null; then say "$t" "found"; else say "$t" "MISSING"; fail=1; fi
done
GZ=""
for t in gz ign; do command -v $t >/dev/null && GZ=$t && break; done
GZVER=$({ $GZ sim --version; $GZ gazebo --version; $GZ --version; } 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
if [ -n "$GZ" ]; then say "gazebo tool" "$GZ ${GZVER:-version unknown}"
else say "gazebo tool" "MISSING"; fail=1; fi

echo
echo "== the model builds =="
URDF=$(mktemp /tmp/rover_check_XXXX.urdf)
if xacro "$PKG/urdf/rover.urdf.xacro" > "$URDF" 2>/tmp/rover_check_err; then
  say "xacro" "ok, $(grep -c '<link' "$URDF") links"
  say "mimic tags in the URDF" "$(grep -c '<mimic joint=' "$URDF")"
else
  say "xacro" "FAILED"
  if grep -q "PackageNotFoundError" /tmp/rover_check_err; then
    echo "    The package is not on the ROS search path. Build it and source the"
    echo "    workspace, then run this again:"
    echo "        colcon build --packages-select rover_gazebo"
    echo "        source install/setup.bash"
  else
    sed 's/^/    /' /tmp/rover_check_err
  fi
  fail=1
fi

echo
echo "== can this Gazebo enforce the coupling? =="
python3 "$HERE/make_sdf.py" --check-only
rc=$?

echo
if [ $rc -eq 0 ]; then
  cat <<'MSG'
This Gazebo understands mimic constraints. Two things left to check, in order:

  1. Does the URDF converter carry them through? Run
         tools/make_sdf.py
     If it reports that some survived unaided, the plain launch is enough:
         ros2 launch rover_gazebo rover_sim.launch.py
     If it reports zero survived, it has just written a model with them put back in.
     Launch that instead, with the command it printed.

  2. Mimic constraints need the DART physics engine, which is the default. If your
     world selects another engine, they will be ignored silently.
MSG
else
  cat <<'MSG'
This Gazebo cannot enforce the coupling in its solver. Use the fallback, which applies
the same constraints as joint torques and works on any version:

    ros2 launch rover_gazebo rover_sim.launch.py coupling:=controller

Read doc/VERIFICATION.md for what that path does and does not hold to.
MSG
fi

rm -f "$URDF" /tmp/rover_check_err
exit $fail
