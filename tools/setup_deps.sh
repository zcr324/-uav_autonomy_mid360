#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for p in plan_env path_searching bspline_opt traj_utils plan_manage; do
  test -f "$WS/repos/ego-planner-team/planner/$p/package.xml"
  ln -sfn "../repos/ego-planner-team/planner/$p" "$WS/src/$p"
done
test -f "$WS/repos/ego-planner-team/quadrotor_msgs/package.xml"
ln -sfn ../repos/ego-planner-team/quadrotor_msgs "$WS/src/quadrotor_msgs"
echo "Team planner links restored; run tools/build_integration.sh"
