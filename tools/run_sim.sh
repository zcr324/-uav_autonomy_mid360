#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WS/env_uav_ws.bash"
if [[ -n "$(ss -H -ltn 'sport = :4560')" ]]; then
  echo "TCP 4560 is still occupied. Stop the existing simulator and wait for exit before starting PX4."
  exit 1
fi
mkdir -p "$WS/integration/logs"
echo $$ >"$WS/integration/logs/simulation.pid"
export GAZEBO_MODEL_DATABASE_URI=""
exec roslaunch sim_env autonomy.launch "$@"
