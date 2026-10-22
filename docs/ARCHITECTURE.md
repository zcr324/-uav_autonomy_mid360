# UAV framework: module ownership and runtime contracts

## Runtime chain
Gazebo/PX4 sensors -> FAST-LIO (LTA-OM fork) -> bridge -> EGO -> control adapter -> communication -> PX4.
FAST-LIO registered clouds/poses -> STD -> GTSAM pose graph -> corrected path/map.

- Simulation: main framework plus leader's Driver2 MID360 model/plugin; original delivery retained.
- Localization: LTA-OM bundled FAST-LIO, because STD/backend rely on its correction interface. Leader's standalone FAST-LIO2 remains an independent baseline.
- Backend: existing LTA-OM code, teammate's Markdown handoff used to locate repairs; this machine's runs supply the verification.
- Planning: teammate's EGO packages; do not start their second simulator or px4ctrl in this chain.
- Framework: bridge, process lifecycle, profiles, mission ownership, readiness gates, route/result tools.

## Coordinate and command contracts
LiDAR/IMU relative extrinsic is zero. base_link -> sensor = (-.005,.005,.147).
Frontend camera_init -> map is fixed at initialization. Gazebo truth never enters the estimator/planner/control adapter. The experiment driver uses truth for evaluation, aborts and next-goal acceptance; deployment mission.py uses estimated odometry only.
Control computes fixed map -> PX4 local alignment while disarmed.
MAVROS converts ROS ENU to MAVLink NED; no extra manual axis flip.
PGO camera_init path/map are visualization/analysis outputs. Global correction into the flight control contract is not yet accepted.
LTA-OM can internally correct its map/front end after a loop; experiments must still watch odometry continuity and abort on excessive error.

Only communication publishes MAVROS raw local setpoints.
Takeoff and planner share its upstream Pose topic with explicit ownership:
1. disable /control_transfer/set_enabled before takeoff;
2. enable after takeoff, then send a new goal;
3. adapter waits for a new trajectory ID so an old trajectory cannot resume;
4. disable before landing; communication holds its last position target until mode changes.

## Profiles
config/experiments/profiles.json defines mapping, navigation, loop_observation and navigation_ev.
Scene definitions select world and initial alignment together.
start never arms. baseline and run are separate explicit operations.
experiment.py uses bounded wall time, gates localization, records raw data, checks mode/arming and lands after each route.
Map PCD captures from the frontend and backend must be labeled separately; only backend maps with verified accepted loops demonstrate loop correction.

## Result evidence
Each run directory contains launch.json, simulation.log and separate experiment directories.
Each experiment provides result.json, events.json, telemetry.csv, data.bag, map.pcd and overview.png.
Map points come from sensor reconstruction. The displayed static world outline and truth trajectory are evaluation references.
A published PGO path without accepted loop factors is only a backend-chain test.

## Height and time bounds
Global /framework/min_flight_z and max_flight_z default to 0.6 and 1.6m in modules.launch.
EGO's occupancy query treats heights outside these bounds as occupied. The command
adapter rejects out-of-envelope commands, and mission validates goal heights.
These bounds apply to active route tracking, not takeoff from the ground or landing.
The experiment also monitors actual height and localization error; it has a
1200-second default wall budget, per-stage timeouts and a bounded landing attempt.

## Offline backend verification
raw replay recomputes LIO/STD/PGO from recorded LiDAR and IMU.
submaps replay feeds exact recorded keyframe clouds and poses to STD/PGO,
without motion gating or another downsampling pass.
backend replay feeds the recorded submap/triangle/constraint interfaces to PGO.
All use an isolated master and preserve the live results.
STD's optional /std/refine_loop_icp (default false) activates its original
plane-to-plane geometric refinement for a separately labeled offline comparison.
No truth is supplied to these nodes. PGO overlap/residual checks remain unchanged.
