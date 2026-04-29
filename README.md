# 3d-path-planner

ROS2 global path planner for a UAV in a 3D Gazebo environment. Builds an
OctoMap from an onboard depth camera, plans collision-free paths with
**A\*** on a voxel grid (honouring **altitude bounds** and **no-fly
zones**), and executes the plan on **PX4 SITL** via MAVSDK OFFBOARD.

Stack: ROS2 Jazzy, Gazebo Harmonic, PX4-Autopilot, OctoMap, MAVSDK.
GPU-aware Docker image; everything runs end-to-end inside one container.

---

## Demo

<video src="media/Path_Planning_Demo.mp4" controls width="720"></video>

Full end-to-end run: [`media/Path_Planning_Demo.mp4`](media/Path_Planning_Demo.mp4).

The drone takes off from the **green sphere** (start marker at the
origin) and navigates the A\*-planned route to the **red sphere** (goal
marker, 50 m east), detouring ~5 m south around the tall red building
along the way. The drone does not first fly *to* the green sphere —
the sphere just marks where it spawns. The eight Docker terminals
driving PX4, the gz↔ROS2 bridges, the planner, and the mission
executor are visible in the recording.

---

## Architecture

```
                              Gazebo Harmonic
                              ┌──────────────────────┐
                              │  x500_depth airframe │
                              │  └─ depth camera     │
                              └──────────┬───────────┘
                                         │ gz: /depth_camera/points
                                         ▼
                              ┌──────────────────────┐
                              │  ros_gz_bridge       │
                              │  (gz → ROS2)         │
                              └──────────┬───────────┘
                                         │ sensor_msgs/PointCloud2
                                         ▼
                              ┌──────────────────────┐
                              │  octomap_server      │
                              └──────────┬───────────┘
                                         │ /octomap_point_cloud_centers
                                         ▼
   ┌──────────────────────┐   ┌──────────────────────┐    /plan_path
   │ static_transform_pub │──▶│  global_planner      │ ◀──── service
   │   map → camera_link  │   │  • inflated grid     │      (run_mission.py)
   └──────────────────────┘   │  • A* (26-conn)      │
                              │  • alt bounds        │
                              │  • no-fly zones      │
                              └──────────┬───────────┘
                              /global_plan │ /no_fly_zones
                                         ▼
                              ┌──────────────────────┐
                              │  RViz2               │
                              └──────────────────────┘

   ┌──────────────────────┐
   │  run_mission.py      │   ENU planner path → NED setpoints
   │  • calls /plan_path  │ ────────────────────────────────────▶ PX4 SITL
   │  • MAVSDK OFFBOARD   │   (UDP 14540, OFFBOARD position)         │
   └──────────────────────┘                                          │
                                                                     ▼
                                                                  Gazebo
```

---

## Features

- **3D A\* planner** on a voxel grid keyed off the live OctoMap.
  26-connectivity, Euclidean heuristic, ~5s typical plan time over a
  50 m x 30 m x 15 m volume at 0.2 m resolution.
- **`PlanPath` service** with start, goal, altitude bounds, and an
  array of axis-aligned no-fly zones (custom `NoFlyZone.msg`).
- **OctoMap** built live from the onboard depth camera via
  `ros_gz_bridge` + `octomap_server`.
- **No-fly zones** rendered as translucent red cubes in RViz2 so what
  the planner is forbidden from is visually obvious.
- **PX4 OFFBOARD execution** via MAVSDK Python — no auto-takeoff (the
  drone climbs straight to the first setpoint, which avoids the
  takeoff/OFFBOARD yaw fight that otherwise spins the airframe).
- **Reproducible scene** spawned at runtime via `gz service` calls
  (Fuel-model `<include>` blocks in the world SDF break PX4 startup).
- **Single GPU-enabled Docker image** layered on top of the upstream
  `erdemuysalx/px4-sitl` base.

---

## Prerequisites

- **Docker** (Engine 24+ with Compose v2)
- **NVIDIA GPU + driver + NVIDIA Container Toolkit** (only required for
  GPU-accelerated rendering; CPU will work but Gazebo is slow)
- ~10 GB disk for the image + PX4 + Fuel cache

The base image exposes a noVNC desktop on port **6080** — the Gazebo
GUI and RViz2 windows are usable in any browser, no host X server
needed.

---

## Setup

Clone both repos as siblings, layer this repo's GPU + bind-mount config
onto px4-sim's compose, build the image, then build the ROS2 workspace
inside the container. Four shell commands.

```bash
# 1. Clone both repos as siblings (any parent directory works).
mkdir -p ~/project && cd ~/project
git clone https://github.com/rkaushik97/3d-path-planner.git
git clone https://github.com/erdemuysalx/px4-sim.git

# 2. Layer this repo's compose override onto px4-sim's compose.
cd ~/project/px4-sim
ln -s ../3d-path-planner/docker/docker-compose.override.yml .

# 3. Build the image and start the container.
docker compose build
docker compose up -d

# 4. Build the ROS2 workspace inside the container (~7 s).
docker exec -it px4_sitl bash -lc \
  'source /opt/ros/jazzy/setup.bash &&
   cd /root/3d-path-planner/ros2_ws &&
   colcon build'
```

Verify the container is healthy and dependencies loaded:

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  python3 -c "from mavsdk import System; print(\"mavsdk OK\")" &&
  dpkg -l | grep -E "octomap|libompl" | awk "{print \$2}" &&
  nvidia-smi -L
'
```

Expected:

```
mavsdk OK
libompl-dev
libompl16t64:amd64
ros-jazzy-octomap
ros-jazzy-octomap-msgs
ros-jazzy-octomap-ros
ros-jazzy-octomap-rviz-plugins
ros-jazzy-octomap-server
GPU 0: NVIDIA H100 NVL (UUID: ...)
```

---

## Running the demo

Open **noVNC** at <http://localhost:6080> (or your server's IP) so the
Gazebo and RViz2 windows are visible. Then open eight `docker exec` shells
into the container in this order — each step assumes the previous ones
are still running.

### Terminal 1 — PX4 SITL with the planner world

The planner world is symlinked into PX4's worlds directory once, then
loaded via `PX4_GZ_WORLD`.

```bash
docker exec -it px4_sitl bash -lc \
  'ln -sf /root/3d-path-planner/worlds/planner_world.sdf \
          /root/PX4-Autopilot/Tools/simulation/gz/worlds/planner_world.sdf'

docker exec -it px4_sitl bash
cd /root/PX4-Autopilot
PX4_GZ_WORLD=planner_world make px4_sitl gz_x500_depth
```

Wait until you see `pxh>`. Sample tail:

```
INFO  [init] Starting gazebo with world: .../planner_world.sdf
INFO  [init] Gazebo world is ready
INFO  [gz_bridge] world: planner_world, model: x500_depth_0
INFO  [px4] Startup script returned successfully
pxh>
```

In the noVNC tab the quadcopter is now sitting on the gray ground plane.

### Terminal 2 — spawn obstacles

```bash
docker exec -it px4_sitl /root/3d-path-planner/scripts/spawn_world.sh
```

First run pulls the Fuel models (~15 s). Output:

```
Spawning tall_blocker...
data: true
Spawning mid_n1...
data: true
...
Spawn complete. 11 entities in /world/planner_world.
```

11 entities total: 5 colored boxes (tall red, two mid blue, two low
orange), 2 trees (pine + oak), 2 people, plus emissive green/red
spheres at the start and goal positions.

### Terminal 3 — bridge depth point cloud to ROS2

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 run ros_gz_bridge parameter_bridge \
    /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
'
```

Output:

```
[INFO] [...] [ros_gz_bridge]: Creating GZ->ROS Bridge:
  [/depth_camera/points (gz.msgs.PointCloudPacked) ->
   /depth_camera/points (sensor_msgs/msg/PointCloud2)]
```

Sanity check from another terminal:

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  timeout 3 ros2 topic hz /depth_camera/points
'
```

Expected `~5 Hz`.

### Terminal 4 — static transform map → camera_link

The drone is stationary during the OctoMap-build phase, so a single
static transform is sufficient. (Dynamic TF would be needed for an
in-flight survey; not in this version.)

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.2 \
    --frame-id map --child-frame-id camera_link
'
```

The publisher emits one transform and idles.

### Terminal 5 — octomap_server

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    -p frame_id:=map \
    -p resolution:=0.2 \
    -p sensor_model.max_range:=10.0 \
    -r cloud_in:=/depth_camera/points
'
```

Verify topics from another terminal:

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 topic list | grep -E "octomap|occupied"
'
```

Expected:

```
/occupied_cells_vis_array
/octomap_binary
/octomap_full
/octomap_point_cloud_centers
```

### Terminal 6 — global_planner node

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /root/3d-path-planner/ros2_ws/install/setup.bash &&
  ros2 run global_planner planner_node
'
```

Output:

```
[INFO] [global_planner]: global_planner ready. resolution=0.2 m,
       inflation=2 voxels (~0.40 m).
```

The node now subscribes to `/octomap_point_cloud_centers`, exposes
`/plan_path`, and publishes paths on `/global_plan` and zones on
`/no_fly_zones`.

### Terminal 7 — RViz2

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  ros2 run rviz2 rviz2
'
```

In the RViz2 window (visible in noVNC):

| Step | Action |
|---|---|
| 1 | "Global Options" → **Fixed Frame**: `map` |
| 2 | **Add → MarkerArray** → topic `/occupied_cells_vis_array` (octree voxels) |
| 3 | **Add → Path** → topic `/global_plan` (the planned route) |
| 4 | **Add → MarkerArray** → topic `/no_fly_zones` (red transparent boxes) |

> The `octomap_rviz_plugins/OccupancyGrid` display fails to load on
> Jazzy (ABI mismatch with the installed `octomap` library). The
> MarkerArray on `/occupied_cells_vis_array` is the supported workaround
> — same voxels, no plugin dependency.

### Terminal 8 — plan + fly the mission

```bash
docker exec -it px4_sitl bash -lc 'pkill -f mavsdk_server || true' && \
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /root/3d-path-planner/ros2_ws/install/setup.bash &&
  python3 /root/3d-path-planner/scripts/run_mission.py
'
```

The script calls `/plan_path` with start `(0,0,5)`, goal `(50,0,5)`,
altitude window `[3, 8] m`, and five no-fly zones around the box
buildings, then executes the plan via MAVSDK OFFBOARD.

Sample output:

```
Plan: Path found: 251 waypoints in 5341.4 ms.
Path has 251 waypoints (raw).
Flying 26 subsampled setpoints.
Connecting to PX4 on udpin://0.0.0.0:14540 ...
Connected.
Waiting for global position + home estimate ...
Position estimate OK.
Arming ...
Seeding OFFBOARD setpoint  N=0.1 E=0.1 D=-5.1 ...
OFFBOARD active. Climbing to first setpoint ...
Streaming 25 more setpoints @ ~0.7 Hz ...
    1/25  N=  0.10 E=  2.10 D= -5.10
    ...
   12/25  N= -5.10 E= 24.10 D= -5.10
   13/25  N= -5.10 E= 26.10 D= -5.10
   ...
   25/25  N=  0.10 E= 50.10 D= -5.10
Holding final setpoint for 5 s ...
Stopping OFFBOARD and landing ...
Landed.
```

In Gazebo, the drone arms, climbs to ~5 m, flies along +X, **detours
~5 m south around the tall red building** (north value drops to N=-5.1
mid-flight), and lands near the goal. In RViz2, the green path
visualises the same trajectory with the five red zones around the boxes.

---

## Custom interfaces

`planner_msgs/srv/PlanPath.srv`:

```
geometry_msgs/PointStamped start
geometry_msgs/PointStamped goal
float64 altitude_min
float64 altitude_max
planner_msgs/NoFlyZone[] no_fly_zones
---
nav_msgs/Path path
bool success
float64 planning_time_ms
string message
```

`planner_msgs/msg/NoFlyZone.msg`:

```
geometry_msgs/Point min_corner
geometry_msgs/Point max_corner
```

Manually triggering a plan from the CLI (any terminal):

```bash
docker exec -it px4_sitl bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /root/3d-path-planner/ros2_ws/install/setup.bash &&
  ros2 service call /plan_path planner_msgs/srv/PlanPath "{
    start: {header: {frame_id: map}, point: {x: 0.0, y: 0.0, z: 5.0}},
    goal:  {header: {frame_id: map}, point: {x: 50.0, y: 0.0, z: 5.0}},
    altitude_min: 3.0,
    altitude_max: 8.0,
    no_fly_zones: [
      {min_corner: {x: 25.0, y: -5.0, z: 0.0},
       max_corner: {x: 35.0, y:  5.0, z: 16.0}}
    ]
  }"
'
```

---

## Repository structure

```
3d-path-planner/
├── README.md
├── docker/
│   ├── Dockerfile                       # extends erdemuysalx/px4-sitl
│   └── docker-compose.override.yml      # GPU + bind-mount + port remap
├── ros2_ws/
│   └── src/
│       ├── planner_msgs/                # ament_cmake — service + msg
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── msg/NoFlyZone.msg
│       │   └── srv/PlanPath.srv
│       └── global_planner/              # ament_python — A* node
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           └── global_planner/planner_node.py
├── scripts/
│   ├── spawn_world.sh                   # populate Gazebo with the scene
│   ├── despawn_world.sh                 # tear it down (idempotent)
│   ├── test_mavsdk.py                   # MAVSDK ↔ PX4 link smoke test
│   └── run_mission.py                   # plan + execute on the drone
└── worlds/
    └── planner_world.sdf                # clean light-gray world for PX4
```

---

## Implementation notes

The non-obvious things that bit during development. The scripts and
Dockerfile already handle them — these notes explain *why* so anyone
reproducing the build understands the choices.

1. **`PX4_GZ_WORLD` must match `<world name="...">` in the SDF.** PX4
   waits for `/world/<PX4_GZ_WORLD>/...` services to come up; if the
   SDF's world name disagrees, the wait times out. `planner_world.sdf`
   uses `<world name="planner_world">` to match.
2. **`<include>` of Fuel models in the world SDF breaks PX4 startup.**
   Gazebo blocks the world from going ready while it downloads each
   model, and PX4 aborts. `spawn_world.sh` calls `gz service` at
   runtime instead.
3. **Compose merges `ports` lists rather than replacing them.** The
   override declares `ports: !override` to drop the inherited
   `5901:5901` mapping (the host already runs an `Xtigervnc` on 5901)
   and remap to `5902:5901`.
4. **Software-rendered OpenGL renders Fuel buildings black.** Fuel
   building / vehicle models use PBR materials that need a real GPU;
   the noVNC X session uses llvmpipe and they fall back to a flat
   gray/black material. Trees use simpler materials and survive. This
   is why the scene's buildings are colored primitive boxes spawned
   inline rather than Fuel models.
5. **`ros_gz_bridge` Pose_V → TFMessage drops frame names.** The bridge
   does not propagate per-pose `name` fields into `child_frame_id`, so
   `/tf` arrives with empty frames and is unusable for TF lookups. The
   workaround used here is a single static transform `map →
   camera_link` for the (stationary-during-mapping) drone. Dynamic TF
   from PX4 odometry is left for a future revision.
6. **`ros-jazzy-octomap-rviz-plugins` has an ABI mismatch with
   `ros-jazzy-octomap`.** The plugin fails with `undefined symbol:
   _ZTIN7octomap13OcTreeStampedE`. The workaround is to use the
   built-in `MarkerArray` display on `/occupied_cells_vis_array`,
   which `octomap_server` publishes specifically to bypass the plugin.
7. **MAVSDK requires `udpin://0.0.0.0:14540` on Jazzy.** The older
   `udp://:14540` is deprecated and the newer `udpin://` form requires
   an explicit interface (e.g. `0.0.0.0`).
8. **Skip auto-takeoff before OFFBOARD.** Calling
   `drone.action.takeoff()` and then switching to OFFBOARD causes a
   yaw fight (auto-takeoff doesn't control yaw, OFFBOARD with
   `yaw=0.0` snaps to north). Better: arm, seed OFFBOARD setpoints at
   the climb altitude, start OFFBOARD — the drone climbs straight up
   to the first setpoint with no rotation.
9. **OFFBOARD setpoints need pacing.** Streaming 60+ setpoints at 2 Hz
   means the drone is always lagging behind the commanded position;
   around obstacle corners that lag clipped buildings. The current
   script subsamples to ~25 setpoints at ~0.7 Hz so the drone has
   time to actually arrive before the next command.
10. **Files written inside the container are root-owned on the host.**
    The bind-mount preserves uid/gid and the container runs as `root`.
    Before host-side `git` operations: `sudo chown -R $USER:$USER
    ~/project/3d-path-planner`.

---

## Limitations

- **Initial OctoMap is partial.** The drone is stationary during
  mapping, so the live OctoMap only contains what the depth camera
  sees from home (~ground plane + close obstacles within ~10 m).
  Distant boxes and tree canopies are unobserved.
- **Known buildings are encoded as no-fly zones in the mission
  request**, not learned from the live OctoMap. This is the
  prior-map-plus-live-sensor pattern used in real deployments; the
  live OctoMap remains in the planner pipeline for genuinely unknown
  obstacles.
- **No dynamic TF.** The drone's pose during flight is not published
  to `/tf`, so RViz shows the path and zones but not the moving drone.
- **Fixed yaw of 0** (NED north) on all OFFBOARD setpoints. The drone
  flies sideways during eastward portions; visually fine, but a
  real flight would face the direction of motion.

---

## Acknowledgements

- **px4-sim** by [@erdemuysalx](https://github.com/erdemuysalx) — the
  Docker base image (ROS2 Jazzy + Gazebo Harmonic + PX4 SITL + noVNC).
- **PX4-Autopilot**, **OctoMap**, and **MAVSDK** for the core stack.
- **Open Robotics / Gazebo Fuel** for the tree and human models.

---

## License

Apache License 2.0.
