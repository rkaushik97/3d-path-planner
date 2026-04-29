#!/usr/bin/env python3
"""Plan a path with /plan_path and fly it on PX4 SITL via MAVSDK.

Flow: rclpy -> service call -> shutdown -> asyncio -> MAVSDK arm ->
seed OFFBOARD setpoints at climb altitude -> start OFFBOARD ->
stream remaining setpoints at 2 Hz -> land. ENU planner output is
rotated to NED setpoints (north = y, east = x, down = -z).

Auto-takeoff is intentionally skipped; OFFBOARD lifts the drone
directly to the first setpoint, avoiding the takeoff/OFFBOARD yaw
fight that causes spinning during the mode transition.

Run while planner_node, octomap_server, depth bridge, and PX4 SITL
are all up:

    docker exec -it px4_sitl bash -lc '
      source /opt/ros/jazzy/setup.bash &&
      source /root/3d-path-planner/ros2_ws/install/setup.bash &&
      python3 /root/3d-path-planner/scripts/run_mission.py'
"""

import asyncio
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

from planner_msgs.msg import NoFlyZone
from planner_msgs.srv import PlanPath


START = (0.0, 0.0, 5.0)
GOAL = (50.0, 0.0, 5.0)
ALT_MIN = 3.0
ALT_MAX = 8.0

# Prior-map building bounds, mirroring scripts/spawn_world.sh, padded by 2 m
# horizontally for drone half-span (~0.4 m) + tracking error in OFFBOARD mode.
NO_FLY_ZONES = [
    # tall_blocker — box (27..33, -3..3, 0..14)
    ((25.0, -5.0, 0.0), (35.0, 5.0, 16.0)),
    # mid_n1 — box (9.5..14.5, 7.5..12.5, 0..8)
    (( 7.5,  5.5, 0.0), (16.5, 14.5, 10.0)),
    # mid_s1 — box (9.5..14.5, -12.5..-7.5, 0..8)
    (( 7.5, -14.5, 0.0), (16.5, -5.5, 10.0)),
    # low_n2 — box (43..47, 8..12, 0..4)
    ((41.0,  6.0, 0.0), (49.0, 14.0,  6.0)),
    # low_s2 — box (43..47, -12..-8, 0..4)
    ((41.0, -14.0, 0.0), (49.0, -6.0,  6.0)),
]


def _make_zones() -> list[NoFlyZone]:
    out = []
    for (mn, mx) in NO_FLY_ZONES:
        z = NoFlyZone()
        z.min_corner.x, z.min_corner.y, z.min_corner.z = mn
        z.max_corner.x, z.max_corner.y, z.max_corner.z = mx
        out.append(z)
    return out


def request_plan() -> Path:
    rclpy.init()
    node = Node('mission_executor_planner_client')
    client = node.create_client(PlanPath, '/plan_path')
    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Waiting for /plan_path service...')

    req = PlanPath.Request()
    req.start = PointStamped()
    req.start.header.frame_id = 'map'
    req.start.point.x, req.start.point.y, req.start.point.z = START
    req.goal = PointStamped()
    req.goal.header.frame_id = 'map'
    req.goal.point.x, req.goal.point.y, req.goal.point.z = GOAL
    req.altitude_min = ALT_MIN
    req.altitude_max = ALT_MAX
    req.no_fly_zones = _make_zones()

    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    response = fut.result()
    rclpy.shutdown()

    if not response.success:
        print(f'Plan failed: {response.message}', file=sys.stderr)
        sys.exit(1)

    print(f'Plan: {response.message}')
    return response.path


def path_to_ned_setpoints(path: Path) -> list[PositionNedYaw]:
    """ENU planner -> NED PX4: north=y, east=x, down=-z. Yaw fixed to 0."""
    return [
        PositionNedYaw(p.pose.position.y, p.pose.position.x, -p.pose.position.z, 0.0)
        for p in path.poses
    ]


def subsample(setpoints: list, target_count: int = 25) -> list:
    if len(setpoints) <= target_count:
        return setpoints
    step = len(setpoints) // target_count
    out = setpoints[::step]
    if out[-1] is not setpoints[-1]:
        out.append(setpoints[-1])
    return out


async def fly_path(setpoints: list[PositionNedYaw]) -> None:
    drone = System()
    print('Connecting to PX4 on udpin://0.0.0.0:14540 ...')
    await drone.connect(system_address='udpin://0.0.0.0:14540')
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print('Connected.')

    print('Waiting for global position + home estimate ...')
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    print('Position estimate OK.')

    print('Arming ...')
    await drone.action.arm()

    first = setpoints[0]
    print(f'Seeding OFFBOARD setpoint  N={first.north_m:.1f} E={first.east_m:.1f} D={first.down_m:.1f} ...')
    for _ in range(20):
        await drone.offboard.set_position_ned(first)
        await asyncio.sleep(0.05)

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f'OFFBOARD start failed: {e}', file=sys.stderr)
        await drone.action.disarm()
        sys.exit(1)
    print('OFFBOARD active. Climbing to first setpoint ...')

    await asyncio.sleep(8.0)

    # Slower stream — 1 setpoint per 1.5 s — gives the drone time to actually
    # arrive at each waypoint before the next one is commanded. Tracking lag
    # is what was clipping the obstacle corners.
    print(f'Streaming {len(setpoints) - 1} more setpoints @ ~0.7 Hz ...')
    for i, sp in enumerate(setpoints[1:], start=1):
        await drone.offboard.set_position_ned(sp)
        print(f'  {i:>3}/{len(setpoints) - 1}  N={sp.north_m:6.2f} E={sp.east_m:6.2f} D={sp.down_m:6.2f}')
        await asyncio.sleep(1.5)

    print('Holding final setpoint for 5 s ...')
    for _ in range(10):
        await drone.offboard.set_position_ned(setpoints[-1])
        await asyncio.sleep(0.5)

    print('Stopping OFFBOARD and landing ...')
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print('Landed.')


def main() -> None:
    path = request_plan()
    print(f'Path has {len(path.poses)} waypoints (raw).')
    setpoints = subsample(path_to_ned_setpoints(path))
    print(f'Flying {len(setpoints)} subsampled setpoints.')
    asyncio.run(fly_path(setpoints))


if __name__ == '__main__':
    main()
