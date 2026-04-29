"""3D global path planner.

Subscribes to /octomap_point_cloud_centers, maintains an inflated
occupancy set, and exposes a /plan_path service backed by A* on a
26-connected voxel grid. Honours altitude bounds and an array of
axis-aligned no-fly zones. Publishes the resulting path on
/global_plan and the zones on /no_fly_zones for RViz2.
"""

import heapq
import time
from math import sqrt

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from planner_msgs.msg import NoFlyZone
from planner_msgs.srv import PlanPath


_NEIGHBORS_26 = [
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
]


def _euclidean(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def a_star_3d(start, goal, blocked, alt_bounds, max_iters=300_000):
    """A* on integer voxel indices with 26-connectivity.

    blocked: set of (i,j,k) treated as obstacles (already inflated + zones).
    alt_bounds: (k_min, k_max) inclusive — voxel-index altitude window.
    """
    if start in blocked or goal in blocked:
        return None

    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}

    iters = 0
    while open_heap:
        iters += 1
        if iters > max_iters:
            return None

        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        for dx, dy, dz in _NEIGHBORS_26:
            nxt = (current[0] + dx, current[1] + dy, current[2] + dz)
            if nxt in blocked:
                continue
            if not (alt_bounds[0] <= nxt[2] <= alt_bounds[1]):
                continue

            step = sqrt(dx * dx + dy * dy + dz * dz)
            tentative = g_score[current] + step
            if tentative < g_score.get(nxt, float('inf')):
                came_from[nxt] = current
                g_score[nxt] = tentative
                heapq.heappush(open_heap, (tentative + _euclidean(nxt, goal), nxt))

    return None


class PlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('inflation_radius_voxels', 2)
        self.resolution = self.get_parameter('resolution').value
        self.inflation_radius = int(self.get_parameter('inflation_radius_voxels').value)

        self.occupied: set = set()
        self._inflated: set | None = None
        self.octomap_frame = 'map'

        self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self._octomap_cb,
            10,
        )
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.zone_pub = self.create_publisher(MarkerArray, '/no_fly_zones', 10)
        self.create_service(PlanPath, '/plan_path', self._plan_path_cb)

        self.get_logger().info(
            f'global_planner ready. resolution={self.resolution} m, '
            f'inflation={self.inflation_radius} voxels '
            f'(~{self.inflation_radius * self.resolution:.2f} m).'
        )

    def _octomap_cb(self, msg: PointCloud2) -> None:
        struct = np.array(
            list(point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))
        )
        if struct.size == 0:
            return
        xyz = np.stack([struct['x'], struct['y'], struct['z']], axis=-1)
        indices = np.floor(xyz / self.resolution).astype(np.int32)
        self.occupied = set(map(tuple, indices))
        self._inflated = None
        self.octomap_frame = msg.header.frame_id

    def _get_inflated(self) -> set:
        if self._inflated is not None:
            return self._inflated
        r = self.inflation_radius
        offsets = [
            (di, dj, dk)
            for di in range(-r, r + 1)
            for dj in range(-r, r + 1)
            for dk in range(-r, r + 1)
        ]
        inflated = set()
        for (i, j, k) in self.occupied:
            for di, dj, dk in offsets:
                inflated.add((i + di, j + dj, k + dk))
        self._inflated = inflated
        return inflated

    def _world_to_index(self, x, y, z):
        return (
            int(np.floor(x / self.resolution)),
            int(np.floor(y / self.resolution)),
            int(np.floor(z / self.resolution)),
        )

    def _index_to_world(self, idx):
        return (
            (idx[0] + 0.5) * self.resolution,
            (idx[1] + 0.5) * self.resolution,
            (idx[2] + 0.5) * self.resolution,
        )

    def _zones_to_indices(self, zones: list[NoFlyZone]) -> set:
        result = set()
        res = self.resolution
        for z in zones:
            i_min, i_max = sorted([int(np.floor(z.min_corner.x / res)),
                                    int(np.floor(z.max_corner.x / res))])
            j_min, j_max = sorted([int(np.floor(z.min_corner.y / res)),
                                    int(np.floor(z.max_corner.y / res))])
            k_min, k_max = sorted([int(np.floor(z.min_corner.z / res)),
                                    int(np.floor(z.max_corner.z / res))])
            for i in range(i_min, i_max + 1):
                for j in range(j_min, j_max + 1):
                    for k in range(k_min, k_max + 1):
                        result.add((i, j, k))
        return result

    def _publish_zone_markers(self, zones: list[NoFlyZone]) -> None:
        arr = MarkerArray()
        # Clear previous markers first.
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for i, z in enumerate(zones):
            m = Marker()
            m.header.frame_id = self.octomap_frame
            m.header.stamp = now
            m.ns = 'no_fly_zones'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = (z.min_corner.x + z.max_corner.x) / 2.0
            m.pose.position.y = (z.min_corner.y + z.max_corner.y) / 2.0
            m.pose.position.z = (z.min_corner.z + z.max_corner.z) / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = abs(z.max_corner.x - z.min_corner.x)
            m.scale.y = abs(z.max_corner.y - z.min_corner.y)
            m.scale.z = abs(z.max_corner.z - z.min_corner.z)
            m.color.r = 1.0
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.3
            arr.markers.append(m)
        self.zone_pub.publish(arr)

    def _plan_path_cb(self, request: PlanPath.Request, response: PlanPath.Response):
        if not self.occupied:
            response.success = False
            response.message = 'No octomap data received yet.'
            response.planning_time_ms = 0.0
            return response

        s = request.start.point
        g = request.goal.point
        start_idx = self._world_to_index(s.x, s.y, s.z)
        goal_idx = self._world_to_index(g.x, g.y, g.z)
        alt_bounds = (
            int(np.floor(request.altitude_min / self.resolution)),
            int(np.floor(request.altitude_max / self.resolution)),
        )
        zones = list(request.no_fly_zones)

        self.get_logger().info(
            f'/plan_path: start=({s.x:.1f},{s.y:.1f},{s.z:.1f}) '
            f'goal=({g.x:.1f},{g.y:.1f},{g.z:.1f}) '
            f'alt=[{request.altitude_min:.1f},{request.altitude_max:.1f}] '
            f'no_fly_zones={len(zones)}'
        )

        self._publish_zone_markers(zones)

        blocked = self._get_inflated() | self._zones_to_indices(zones)

        t0 = time.perf_counter()
        path_idx = a_star_3d(start_idx, goal_idx, blocked, alt_bounds)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        if path_idx is None:
            response.success = False
            response.message = (
                'A* failed: start/goal blocked, out of altitude bounds, '
                'inside a no-fly zone, or no connected free path.'
            )
            response.planning_time_ms = elapsed_ms
            self.get_logger().warn(response.message)
            return response

        path = Path()
        path.header.frame_id = self.octomap_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for idx in path_idx:
            ps = PoseStamped()
            ps.header = path.header
            wx, wy, wz = self._index_to_world(idx)
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = wz
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.path_pub.publish(path)

        response.path = path
        response.success = True
        response.planning_time_ms = elapsed_ms
        response.message = f'Path found: {len(path.poses)} waypoints in {elapsed_ms:.1f} ms.'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
