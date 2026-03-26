#!/usr/bin/env python3
# FILE: swarm_flocking/flock_monitor_node.py
"""
FlockMonitorNode — singleton node that monitors the entire swarm.

Responsibilities:
  • Subscribe to all /robot_i/pose_share and /robot_i/velocity_share topics.
  • At 2 Hz:
      - Compute centroid and cohesion radius.
      - Compute average speed.
      - Graph-based split detection (BFS connected components).
      - Convex hull of all robot positions (Graham scan).
      - Detect near-collision pairs (for collision_count).
  • Publish:
      - /flock/centroid        (geometry_msgs/PointStamped)
      - /flock/convex_hull     (visualization_msgs/Marker  — LINE_STRIP)
      - /flock/neighbor_links  (visualization_msgs/MarkerArray — thin lines)
      - /flock/state           (swarm_interfaces/FlockState)

All geometry uses frame_id='map'.
"""

import math
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import PoseStamped, TwistStamped

from swarm_interfaces.msg import FlockState
from swarm_flocking.utils.reynolds import yaw_from_quaternion

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STALE_TIMEOUT_S = 2.0

# Near-collision threshold: if two robots are closer than this we count it
COLLISION_RADIUS = 0.25   # meters (slightly > TurtleBot3 burger body radius)

STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


# ---------------------------------------------------------------------------
# Simple data holders
# ---------------------------------------------------------------------------

class RobotState:
    __slots__ = ('x', 'y', 'theta', 'vx', 'vy', 'pose_ts', 'vel_ts')

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.vx = self.vy = 0.0
        self.pose_ts: float = 0.0
        self.vel_ts: float = 0.0


# ---------------------------------------------------------------------------
# FlockMonitorNode
# ---------------------------------------------------------------------------

class FlockMonitorNode(Node):

    def __init__(self):
        super().__init__('flock_monitor')

        # ------- Parameters -------
        self.declare_parameter('num_robots', 6)
        self.declare_parameter('neighbor_radius', 3.0)
        self.declare_parameter('separation_radius', 0.8)

        self.num_robots    = int(self.get_parameter('num_robots').value)
        self.neighbour_r   = float(self.get_parameter('neighbor_radius').value)

        # ------- Per-robot state -------
        self.robot_states: Dict[int, RobotState] = {
            i: RobotState() for i in range(self.num_robots)
        }

        # Cumulative collision count (persistent across cycles)
        self._cumulative_collisions: int = 0
        self._prev_collision_pairs: set = set()

        # ------- Subscriptions -------
        for i in range(self.num_robots):
            self.create_subscription(
                PoseStamped,
                f'/robot_{i}/pose_share',
                lambda msg, rid=i: self._pose_cb(rid, msg),
                STATE_QOS,
            )
            self.create_subscription(
                TwistStamped,
                f'/robot_{i}/velocity_share',
                lambda msg, rid=i: self._vel_cb(rid, msg),
                STATE_QOS,
            )

        # ------- Publishers -------
        self.centroid_pub = self.create_publisher(
            PointStamped, '/flock/centroid', 10)
        self.hull_pub = self.create_publisher(
            Marker, '/flock/convex_hull', 10)
        self.links_pub = self.create_publisher(
            MarkerArray, '/flock/neighbor_links', 10)
        self.state_pub = self.create_publisher(
            FlockState, '/flock/state', 10)

        # ------- Timer at 2 Hz -------
        self.timer = self.create_timer(0.5, self._compute_and_publish)

        self.get_logger().info(
            f'FlockMonitorNode started — monitoring {self.num_robots} robots')

    # ====================================================================
    # Callbacks
    # ====================================================================

    def _pose_cb(self, robot_id: int, msg: PoseStamped) -> None:
        s = self.robot_states[robot_id]
        s.x = msg.pose.position.x
        s.y = msg.pose.position.y
        s.theta = yaw_from_quaternion(msg.pose.orientation)
        s.pose_ts = time.monotonic()

    def _vel_cb(self, robot_id: int, msg: TwistStamped) -> None:
        s = self.robot_states[robot_id]
        s.vx = msg.twist.linear.x
        s.vy = msg.twist.linear.y
        s.vel_ts = time.monotonic()

    # ====================================================================
    # Main compute cycle
    # ====================================================================

    def _compute_and_publish(self) -> None:
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        # Filter to robots with fresh pose data
        active_ids = [
            rid for rid, s in self.robot_states.items()
            if s.pose_ts > 0.0 and (now - s.pose_ts) <= STALE_TIMEOUT_S
        ]

        num_active = len(active_ids)
        if num_active == 0:
            return

        # ------------------------------------------------------------------
        # 1. Centroid
        # ------------------------------------------------------------------
        cx = sum(self.robot_states[rid].x for rid in active_ids) / num_active
        cy = sum(self.robot_states[rid].y for rid in active_ids) / num_active

        # ------------------------------------------------------------------
        # 2. Cohesion radius (mean distance from centroid)
        # ------------------------------------------------------------------
        cohesion_radius = sum(
            math.hypot(self.robot_states[rid].x - cx,
                       self.robot_states[rid].y - cy)
            for rid in active_ids
        ) / num_active

        # ------------------------------------------------------------------
        # 3. Average speed
        # ------------------------------------------------------------------
        avg_speed = sum(
            math.hypot(self.robot_states[rid].vx, self.robot_states[rid].vy)
            for rid in active_ids
        ) / num_active

        # ------------------------------------------------------------------
        # 4. Collision detection
        # ------------------------------------------------------------------
        current_pairs: set = set()
        positions = [(rid, self.robot_states[rid].x, self.robot_states[rid].y)
                     for rid in active_ids]

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                rid_a, xa, ya = positions[i]
                rid_b, xb, yb = positions[j]
                if math.hypot(xa - xb, ya - yb) < COLLISION_RADIUS:
                    pair = (min(rid_a, rid_b), max(rid_a, rid_b))
                    current_pairs.add(pair)

        # Count new collision events (pairs that weren't colliding before)
        new_events = current_pairs - self._prev_collision_pairs
        self._cumulative_collisions += len(new_events)
        self._prev_collision_pairs = current_pairs

        # ------------------------------------------------------------------
        # 5. Graph-based split detection (BFS connected components)
        # ------------------------------------------------------------------
        num_subgroups, is_split = self._detect_split(active_ids)

        # ------------------------------------------------------------------
        # 6. Publish FlockState
        # ------------------------------------------------------------------
        state_msg = FlockState()
        state_msg.header.stamp = stamp
        state_msg.header.frame_id = 'map'
        state_msg.centroid.x = cx
        state_msg.centroid.y = cy
        state_msg.centroid.z = 0.0
        state_msg.cohesion_radius    = float(cohesion_radius)
        state_msg.avg_speed          = float(avg_speed)
        state_msg.num_active_robots  = num_active
        state_msg.collision_count    = self._cumulative_collisions
        state_msg.is_split           = is_split
        state_msg.num_subgroups      = num_subgroups
        self.state_pub.publish(state_msg)

        # ------------------------------------------------------------------
        # 7. Publish centroid marker
        # ------------------------------------------------------------------
        centroid_msg = PointStamped()
        centroid_msg.header.stamp = stamp
        centroid_msg.header.frame_id = 'map'
        centroid_msg.point.x = cx
        centroid_msg.point.y = cy
        centroid_msg.point.z = 0.05
        self.centroid_pub.publish(centroid_msg)

        # ------------------------------------------------------------------
        # 8. Convex hull marker
        # ------------------------------------------------------------------
        hull_marker = self._build_hull_marker(active_ids, stamp)
        self.hull_pub.publish(hull_marker)

        # ------------------------------------------------------------------
        # 9. Neighbor link markers
        # ------------------------------------------------------------------
        links_array = self._build_link_markers(active_ids, stamp)
        self.links_pub.publish(links_array)

    # ====================================================================
    # Split detection via BFS
    # ====================================================================

    def _detect_split(self, active_ids: List[int]) -> Tuple[int, bool]:
        """
        Build an adjacency graph where robots within neighbour_radius are
        connected.  Count connected components via BFS.

        Returns (num_components, is_split).
        """
        if len(active_ids) <= 1:
            return (len(active_ids), False)

        # Build adjacency list
        adj: Dict[int, List[int]] = {rid: [] for rid in active_ids}
        for i in range(len(active_ids)):
            for j in range(i + 1, len(active_ids)):
                rid_a = active_ids[i]
                rid_b = active_ids[j]
                sa = self.robot_states[rid_a]
                sb = self.robot_states[rid_b]
                if math.hypot(sa.x - sb.x, sa.y - sb.y) <= self.neighbour_r:
                    adj[rid_a].append(rid_b)
                    adj[rid_b].append(rid_a)

        # BFS to count components
        visited: set = set()
        num_components = 0
        for start in active_ids:
            if start in visited:
                continue
            num_components += 1
            queue = deque([start])
            while queue:
                node = queue.popleft()
                if node in visited:
                    continue
                visited.add(node)
                for neighbour in adj[node]:
                    if neighbour not in visited:
                        queue.append(neighbour)

        return (num_components, num_components > 1)

    # ====================================================================
    # Convex hull (Graham scan)
    # ====================================================================

    def _convex_hull(
        self, points: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Graham scan convex hull.  Returns vertices in CCW order.
        Handles degenerate cases (< 3 unique points) gracefully.
        """
        # Deduplicate
        pts = list(set((round(x, 6), round(y, 6)) for x, y in points))

        if len(pts) < 2:
            return pts
        if len(pts) == 2:
            return pts

        # Find bottom-most (then left-most) point as pivot
        pivot = min(pts, key=lambda p: (p[1], p[0]))

        def polar_angle(p):
            dx = p[0] - pivot[0]
            dy = p[1] - pivot[1]
            return math.atan2(dy, dx)

        def dist_sq(p):
            dx = p[0] - pivot[0]
            dy = p[1] - pivot[1]
            return dx * dx + dy * dy

        # Sort by polar angle, break ties by distance
        sorted_pts = sorted(pts, key=lambda p: (polar_angle(p), dist_sq(p)))

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        hull = []
        for p in sorted_pts:
            while len(hull) >= 2 and cross(hull[-2], hull[-1], p) <= 0:
                hull.pop()
            hull.append(p)

        return hull

    # ====================================================================
    # RViz marker builders
    # ====================================================================

    def _build_hull_marker(self, active_ids: List[int], stamp) -> Marker:
        """Build a LINE_STRIP marker tracing the convex hull of the swarm."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = 'flock_hull'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05   # line width (m)
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0

        pts_2d = [
            (self.robot_states[rid].x, self.robot_states[rid].y)
            for rid in active_ids
        ]
        hull = self._convex_hull(pts_2d)

        if len(hull) < 2:
            # Just a dot or single point — draw a tiny loop
            for hx, hy in hull:
                p = Point()
                p.x, p.y, p.z = hx, hy, 0.05
                marker.points.append(p)
        else:
            # Close the polygon
            for hx, hy in hull:
                p = Point()
                p.x, p.y, p.z = hx, hy, 0.05
                marker.points.append(p)
            # Close by repeating first point
            p = Point()
            p.x, p.y, p.z = hull[0][0], hull[0][1], 0.05
            marker.points.append(p)

        return marker

    def _build_link_markers(
        self, active_ids: List[int], stamp
    ) -> MarkerArray:
        """
        Build a MarkerArray of thin LINE_LIST markers connecting every pair
        of robots within neighbour_radius.
        """
        array = MarkerArray()

        # First, delete old markers to avoid ghost lines
        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = 'map'
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        marker_id = 1
        for i in range(len(active_ids)):
            for j in range(i + 1, len(active_ids)):
                rid_a = active_ids[i]
                rid_b = active_ids[j]
                sa = self.robot_states[rid_a]
                sb = self.robot_states[rid_b]
                dist = math.hypot(sa.x - sb.x, sa.y - sb.y)

                if dist > self.neighbour_r:
                    continue

                link = Marker()
                link.header.stamp = stamp
                link.header.frame_id = 'map'
                link.ns = 'flock_links'
                link.id = marker_id
                link.type = Marker.LINE_LIST
                link.action = Marker.ADD
                link.scale.x = 0.02  # thin line
                link.color.r = 0.3
                link.color.g = 0.6
                link.color.b = 1.0
                link.color.a = 0.5
                link.pose.orientation.w = 1.0

                pa = Point()
                pa.x, pa.y, pa.z = sa.x, sa.y, 0.05
                pb = Point()
                pb.x, pb.y, pb.z = sb.x, sb.y, 0.05
                link.points.extend([pa, pb])

                array.markers.append(link)
                marker_id += 1

        return array


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FlockMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
