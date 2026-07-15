"""Find parking goal candidates in gaps between Zone Scan wall points."""

import math
import os
import re
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


Corner = Tuple[float, float]
CORNER_ORDER = ('좌전방', '우전방', '우후방', '좌후방')
ZONE_PATTERN = re.compile(r'^\s*ParkingZone\s*(\d+)\s*$', re.IGNORECASE)
CORNER_PATTERN = re.compile(
    r'^\s*(좌전방|좌후방|우전방|우후방).*?'
    r'\(\s*E\s*([-+]?\d+(?:\.\d+)?)\s*,\s*'
    r'N\s*([-+]?\d+(?:\.\d+)?)\s*\)',
)


class PointParking(Node):
    """Project accumulated walls onto each zone opening and find free gaps."""

    def __init__(self) -> None:
        super().__init__('point_parking')

        installed_zone_file = os.path.join(
            get_package_share_directory('auto_parking'),
            'resource',
            'parking_Zone',
        )
        package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        source_zone_file = os.path.join(
            package_root, 'resource', 'parking_Zone')
        default_zone_file = installed_zone_file
        if (not os.path.isfile(default_zone_file) and
                os.path.isfile(source_zone_file)):
            default_zone_file = source_zone_file

        self.declare_parameter('zone_file', default_zone_file)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('utm_frame_id', 'utm')
        self.declare_parameter('datum_topic', '/utm_datum')
        self.declare_parameter(
            'occupied_points_topic', '/zone_scan/occupied_points')
        self.declare_parameter('goal_topic', '/point_parking/goal_utm_yaw')
        self.declare_parameter('goal_pose_topic', '/point_parking/goal_pose')
        self.declare_parameter(
            'candidate_pose_topic', '/point_parking/goal_candidates')
        self.declare_parameter(
            'goal_valid_topic', '/point_parking/goal_valid')
        self.declare_parameter('marker_topic', '/point_parking/markers')
        self.declare_parameter('min_gap_width', 1.5)
        self.declare_parameter('wall_merge_distance', 0.35)
        self.declare_parameter('wall_padding', 0.1)
        self.declare_parameter('wall_search_depth', 4.0)
        self.declare_parameter('wall_behind_margin', 1.0)
        self.declare_parameter('roi_endpoint_margin', 1.0)
        self.declare_parameter('min_zone_wall_points', 3)
        self.declare_parameter('marker_height', 0.15)

        zone_file = str(self.get_parameter('zone_file').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._utm_frame_id = str(self.get_parameter('utm_frame_id').value)
        datum_topic = str(self.get_parameter('datum_topic').value)
        occupied_points_topic = str(
            self.get_parameter('occupied_points_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        goal_pose_topic = str(self.get_parameter('goal_pose_topic').value)
        candidate_pose_topic = str(
            self.get_parameter('candidate_pose_topic').value)
        goal_valid_topic = str(
            self.get_parameter('goal_valid_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        self._min_gap_width = float(
            self.get_parameter('min_gap_width').value)
        self._wall_merge_distance = float(
            self.get_parameter('wall_merge_distance').value)
        self._wall_padding = float(
            self.get_parameter('wall_padding').value)
        self._wall_search_depth = float(
            self.get_parameter('wall_search_depth').value)
        self._wall_behind_margin = float(
            self.get_parameter('wall_behind_margin').value)
        self._roi_endpoint_margin = float(
            self.get_parameter('roi_endpoint_margin').value)
        self._min_zone_wall_points = int(
            self.get_parameter('min_zone_wall_points').value)
        self._marker_height = float(
            self.get_parameter('marker_height').value)

        if self._min_gap_width <= 0.0:
            raise ValueError('min_gap_width must be greater than zero')
        if self._wall_merge_distance < 0.0:
            raise ValueError('wall_merge_distance cannot be negative')
        if self._wall_padding < 0.0:
            raise ValueError('wall_padding cannot be negative')
        if self._wall_search_depth <= 0.0:
            raise ValueError('wall_search_depth must be greater than zero')
        if self._wall_behind_margin < 0.0:
            raise ValueError('wall_behind_margin cannot be negative')
        if self._roi_endpoint_margin < 0.0:
            raise ValueError('roi_endpoint_margin cannot be negative')
        if self._min_zone_wall_points < 1:
            raise ValueError('min_zone_wall_points must be at least 1')

        self._zones = self._load_zones(zone_file)
        self._datum: Corner | None = None
        self._map_points = np.empty((0, 2), dtype=np.float64)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_publisher = self.create_publisher(
            Float64MultiArray, goal_topic, latched_qos)
        self._goal_pose_publisher = self.create_publisher(
            PoseStamped, goal_pose_topic, latched_qos)
        self._candidate_publisher = self.create_publisher(
            PoseArray, candidate_pose_topic, latched_qos)
        self._goal_valid_publisher = self.create_publisher(
            Bool, goal_valid_topic, latched_qos)
        self._marker_publisher = self.create_publisher(
            MarkerArray, marker_topic, latched_qos)
        self._datum_subscription = self.create_subscription(
            PointStamped, datum_topic, self._on_datum, latched_qos)
        self._wall_subscription = self.create_subscription(
            PointCloud2,
            occupied_points_topic,
            self._on_occupied_points,
            latched_qos,
        )

        self.get_logger().info(
            f'Point Parking started: min_gap={self._min_gap_width:.2f} m, '
            f'wall_topic={occupied_points_topic}.')

    @staticmethod
    def _load_zones(zone_file: str):
        zones: Dict[str, Dict[str, Corner]] = {}
        current_zone = None
        with open(zone_file, 'r', encoding='utf-8') as stream:
            for line_number, line in enumerate(stream, start=1):
                zone_match = ZONE_PATTERN.match(line)
                if zone_match is not None:
                    current_zone = f'ParkingZone{zone_match.group(1)}'
                    zones[current_zone] = {}
                    continue
                match = CORNER_PATTERN.search(line)
                if match is None:
                    continue
                if current_zone is None:
                    current_zone = 'ParkingZone1'
                    zones[current_zone] = {}
                corner_name = match.group(1)
                if corner_name in zones[current_zone]:
                    raise ValueError(
                        f'Duplicate {corner_name} in {current_zone} at '
                        f'line {line_number}')
                zones[current_zone][corner_name] = (
                    float(match.group(2)), float(match.group(3)))

        parsed = []
        for zone_name, corners in zones.items():
            missing = [name for name in CORNER_ORDER if name not in corners]
            if missing:
                raise ValueError(
                    f'{zone_name} is missing corners: {", ".join(missing)}')
            if zone_name == 'ParkingZone1':
                open_edge = (corners['좌전방'], corners['우전방'])
            elif zone_name == 'ParkingZone2':
                open_edge = (corners['좌전방'], corners['좌후방'])
            else:
                raise ValueError(
                    f'Open edge is not defined for {zone_name}')
            parsed.append({
                'name': zone_name,
                'boundary': [corners[name] for name in CORNER_ORDER],
                'open_edge': open_edge,
            })
        if not parsed:
            raise ValueError(f'{zone_file} contains no parking zones')
        return parsed

    def _on_datum(self, message: PointStamped) -> None:
        self._datum = (float(message.point.x), float(message.point.y))
        self._update_goals()

    def _on_occupied_points(self, message: PointCloud2) -> None:
        points = [
            (float(point[0]), float(point[1]))
            for point in point_cloud2.read_points(
                message, field_names=('x', 'y'), skip_nans=True)
        ]
        self._map_points = (
            np.asarray(points, dtype=np.float64)
            if points else np.empty((0, 2), dtype=np.float64)
        )
        self._update_goals()

    @staticmethod
    def _pose(easting: float, northing: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = easting
        pose.position.y = northing
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _utm_points(self):
        if self._datum is None or len(self._map_points) == 0:
            return np.empty((0, 2), dtype=np.float64)
        points = self._map_points.copy()
        points[:, 0] += self._datum[0]
        points[:, 1] = self._datum[1] - points[:, 1]
        return points

    def _zone_candidates(self, zone, utm_points):
        raw_edge_start = np.asarray(
            zone['open_edge'][0], dtype=np.float64)
        raw_edge_end = np.asarray(
            zone['open_edge'][1], dtype=np.float64)
        raw_edge_vector = raw_edge_end - raw_edge_start
        raw_edge_length = float(np.linalg.norm(raw_edge_vector))
        if raw_edge_length <= 1e-9:
            return []
        edge_axis = raw_edge_vector / raw_edge_length
        edge_start = (
            raw_edge_start - edge_axis * self._roi_endpoint_margin)
        edge_end = raw_edge_end + edge_axis * self._roi_endpoint_margin
        edge_length = raw_edge_length + 2.0 * self._roi_endpoint_margin

        centroid = np.mean(
            np.asarray(zone['boundary'], dtype=np.float64), axis=0)
        edge_midpoint = (raw_edge_start + raw_edge_end) * 0.5
        inward = centroid - edge_midpoint
        inward_length = float(np.linalg.norm(inward))
        if inward_length <= 1e-9:
            return []
        inward /= inward_length
        # Gap 탐색은 zone 안쪽 방향을 기준으로 유지하되, 주차 goal의
        # 자세는 반대인 zone 안쪽 -> 열린 입구 방향으로 설정한다.
        yaw_utm = math.atan2(float(-inward[1]), float(-inward[0]))

        relative = utm_points - edge_start
        projections = relative @ edge_axis
        projected_points = (
            edge_start[np.newaxis, :] +
            projections[:, np.newaxis] * edge_axis[np.newaxis, :]
        )
        depths = np.einsum(
            'ij,j->i', utm_points - projected_points, inward)
        valid = (
            (projections >= 0.0) &
            (projections <= edge_length) &
            (depths >= -self._wall_behind_margin) &
            (depths <= self._wall_search_depth)
        )
        wall_positions = np.sort(projections[valid])
        if len(wall_positions) < self._min_zone_wall_points:
            return []

        intervals = [
            [
                max(0.0, float(position) - self._wall_padding),
                min(edge_length, float(position) + self._wall_padding),
            ]
            for position in wall_positions
        ]
        merged = []
        for interval in intervals:
            if (not merged or
                    interval[0] >
                    merged[-1][1] + self._wall_merge_distance):
                merged.append(interval)
            else:
                merged[-1][1] = max(merged[-1][1], interval[1])

        gaps = []
        cursor = 0.0
        for interval_start, interval_end in merged:
            if interval_start - cursor >= self._min_gap_width:
                gaps.append((cursor, interval_start))
            cursor = max(cursor, interval_end)
        if edge_length - cursor >= self._min_gap_width:
            gaps.append((cursor, edge_length))

        candidates = []
        for gap_start, gap_end in gaps:
            start_utm = edge_start + edge_axis * gap_start
            end_utm = edge_start + edge_axis * gap_end
            midpoint = (start_utm + end_utm) * 0.5
            candidates.append({
                'zone': zone['name'],
                'width': gap_end - gap_start,
                'start': tuple(start_utm),
                'end': tuple(end_utm),
                'easting': float(midpoint[0]),
                'northing': float(midpoint[1]),
                'yaw': yaw_utm,
            })
        return candidates

    def _update_goals(self) -> None:
        if self._datum is None:
            return
        stamp = self.get_clock().now().to_msg()
        utm_points = self._utm_points()
        candidates = []
        if len(utm_points) > 0:
            for zone in self._zones:
                candidates.extend(self._zone_candidates(zone, utm_points))
        candidates.sort(key=lambda candidate: candidate['width'], reverse=True)

        candidate_message = PoseArray()
        candidate_message.header.frame_id = self._utm_frame_id
        candidate_message.header.stamp = stamp
        candidate_message.poses = [
            self._pose(
                candidate['easting'], candidate['northing'],
                candidate['yaw'])
            for candidate in candidates
        ]
        self._candidate_publisher.publish(candidate_message)

        valid_message = Bool()
        valid_message.data = bool(candidates)
        self._goal_valid_publisher.publish(valid_message)

        goal_values = Float64MultiArray()
        if candidates:
            selected = candidates[0]
            goal_values.data = [
                selected['easting'],
                selected['northing'],
                selected['yaw'],
            ]
            goal_pose = PoseStamped()
            goal_pose.header = candidate_message.header
            goal_pose.pose = self._pose(
                selected['easting'], selected['northing'], selected['yaw'])
            self._goal_pose_publisher.publish(goal_pose)
        self._goal_publisher.publish(goal_values)
        self._publish_markers(stamp, candidates)

    def _publish_markers(self, stamp, candidates) -> None:
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        datum_easting, datum_northing = self._datum
        for index, candidate in enumerate(candidates):
            start_map = (
                candidate['start'][0] - datum_easting,
                -(candidate['start'][1] - datum_northing),
            )
            end_map = (
                candidate['end'][0] - datum_easting,
                -(candidate['end'][1] - datum_northing),
            )
            goal_map = (
                candidate['easting'] - datum_easting,
                -(candidate['northing'] - datum_northing),
            )
            is_selected = index == 0
            color = (0.1, 1.0, 0.2) if is_selected else (0.1, 0.7, 1.0)

            gap = self._marker(
                stamp, 'parking_gap', index, Marker.LINE_STRIP)
            gap.scale.x = 0.18 if is_selected else 0.1
            gap.color.r, gap.color.g, gap.color.b = color
            gap.color.a = 1.0
            gap.points = [
                self._point(*start_map, self._marker_height),
                self._point(*end_map, self._marker_height),
            ]
            marker_array.markers.append(gap)

            goal = self._marker(
                stamp, 'parking_goal', index, Marker.SPHERE)
            goal.pose.position.x = goal_map[0]
            goal.pose.position.y = goal_map[1]
            goal.pose.position.z = self._marker_height
            goal.scale.x = goal.scale.y = goal.scale.z = (
                0.45 if is_selected else 0.3)
            goal.color.r, goal.color.g, goal.color.b = color
            goal.color.a = 1.0
            marker_array.markers.append(goal)

            arrow = self._marker(
                stamp, 'parking_goal_yaw', index, Marker.ARROW)
            arrow.pose.position.x = goal_map[0]
            arrow.pose.position.y = goal_map[1]
            arrow.pose.position.z = self._marker_height
            map_yaw = -candidate['yaw']
            arrow.pose.orientation.z = math.sin(map_yaw * 0.5)
            arrow.pose.orientation.w = math.cos(map_yaw * 0.5)
            arrow.scale.x = 0.9
            arrow.scale.y = 0.16
            arrow.scale.z = 0.16
            arrow.color.r, arrow.color.g, arrow.color.b = color
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            label = self._marker(
                stamp, 'parking_goal_label', index, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = goal_map[0]
            label.pose.position.y = goal_map[1]
            label.pose.position.z = self._marker_height + 0.5
            label.scale.z = 0.45
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = (
                f'{candidate["zone"]} gap={candidate["width"]:.2f}m')
            marker_array.markers.append(label)

        self._marker_publisher.publish(marker_array)

    def _marker(self, stamp, namespace, marker_id, marker_type) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointParking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
