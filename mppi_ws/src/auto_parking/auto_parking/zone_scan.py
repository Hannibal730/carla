"""Publish the UTM parking-zone boundary as RViz markers in the map frame."""

from collections import deque
import os
import re
from math import ceil, floor, hypot
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Corner = Tuple[float, float]

CORNER_ORDER = ('좌전방', '우전방', '우후방', '좌후방')
ZONE_PATTERN = re.compile(r'^\s*ParkingZone\s*(\d+)\s*$', re.IGNORECASE)
CORNER_PATTERN = re.compile(
    r'^\s*(좌전방|좌후방|우전방|우후방).*?'
    r'\(\s*E\s*([-+]?\d+(?:\.\d+)?)\s*,\s*'
    r'N\s*([-+]?\d+(?:\.\d+)?)\s*\)',
)


class ZoneScan(Node):
    """Convert an absolute UTM parking boundary to map-relative markers."""

    def __init__(self) -> None:
        super().__init__('zone_scan')

        installed_zone_file = os.path.join(
            get_package_share_directory('auto_parking'),
            'resource',
            'parking_Zone',
        )
        # A stale incremental/symlink install can contain the new executable
        # before the newly-added resource. Resolve the symlink back to source.
        package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        source_zone_file = os.path.join(
            package_root, 'resource', 'parking_Zone')
        default_zone_file = installed_zone_file
        if not os.path.isfile(default_zone_file) and os.path.isfile(source_zone_file):
            default_zone_file = source_zone_file
            self.get_logger().warning(
                'Installed parking_Zone resource is missing; using source '
                f'file: {source_zone_file}')

        self.declare_parameter('zone_file', default_zone_file)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('datum_topic', '/utm_datum')
        self.declare_parameter('marker_topic', '/parking_zones')
        self.declare_parameter('line_width', 0.25)
        self.declare_parameter('zone_height', 0.05)
        self.declare_parameter(
            'lidar_topic', '/carla/car/lidar_2d/point_cloud')
        self.declare_parameter('history_resolution', 0.15)
        self.declare_parameter('roi_margin', 1.0)
        self.declare_parameter('lidar_line_width', 0.12)
        self.declare_parameter('tf_wait_timeout', 0.5)
        self.declare_parameter('spatial_merge_radius', 0.35)
        self.declare_parameter('accumulation_scope', 'parking_zones')
        self.declare_parameter('icp_enabled', True)
        self.declare_parameter('icp_max_iterations', 8)
        self.declare_parameter('icp_min_submap_points', 30)
        self.declare_parameter('icp_min_correspondences', 20)
        self.declare_parameter('icp_max_correspondence_distance', 0.5)
        self.declare_parameter('icp_normal_radius', 0.6)
        self.declare_parameter('icp_max_normal_ratio', 0.3)
        self.declare_parameter('icp_huber_delta', 0.1)
        self.declare_parameter('icp_max_translation', 0.5)
        self.declare_parameter('icp_max_rotation_deg', 3.0)
        self.declare_parameter('icp_max_scan_points', 400)
        self.declare_parameter('icp_max_submap_points', 2500)
        self.declare_parameter('icp_prior_weight', 1.0)
        self.declare_parameter('icp_min_improvement', 0.002)
        self.declare_parameter('icp_max_point_to_line_rmse', 0.15)
        self.declare_parameter('gate_control_enabled', False)
        self.declare_parameter(
            'zone1_gate_a_utm',
            [0.0, 0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter(
            'zone1_gate_b_utm',
            [0.0, 0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter(
            'zone2_gate_a_utm',
            [0.0, 0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter(
            'zone2_gate_b_utm',
            [0.0, 0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('gate_tolerance', 0.5)
        self.declare_parameter('gate_rearm_distance', 1.0)
        self.declare_parameter('clear_map_on_start', False)
        self.declare_parameter('utm_position_topic', '/f9p_utm')
        self.declare_parameter('parking_mode_topic', '/parkingMode')
        self.declare_parameter('parking_scan_topic', '/parkingScan')

        zone_file = str(self.get_parameter('zone_file').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        datum_topic = str(self.get_parameter('datum_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        self._line_width = float(self.get_parameter('line_width').value)
        self._zone_height = float(self.get_parameter('zone_height').value)
        lidar_topic = str(self.get_parameter('lidar_topic').value)
        self._history_resolution = float(
            self.get_parameter('history_resolution').value)
        self._roi_margin = float(self.get_parameter('roi_margin').value)
        self._lidar_line_width = float(
            self.get_parameter('lidar_line_width').value)
        self._tf_wait_timeout = float(
            self.get_parameter('tf_wait_timeout').value)
        self._spatial_merge_radius = float(
            self.get_parameter('spatial_merge_radius').value)
        self._accumulation_scope = str(
            self.get_parameter('accumulation_scope').value)
        self._icp_enabled = bool(
            self.get_parameter('icp_enabled').value)
        self._icp_max_iterations = int(
            self.get_parameter('icp_max_iterations').value)
        self._icp_min_submap_points = int(
            self.get_parameter('icp_min_submap_points').value)
        self._icp_min_correspondences = int(
            self.get_parameter('icp_min_correspondences').value)
        self._icp_max_correspondence_distance = float(
            self.get_parameter('icp_max_correspondence_distance').value)
        self._icp_normal_radius = float(
            self.get_parameter('icp_normal_radius').value)
        self._icp_max_normal_ratio = float(
            self.get_parameter('icp_max_normal_ratio').value)
        self._icp_huber_delta = float(
            self.get_parameter('icp_huber_delta').value)
        self._icp_max_translation = float(
            self.get_parameter('icp_max_translation').value)
        self._icp_max_rotation = np.deg2rad(float(
            self.get_parameter('icp_max_rotation_deg').value))
        self._icp_max_scan_points = int(
            self.get_parameter('icp_max_scan_points').value)
        self._icp_max_submap_points = int(
            self.get_parameter('icp_max_submap_points').value)
        self._icp_prior_weight = float(
            self.get_parameter('icp_prior_weight').value)
        self._icp_min_improvement = float(
            self.get_parameter('icp_min_improvement').value)
        self._icp_max_point_to_line_rmse = float(
            self.get_parameter('icp_max_point_to_line_rmse').value)
        self._gate_control_enabled = bool(
            self.get_parameter('gate_control_enabled').value)
        self._zone1_gate_a_utm = self._gate_from_parameter(
            'zone1_gate_a_utm')
        self._zone1_gate_b_utm = self._gate_from_parameter(
            'zone1_gate_b_utm')
        self._zone2_gate_a_utm = self._gate_from_parameter(
            'zone2_gate_a_utm')
        self._zone2_gate_b_utm = self._gate_from_parameter(
            'zone2_gate_b_utm')
        self._gate_tolerance = float(
            self.get_parameter('gate_tolerance').value)
        self._gate_rearm_distance = float(
            self.get_parameter('gate_rearm_distance').value)
        self._clear_map_on_start = bool(
            self.get_parameter('clear_map_on_start').value)
        utm_position_topic = str(
            self.get_parameter('utm_position_topic').value)
        parking_mode_topic = str(
            self.get_parameter('parking_mode_topic').value)
        parking_scan_topic = str(
            self.get_parameter('parking_scan_topic').value)

        if self._history_resolution <= 0.0:
            raise ValueError('history_resolution must be greater than zero')
        if self._roi_margin < 0.0:
            raise ValueError('roi_margin cannot be negative')
        if self._lidar_line_width <= 0.0:
            raise ValueError('lidar_line_width must be greater than zero')
        if self._tf_wait_timeout <= 0.0:
            raise ValueError('tf_wait_timeout must be greater than zero')
        if self._spatial_merge_radius < 0.0:
            raise ValueError('spatial_merge_radius cannot be negative')
        if self._accumulation_scope not in {'global', 'parking_zones'}:
            raise ValueError(
                'accumulation_scope must be global or parking_zones')
        if self._icp_max_iterations <= 0:
            raise ValueError('icp_max_iterations must be greater than zero')
        if self._icp_min_submap_points < 3:
            raise ValueError('icp_min_submap_points must be at least 3')
        if self._icp_min_correspondences < 3:
            raise ValueError('icp_min_correspondences must be at least 3')
        if self._icp_max_correspondence_distance <= 0.0:
            raise ValueError(
                'icp_max_correspondence_distance must be greater than zero')
        if self._icp_normal_radius <= self._history_resolution:
            raise ValueError(
                'icp_normal_radius must be greater than history_resolution')
        if not 0.0 <= self._icp_max_normal_ratio < 1.0:
            raise ValueError('icp_max_normal_ratio must be in [0, 1)')
        if self._icp_huber_delta <= 0.0:
            raise ValueError('icp_huber_delta must be greater than zero')
        if self._icp_max_translation <= 0.0:
            raise ValueError('icp_max_translation must be greater than zero')
        if self._icp_max_rotation <= 0.0:
            raise ValueError('icp_max_rotation_deg must be greater than zero')
        if self._icp_max_scan_points < self._icp_min_correspondences:
            raise ValueError(
                'icp_max_scan_points must be >= icp_min_correspondences')
        if self._icp_max_submap_points < self._icp_min_submap_points:
            raise ValueError(
                'icp_max_submap_points must be >= icp_min_submap_points')
        if self._icp_prior_weight < 0.0:
            raise ValueError('icp_prior_weight cannot be negative')
        if self._icp_min_improvement < 0.0:
            raise ValueError('icp_min_improvement cannot be negative')
        if self._icp_max_point_to_line_rmse <= 0.0:
            raise ValueError(
                'icp_max_point_to_line_rmse must be greater than zero')
        if self._gate_tolerance < 0.0:
            raise ValueError('gate_tolerance cannot be negative')
        if self._gate_rearm_distance <= self._gate_tolerance:
            raise ValueError(
                'gate_rearm_distance must be greater than gate_tolerance')
        if self._gate_control_enabled:
            self._validate_gate(
                'zone1_gate_a_utm', self._zone1_gate_a_utm)
            self._validate_gate(
                'zone1_gate_b_utm', self._zone1_gate_b_utm)
            self._validate_gate(
                'zone2_gate_a_utm', self._zone2_gate_a_utm)
            self._validate_gate(
                'zone2_gate_b_utm', self._zone2_gate_b_utm)

        self._utm_zones = self._load_utm_zones(zone_file)
        self._datum: Corner | None = None
        self._occupied_voxels = set()
        self._pending_lidar = deque(maxlen=50)
        self._tf_warning_reported = False
        # Always start outside a parking zone with scan recording off.
        self._parking_mode = False
        self._parking_scan = False
        self._previous_utm_position = None
        self._gate_armed = {
            'zone1 gate a': True,
            'zone1 gate b': True,
            'zone2 gate a': True,
            'zone2 gate b': True,
        }

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            MarkerArray, marker_topic, latched_qos)
        self._datum_subscription = self.create_subscription(
            PointStamped, datum_topic, self._on_datum, latched_qos)
        self._parking_mode_publisher = self.create_publisher(
            Bool, parking_mode_topic, latched_qos)
        self._parking_scan_publisher = self.create_publisher(
            Bool, parking_scan_topic, latched_qos)

        lidar_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lidar_subscription = self.create_subscription(
            PointCloud2, lidar_topic, self._on_lidar, lidar_qos)
        self._utm_position_subscription = self.create_subscription(
            PointStamped,
            utm_position_topic,
            self._on_utm_position,
            lidar_qos,
        )

        # LiDAR arrives about 0.05 s before the matching dynamic TF. Queue it
        # and process only when the exact measurement-time TF is available.
        self._lidar_processing_timer = self.create_timer(
            0.02, self._process_pending_lidar)
        # Republish so RViz recovers cleanly after a display reset.
        self._marker_timer = self.create_timer(1.0, self._publish_markers)
        self._publish_parking_mode()
        self._publish_parking_scan()
        self.get_logger().info(
            f'Loaded {len(self._utm_zones)} parking zones from {zone_file}. '
            f'Waiting for datum on {datum_topic}; LiDAR={lidar_topic}; '
            f'accumulation_scope={self._accumulation_scope}; '
            f'point_to_line_icp={"enabled" if self._icp_enabled else "disabled"}; '
            f'gate_control={self._gate_control_enabled}; '
            f'parking_scan={self._parking_scan}; '
            f'parking_mode={self._parking_mode}.')

    def _gate_from_parameter(self, parameter_name: str):
        values = [
            float(value)
            for value in self.get_parameter(parameter_name).value
        ]
        if len(values) != 4:
            raise ValueError(
                f'{parameter_name} must contain [E1, N1, E2, N2]')
        return ((values[0], values[1]), (values[2], values[3]))

    @staticmethod
    def _validate_gate(gate_name: str, gate) -> None:
        if gate[0] == gate[1]:
            raise ValueError(f'{gate_name} endpoints must be different')

    def _publish_parking_mode(self) -> None:
        message = Bool()
        message.data = self._parking_mode
        self._parking_mode_publisher.publish(message)

    def _publish_parking_scan(self) -> None:
        message = Bool()
        message.data = self._parking_scan
        self._parking_scan_publisher.publish(message)

    def _set_parking_state(
            self, scan_enabled: bool, mode_enabled: bool,
            gate_name: str) -> None:
        if (self._parking_scan == scan_enabled and
                self._parking_mode == mode_enabled):
            return

        if (scan_enabled and not self._parking_scan and
                self._clear_map_on_start):
            self._occupied_voxels.clear()
        if self._parking_scan != scan_enabled:
            # A scan captured before a transition must never be processed
            # under the state after the transition.
            self._pending_lidar.clear()

        self._parking_scan = scan_enabled
        self._parking_mode = mode_enabled
        self._publish_parking_scan()
        self._publish_parking_mode()
        self._publish_markers()
        self.get_logger().info(
            f'Crossed {gate_name}: '
            f'/parkingScan={str(scan_enabled).lower()}, '
            f'/parkingMode={str(mode_enabled).lower()}.')

    @staticmethod
    def _orientation(a: Corner, b: Corner, c: Corner) -> float:
        return ((b[0] - a[0]) * (c[1] - a[1]) -
                (b[1] - a[1]) * (c[0] - a[0]))

    @staticmethod
    def _point_to_segment_distance(
            point: Corner, start: Corner, end: Corner) -> float:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            return hypot(point[0] - start[0], point[1] - start[1])
        ratio = (
            (point[0] - start[0]) * dx +
            (point[1] - start[1]) * dy
        ) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        closest = (start[0] + ratio * dx, start[1] + ratio * dy)
        return hypot(point[0] - closest[0], point[1] - closest[1])

    @classmethod
    def _segments_intersect(cls, a, b, c, d) -> bool:
        epsilon = 1e-9
        ab_c = cls._orientation(a, b, c)
        ab_d = cls._orientation(a, b, d)
        cd_a = cls._orientation(c, d, a)
        cd_b = cls._orientation(c, d, b)
        return (
            ((ab_c > epsilon and ab_d < -epsilon) or
             (ab_c < -epsilon and ab_d > epsilon)) and
            ((cd_a > epsilon and cd_b < -epsilon) or
             (cd_a < -epsilon and cd_b > epsilon))
        )

    @classmethod
    def _segment_distance(cls, a, b, c, d) -> float:
        if cls._segments_intersect(a, b, c, d):
            return 0.0
        return min(
            cls._point_to_segment_distance(a, c, d),
            cls._point_to_segment_distance(b, c, d),
            cls._point_to_segment_distance(c, a, b),
            cls._point_to_segment_distance(d, a, b),
        )

    @classmethod
    def _inside_zone_roi(
            cls, point: Corner, boundary: List[Corner],
            margin: float) -> bool:
        """Accept points inside a zone or within margin of its polygon."""
        for index, edge_start in enumerate(boundary):
            edge_end = boundary[(index + 1) % len(boundary)]
            if cls._point_to_segment_distance(
                    point, edge_start, edge_end) <= margin:
                return True

        # Ray casting against the closed four-corner parking polygon.
        inside = False
        x, y = point
        previous = boundary[-1]
        for current in boundary:
            current_above = current[1] > y
            previous_above = previous[1] > y
            if current_above != previous_above:
                intersection_x = (
                    (previous[0] - current[0]) *
                    (y - current[1]) /
                    (previous[1] - current[1]) + current[0]
                )
                if x < intersection_x:
                    inside = not inside
            previous = current
        return inside

    def _gate_crossed_once(
            self, gate_name: str, gate, previous: Corner,
            current: Corner) -> bool:
        """Return one event per physical crossing, despite GPS jitter."""
        current_distance = self._point_to_segment_distance(
            current, gate[0], gate[1])

        if not self._gate_armed[gate_name]:
            if current_distance >= self._gate_rearm_distance:
                self._gate_armed[gate_name] = True
            return False

        travel_distance = self._segment_distance(
            previous, current, gate[0], gate[1])
        if travel_distance > self._gate_tolerance:
            return False

        self._gate_armed[gate_name] = False
        return True

    def _on_utm_position(self, message: PointStamped) -> None:
        current = (float(message.point.x), float(message.point.y))
        previous = self._previous_utm_position
        self._previous_utm_position = current
        if not self._gate_control_enabled or previous is None:
            return

        crossings = (
            ('ParkingZone1 gate A', 'zone1 gate a',
             self._zone1_gate_a_utm, 'zone1_a'),
            ('ParkingZone1 gate B', 'zone1 gate b',
             self._zone1_gate_b_utm, 'zone1_b'),
            ('ParkingZone2 gate A', 'zone2 gate a',
             self._zone2_gate_a_utm, 'zone2_a'),
            ('ParkingZone2 gate B', 'zone2 gate b',
             self._zone2_gate_b_utm, 'zone2_b'),
        )
        for display_name, gate_key, gate, gate_action in crossings:
            if not self._gate_crossed_once(
                    gate_key, gate, previous, current):
                continue

            if (gate_action in {'zone1_a', 'zone2_a'} and
                    not self._parking_scan and not self._parking_mode):
                self._set_parking_state(True, False, display_name)
            elif (gate_action in {'zone1_b', 'zone2_b'} and
                    self._parking_scan and not self._parking_mode):
                self._set_parking_state(False, True, display_name)
            elif (gate_action == 'zone1_a' and
                    not self._parking_scan and self._parking_mode):
                self._set_parking_state(False, False, display_name)
            elif (gate_action == 'zone2_b' and
                    not self._parking_scan and self._parking_mode):
                self._set_parking_state(False, False, display_name)
            break

    @staticmethod
    def _load_utm_zones(zone_file: str):
        zones: Dict[str, Dict[str, Corner]] = {}
        current_zone = None
        with open(zone_file, 'r', encoding='utf-8') as stream:
            for line_number, line in enumerate(stream, start=1):
                zone_match = ZONE_PATTERN.match(line)
                if zone_match is not None:
                    current_zone = f'ParkingZone{zone_match.group(1)}'
                    if current_zone in zones:
                        raise ValueError(
                            f'Duplicate section {current_zone!r} at '
                            f'line {line_number}')
                    zones[current_zone] = {}
                    continue

                match = CORNER_PATTERN.search(line)
                if match is None:
                    continue
                # Preserve compatibility with the original one-zone file.
                if current_zone is None:
                    current_zone = 'ParkingZone1'
                    zones[current_zone] = {}
                name = match.group(1)
                if name in zones[current_zone]:
                    raise ValueError(
                        f'Duplicate corner {name!r} in {current_zone} at '
                        f'line {line_number}')
                zones[current_zone][name] = (
                    float(match.group(2)), float(match.group(3)))

        if not zones:
            raise ValueError(f'{zone_file} contains no parking zones')

        parsed_zones = []
        for zone_name, corners in zones.items():
            missing = [name for name in CORNER_ORDER if name not in corners]
            if missing:
                raise ValueError(
                    f'{zone_name} is missing UTM corners: '
                    f'{", ".join(missing)}')
            parsed_zones.append((
                zone_name,
                [corners[name] for name in CORNER_ORDER],
            ))
        return parsed_zones

    def _on_datum(self, message: PointStamped) -> None:
        new_datum = (float(message.point.x), float(message.point.y))
        if self._datum == new_datum:
            return

        if self._datum is not None:
            # Stored voxels use the old map origin and must not be mixed with
            # points converted using a changed datum.
            self._occupied_voxels.clear()
            self._pending_lidar.clear()
            self.get_logger().warning(
                'UTM datum changed; cleared the accumulated LiDAR map.')
        self._datum = new_datum
        self.get_logger().info(
            f'Using UTM datum E={new_datum[0]:.3f}, '
            f'N={new_datum[1]:.3f}; publishing in {self._frame_id}.')
        self._publish_markers()

    def _map_boundaries(self):
        """Apply the same UTM-to-map conversion to every parking zone."""
        if self._datum is None:
            return []
        datum_easting, datum_northing = self._datum
        return [
            (
                zone_name,
                [
                    (easting - datum_easting,
                     -(northing - datum_northing))
                    for easting, northing in utm_boundary
                ],
            )
            for zone_name, utm_boundary in self._utm_zones
        ]

    @staticmethod
    def _transform_point(x: float, y: float, z: float, transform):
        """Apply a geometry_msgs Transform without a per-point TF lookup."""
        translation = transform.translation
        rotation = transform.rotation

        # Quaternion-vector rotation: v' = v + 2w(q x v) + 2(q x (q x v)).
        cross_x = rotation.y * z - rotation.z * y
        cross_y = rotation.z * x - rotation.x * z
        cross_z = rotation.x * y - rotation.y * x
        second_x = rotation.y * cross_z - rotation.z * cross_y
        second_y = rotation.z * cross_x - rotation.x * cross_z
        second_z = rotation.x * cross_y - rotation.y * cross_x
        return (
            x + 2.0 * (rotation.w * cross_x + second_x) + translation.x,
            y + 2.0 * (rotation.w * cross_y + second_y) + translation.y,
            z + 2.0 * (rotation.w * cross_z + second_z) + translation.z,
        )

    def _on_lidar(self, message: PointCloud2) -> None:
        if (not self._parking_scan or self._datum is None or
                not message.header.frame_id):
            return
        self._pending_lidar.append((
            message,
            self.get_clock().now().nanoseconds,
        ))

    def _process_pending_lidar(self) -> None:
        """Process queued scans only with their exact global TF timestamp."""
        if not self._parking_scan:
            self._pending_lidar.clear()
            return
        processed = 0
        while self._pending_lidar and processed < 5:
            message, queued_at = self._pending_lidar[0]
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._frame_id,
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=0.0),
                ).transform
            except TransformException as error:
                age = (
                    self.get_clock().now().nanoseconds - queued_at
                ) / 1_000_000_000.0
                if age < self._tf_wait_timeout:
                    return

                self._pending_lidar.popleft()
                if not self._tf_warning_reported:
                    self.get_logger().warning(
                        f'Dropping LiDAR scan after waiting {age:.3f}s for '
                        f'exact {self._frame_id} TF: {error}')
                    self._tf_warning_reported = True
                continue

            self._pending_lidar.popleft()
            self._tf_warning_reported = False
            self._accumulate_lidar(message, transform)
            processed += 1

    def _is_previous_observation(self, voxel) -> bool:
        """Return true when a prior scan already fixed this global location."""
        if not self._occupied_voxels:
            return False
        if self._spatial_merge_radius == 0.0:
            return voxel in self._occupied_voxels

        cell_radius = ceil(
            self._spatial_merge_radius / self._history_resolution)
        for offset_x in range(-cell_radius, cell_radius + 1):
            for offset_y in range(-cell_radius, cell_radius + 1):
                neighbor = (voxel[0] + offset_x, voxel[1] + offset_y)
                if neighbor not in self._occupied_voxels:
                    continue
                distance = hypot(
                    offset_x * self._history_resolution,
                    offset_y * self._history_resolution,
                )
                if distance <= self._spatial_merge_radius:
                    return True
        return False

    def _build_icp_submap(self):
        """Return stable voxel centers and locally fitted wall normals."""
        if len(self._occupied_voxels) < self._icp_min_submap_points:
            return None, None

        resolution = self._history_resolution
        neighbor_radius_cells = ceil(self._icp_normal_radius / resolution)
        normal_radius_squared = self._icp_normal_radius ** 2
        target_points = []
        target_normals = []

        for voxel in self._occupied_voxels:
            neighbors = []
            for offset_x in range(
                    -neighbor_radius_cells, neighbor_radius_cells + 1):
                for offset_y in range(
                        -neighbor_radius_cells, neighbor_radius_cells + 1):
                    if ((offset_x * resolution) ** 2 +
                            (offset_y * resolution) ** 2 >
                            normal_radius_squared):
                        continue
                    neighbor = (
                        voxel[0] + offset_x,
                        voxel[1] + offset_y,
                    )
                    if neighbor in self._occupied_voxels:
                        neighbors.append((
                            (neighbor[0] + 0.5) * resolution,
                            (neighbor[1] + 0.5) * resolution,
                        ))

            if len(neighbors) < 3:
                continue
            neighborhood = np.asarray(neighbors, dtype=np.float64)
            centered = neighborhood - np.mean(neighborhood, axis=0)
            covariance = centered.T @ centered / len(neighborhood)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            if eigenvalues[1] <= 1e-9:
                continue
            if eigenvalues[0] / eigenvalues[1] > self._icp_max_normal_ratio:
                continue

            target_points.append((
                (voxel[0] + 0.5) * resolution,
                (voxel[1] + 0.5) * resolution,
            ))
            target_normals.append(eigenvectors[:, 0])

        if len(target_points) < self._icp_min_submap_points:
            return None, None

        points = np.asarray(target_points, dtype=np.float64)
        normals = np.asarray(target_normals, dtype=np.float64)
        if len(points) > self._icp_max_submap_points:
            indices = np.linspace(
                0, len(points) - 1,
                self._icp_max_submap_points,
                dtype=np.int64,
            )
            points = points[indices]
            normals = normals[indices]
        return points, normals

    def _icp_correspondences(
            self, source_points, target_points, target_normals):
        """Find nearest submap point/normal pairs inside the ICP gate."""
        differences = (
            source_points[:, np.newaxis, :] -
            target_points[np.newaxis, :, :]
        )
        distances_squared = np.einsum(
            'ijk,ijk->ij', differences, differences)
        nearest_indices = np.argmin(distances_squared, axis=1)
        source_indices = np.arange(len(source_points))
        nearest_distances_squared = distances_squared[
            source_indices, nearest_indices]
        valid = nearest_distances_squared <= (
            self._icp_max_correspondence_distance ** 2)
        return (
            source_points[valid],
            target_points[nearest_indices[valid]],
            target_normals[nearest_indices[valid]],
        )

    def _align_scan_to_submap(self, initial_points):
        """Constrain a scan to the accumulated map with 2-D point-to-line ICP."""
        if not self._icp_enabled:
            return initial_points
        target_points, target_normals = self._build_icp_submap()
        if target_points is None:
            return initial_points

        if len(initial_points) > self._icp_max_scan_points:
            sample_indices = np.linspace(
                0, len(initial_points) - 1,
                self._icp_max_scan_points,
                dtype=np.int64,
            )
        else:
            sample_indices = np.arange(len(initial_points))

        corrected_points = initial_points.copy()
        corrected_sample = corrected_points[sample_indices].copy()
        initial_centroid = np.mean(corrected_points, axis=0)
        accumulated_rotation = 0.0
        initial_rmse = None

        for _ in range(self._icp_max_iterations):
            source, target, normals = self._icp_correspondences(
                corrected_sample, target_points, target_normals)
            if len(source) < self._icp_min_correspondences:
                return initial_points

            residuals = np.einsum('ij,ij->i', normals, source - target)
            rmse = float(np.sqrt(np.mean(residuals ** 2)))
            if initial_rmse is None:
                initial_rmse = rmse

            absolute_residuals = np.abs(residuals)
            weights = np.ones_like(residuals)
            outliers = absolute_residuals > self._icp_huber_delta
            weights[outliers] = (
                self._icp_huber_delta / absolute_residuals[outliers])

            pivot = np.mean(corrected_points, axis=0)
            relative = source - pivot
            jacobian = np.column_stack((
                normals[:, 0],
                normals[:, 1],
                -normals[:, 0] * relative[:, 1] +
                normals[:, 1] * relative[:, 0],
            ))
            weighted_jacobian = jacobian * weights[:, np.newaxis]
            hessian = jacobian.T @ weighted_jacobian
            hessian += np.eye(3) * self._icp_prior_weight
            gradient = jacobian.T @ (weights * residuals)

            try:
                delta = -np.linalg.solve(hessian, gradient)
            except np.linalg.LinAlgError:
                return initial_points
            if not np.all(np.isfinite(delta)):
                return initial_points

            # Keep every iteration local even before checking the total bound.
            step_distance = float(np.linalg.norm(delta[:2]))
            maximum_step_distance = self._icp_max_translation * 0.5
            if step_distance > maximum_step_distance:
                delta[:2] *= maximum_step_distance / step_distance
            maximum_step_rotation = self._icp_max_rotation * 0.5
            delta[2] = float(np.clip(
                delta[2], -maximum_step_rotation, maximum_step_rotation))

            cosine = np.cos(delta[2])
            sine = np.sin(delta[2])
            rotation = np.asarray(
                ((cosine, -sine), (sine, cosine)), dtype=np.float64)
            corrected_points = (
                (corrected_points - pivot) @ rotation.T +
                pivot + delta[:2]
            )
            corrected_sample = (
                (corrected_sample - pivot) @ rotation.T +
                pivot + delta[:2]
            )
            accumulated_rotation += float(delta[2])

            centroid_shift = float(np.linalg.norm(
                np.mean(corrected_points, axis=0) - initial_centroid))
            if (centroid_shift > self._icp_max_translation or
                    abs(accumulated_rotation) > self._icp_max_rotation):
                return initial_points

            if (float(np.linalg.norm(delta[:2])) < 1e-4 and
                    abs(float(delta[2])) < 1e-4):
                break

        source, target, normals = self._icp_correspondences(
            corrected_sample, target_points, target_normals)
        if len(source) < self._icp_min_correspondences:
            return initial_points
        final_residuals = np.einsum('ij,ij->i', normals, source - target)
        final_rmse = float(np.sqrt(np.mean(final_residuals ** 2)))
        if (final_rmse > self._icp_max_point_to_line_rmse or
                initial_rmse - final_rmse < self._icp_min_improvement):
            return initial_points

        self.get_logger().debug(
            'Accepted point-to-line ICP: '
            f'rmse={initial_rmse:.3f}->{final_rmse:.3f} m, '
            f'rotation={np.rad2deg(accumulated_rotation):.2f} deg.')
        return corrected_points

    def _accumulate_lidar(self, message: PointCloud2, transform) -> None:
        zones = self._map_boundaries()
        if not zones:
            return

        zone_boundaries = (
            [boundary for _, boundary in zones]
            if self._accumulation_scope == 'parking_zones'
            else []
        )

        map_points = []
        for source_point in point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, _ = self._transform_point(
                float(source_point[0]), float(source_point[1]),
                float(source_point[2]), transform)
            if self._accumulation_scope == 'parking_zones':
                # Give ICP enough room to pull a slightly wrong EKF pose back
                # into the strict recording ROI. The final check remains 1 m.
                in_icp_roi = any(
                    self._inside_zone_roi(
                        (x, y), boundary,
                        self._roi_margin + self._icp_max_translation)
                    for boundary in zone_boundaries
                )
                if not in_icp_roi:
                    continue
            map_points.append((x, y))

        if not map_points:
            return
        aligned_points = self._align_scan_to_submap(
            np.asarray(map_points, dtype=np.float64))

        previous_count = len(self._occupied_voxels)
        scan_voxels = set()
        for x, y in aligned_points:
            if self._accumulation_scope == 'parking_zones':
                in_parking_roi = any(
                    self._inside_zone_roi(
                        (float(x), float(y)), boundary, self._roi_margin)
                    for boundary in zone_boundaries
                )
                if not in_parking_roi:
                    continue
            voxel = (
                floor(float(x) / self._history_resolution),
                floor(float(y) / self._history_resolution),
            )
            # Compare only against observations from previous scans. All
            # points from the current dense scan remain available to form a
            # continuous first boundary.
            if not self._is_previous_observation(voxel):
                scan_voxels.add(voxel)

        self._occupied_voxels.update(scan_voxels)

        # Publish immediately whenever the persistent boundary grows. The
        # timer republishes the latched map even when no new cell is observed.
        if len(self._occupied_voxels) != previous_count:
            self._publish_markers()

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def _base_marker(
            self, stamp, namespace: str, marker_id: int,
            marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _append_occupied_boundary(self, marker: Marker) -> None:
        """Append only exposed edges of the persistent occupied voxel map."""
        resolution = self._history_resolution
        occupied = self._occupied_voxels
        for voxel_x, voxel_y in occupied:
            x_min = voxel_x * resolution
            x_max = (voxel_x + 1) * resolution
            y_min = voxel_y * resolution
            y_max = (voxel_y + 1) * resolution
            z = self._zone_height + 0.01

            edges = []
            if (voxel_x - 1, voxel_y) not in occupied:
                edges.append(((x_min, y_min), (x_min, y_max)))
            if (voxel_x + 1, voxel_y) not in occupied:
                edges.append(((x_max, y_min), (x_max, y_max)))
            if (voxel_x, voxel_y - 1) not in occupied:
                edges.append(((x_min, y_min), (x_max, y_min)))
            if (voxel_x, voxel_y + 1) not in occupied:
                edges.append(((x_min, y_max), (x_max, y_max)))

            for start, end in edges:
                marker.points.append(self._point(start[0], start[1], z))
                marker.points.append(self._point(end[0], end[1], z))

    def _publish_markers(self) -> None:
        zones = self._map_boundaries()
        if not zones:
            return

        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self._frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for zone_index, (zone_name, boundary) in enumerate(zones):
            outline = self._base_marker(
                stamp, 'parking_zone_outline', zone_index, Marker.LINE_STRIP)
            outline.scale.x = self._line_width
            if zone_name == 'ParkingZone1':
                # LF -> LR -> RR -> RF: front side remains open.
                wall_path = [boundary[0], boundary[3], boundary[2], boundary[1]]
                outline.color.r = 0.05
                outline.color.g = 1.0
                outline.color.b = 0.20
            elif zone_name == 'ParkingZone2':
                # LF -> RF -> RR -> LR: left side remains open.
                wall_path = [boundary[0], boundary[1], boundary[2], boundary[3]]
                outline.color.r = 0.05
                outline.color.g = 0.65
                outline.color.b = 1.0
            else:
                wall_path = boundary
                outline.color.r = 1.0
                outline.color.g = 1.0
                outline.color.b = 0.1
            outline.color.a = 1.0
            for x, y in wall_path:
                outline.points.append(
                    self._point(x, y, self._zone_height))
            marker_array.markers.append(outline)

            label = self._base_marker(
                stamp, 'parking_zone_label', zone_index,
                Marker.TEXT_VIEW_FACING)
            label.pose.position.x = (
                sum(point[0] for point in boundary) / len(boundary))
            label.pose.position.y = (
                sum(point[1] for point in boundary) / len(boundary))
            label.pose.position.z = self._zone_height + 0.1
            label.scale.z = 0.8
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = zone_name
            marker_array.markers.append(label)

        datum_easting, datum_northing = self._datum
        gates = (
            ('ZONE1 A', self._zone1_gate_a_utm, (1.0, 1.0, 0.0)),
            ('ZONE1 B', self._zone1_gate_b_utm, (1.0, 0.4, 1.0)),
            ('ZONE2 A', self._zone2_gate_a_utm, (1.0, 1.0, 0.0)),
            ('ZONE2 B', self._zone2_gate_b_utm, (1.0, 0.4, 1.0)),
        )
        for gate_id, (gate_name, gate, color) in enumerate(gates):
            # Do not draw an unconfigured [0, 0, 0, 0] gate.
            if gate[0] == gate[1]:
                continue
            map_gate = [
                (
                    point[0] - datum_easting,
                    -(point[1] - datum_northing),
                )
                for point in gate
            ]
            gate_marker = self._base_marker(
                stamp, 'parking_mode_gates', gate_id,
                Marker.LINE_STRIP)
            gate_marker.scale.x = self._line_width * 1.5
            gate_marker.color.r = color[0]
            gate_marker.color.g = color[1]
            gate_marker.color.b = color[2]
            gate_marker.color.a = 1.0
            for x, y in map_gate:
                gate_marker.points.append(
                    self._point(x, y, self._zone_height + 0.02))
            marker_array.markers.append(gate_marker)

            gate_label = self._base_marker(
                stamp, 'parking_mode_gate_labels', gate_id,
                Marker.TEXT_VIEW_FACING)
            gate_label.pose.position.x = (
                map_gate[0][0] + map_gate[1][0]) / 2.0
            gate_label.pose.position.y = (
                map_gate[0][1] + map_gate[1][1]) / 2.0
            gate_label.pose.position.z = self._zone_height + 0.1
            gate_label.scale.z = 0.6
            gate_label.color.r = color[0]
            gate_label.color.g = color[1]
            gate_label.color.b = color[2]
            gate_label.color.a = 1.0
            gate_label.text = f'PARKING {gate_name}'
            marker_array.markers.append(gate_label)

        if self._occupied_voxels:
            history = self._base_marker(
                stamp, 'global_lidar_boundary_map', 0,
                Marker.LINE_LIST)
            history.scale.x = self._lidar_line_width
            history.color.r = 1.0
            history.color.g = 0.35
            history.color.b = 0.0
            history.color.a = 1.0
            self._append_occupied_boundary(history)
            marker_array.markers.append(history)

        self._publisher.publish(marker_array)


# Backward-compatible class name for code importing the former module API.
ParkingZoneVisualizer = ZoneScan


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZoneScan()
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
