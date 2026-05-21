#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import json
import logging
import math
import time

import carla


def _find_existing_vehicle(world, vehicle_id, timeout, actor_id=None):
    if actor_id is None and not vehicle_id:
        raise ValueError("The config file must define a vehicle id when using --attach-existing")

    if actor_id is not None:
        logging.info("Looking for an existing vehicle actor id %s", actor_id)
    else:
        logging.info("Looking for an existing vehicle named '%s'", vehicle_id)

    deadline = time.monotonic() + timeout

    while True:
        if actor_id is not None:
            matches = [
                actor for actor in world.get_actors().filter("vehicle.*")
                if actor.id == actor_id
            ]
        else:
            matches = [
                actor for actor in world.get_actors().filter("vehicle.*")
                if actor.attributes.get("role_name") == vehicle_id or
                actor.attributes.get("ros_name") == vehicle_id
            ]

        if matches:
            if len(matches) > 1:
                logging.warning(
                    "Found %s matching vehicles for '%s'; using actor id %s",
                    len(matches), vehicle_id, matches[0].id)
            logging.info(
                "Attaching to vehicle actor id=%s type=%s role_name=%s",
                matches[0].id,
                matches[0].type_id,
                matches[0].attributes.get("role_name", ""))
            return matches[0]

        if time.monotonic() >= deadline:
            if actor_id is not None:
                raise RuntimeError("Could not find vehicle actor id {}".format(actor_id))

            raise RuntimeError(
                "Could not find a vehicle with role_name or ros_name '{}'".format(vehicle_id))

        time.sleep(0.5)


def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    return world.spawn_actor(
        bp,
        map_.get_spawn_points()[0],
        attach_to=None)


def _setup_sensors(world, vehicle, sensors_config, enable_native_ros=True):
    bp_library = world.get_blueprint_library()

    sensors = []
    for sensor in sensors_config:
        logging.debug("Spawning sensor: {}".format(sensor))

        bp = bp_library.filter(sensor.get("type"))[0]
        bp.set_attribute("ros_name", sensor.get("id"))
        bp.set_attribute("role_name", sensor.get("id"))
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"]),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"])
        )

        sensors.append(
            world.spawn_actor(bp, wp, attach_to=vehicle)
        )
        if enable_native_ros:
            sensors[-1].enable_for_ros()

    return sensors


def _quaternion_from_euler(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PythonRos2Publisher:
    def __init__(self, vehicle_id, base_frame, sensors, sensors_config, vehicle=None):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2, PointField
        from std_msgs.msg import Header
        from tf2_ros import StaticTransformBroadcaster
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self.rclpy = rclpy
        self.Image = Image
        self.Imu = Imu
        self.NavSatFix = NavSatFix
        self.PointCloud2 = PointCloud2
        self.PointField = PointField
        self.Header = Header
        self.TransformStamped = TransformStamped
        self.sensors = sensors
        self.active = True
        self._sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("carla_ros2_sensor_python")
        self.static_tf_broadcaster = StaticTransformBroadcaster(self.node)

        self._publish_static_transforms(base_frame, sensors_config, vehicle)
        self._start_publishers(vehicle_id, sensors, sensors_config)

    def _now(self):
        return self.node.get_clock().now().to_msg()

    def _publish_static_transforms(self, base_frame, sensors_config, vehicle=None):
        transforms = []
        stamp = self._now()

        for sensor_config in sensors_config:
            sensor_id = sensor_config.get("id")
            spawn_point = sensor_config.get("spawn_point", {})
            if not sensor_id:
                continue

            transform = self.TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = base_frame
            transform.child_frame_id = sensor_id
            transform.transform.translation.x = float(spawn_point.get("x", 0.0))
            transform.transform.translation.y = float(spawn_point.get("y", 0.0))
            transform.transform.translation.z = float(spawn_point.get("z", 0.0))

            qx, qy, qz, qw = _quaternion_from_euler(
                float(spawn_point.get("roll", 0.0)),
                float(spawn_point.get("pitch", 0.0)),
                float(spawn_point.get("yaw", 0.0)))
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)

        if vehicle is not None:
            transforms.extend(self._wheel_transforms(base_frame, stamp, vehicle))

        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def _wheel_transforms(self, base_frame, stamp, vehicle):
        vt = vehicle.get_transform()
        vx, vy, vz = vt.location.x, vt.location.y, vt.location.z
        yaw = math.radians(vt.rotation.yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        transforms = []
        wheel_names = ['fl', 'fr', 'rl', 'rr']
        for wheel, name in zip(vehicle.get_physics_control().wheels, wheel_names):
            dx = wheel.position.x / 100.0 - vx
            dy = wheel.position.y / 100.0 - vy
            dz = wheel.position.z / 100.0 - vz

            # World displacement → vehicle-local CARLA frame (inverse yaw rotation)
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -(- dx * sin_yaw + dy * cos_yaw)  # negate for CARLA→ROS Y

            t = self.TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = base_frame
            t.child_frame_id = "microlino/wheel_{}".format(name)
            t.transform.translation.x = local_x
            t.transform.translation.y = local_y
            t.transform.translation.z = dz
            t.transform.rotation.w = 1.0
            transforms.append(t)

        return transforms

    def _start_publishers(self, vehicle_id, sensors, sensors_config):
        for sensor, sensor_config in zip(sensors, sensors_config):
            sensor_type = sensor_config.get("type", "")
            sensor_id = sensor_config.get("id", sensor.type_id.rsplit(".", 1)[-1])
            topic_prefix = "/carla/{}/{}".format(vehicle_id, sensor_id)

            if sensor_type.startswith("sensor.camera."):
                publisher = self.node.create_publisher(
                    self.Image, "{}/image".format(topic_prefix), self._sensor_qos)
                sensor.listen(
                    lambda data, pub=publisher, frame_id=sensor_id:
                    self._publish_image(pub, frame_id, data))

            elif sensor_type == "sensor.lidar.ray_cast":
                publisher = self.node.create_publisher(
                    self.PointCloud2, "{}/point_cloud".format(topic_prefix), self._sensor_qos)
                sensor.listen(
                    lambda data, pub=publisher, frame_id=sensor_id:
                    self._publish_lidar(pub, frame_id, data))

            elif sensor_type == "sensor.other.gnss":
                publisher = self.node.create_publisher(
                    self.NavSatFix, "{}/fix".format(topic_prefix), self._sensor_qos)
                sensor.listen(
                    lambda data, pub=publisher, frame_id=sensor_id:
                    self._publish_gnss(pub, frame_id, data))

            elif sensor_type == "sensor.other.imu":
                publisher = self.node.create_publisher(
                    self.Imu, "{}/data".format(topic_prefix), self._sensor_qos)
                sensor.listen(
                    lambda data, pub=publisher, frame_id=sensor_id:
                    self._publish_imu(pub, frame_id, data))

    def _publish_image(self, publisher, frame_id, image):
        if not self.active:
            return

        msg = self.Image()
        msg.header.stamp = self._now()
        msg.header.frame_id = frame_id
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "bgra8"
        msg.is_bigendian = False
        msg.step = image.width * 4
        msg.data = bytes(image.raw_data)
        self._publish(publisher, msg)

    def _publish_lidar(self, publisher, frame_id, lidar):
        if not self.active:
            return

        msg = self.PointCloud2()
        msg.header.stamp = self._now()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(lidar.raw_data) // 16
        msg.fields = [
            self.PointField(name="x", offset=0, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name="y", offset=4, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name="z", offset=8, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name="intensity", offset=12, datatype=self.PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Negate Y (CARLA +Y right → ROS +Y left): flip sign bit of each Y float32.
        # Each point is 16 bytes [x:4][y:4][z:4][i:4]; Y sign bit is byte 7 (little-endian).
        raw = bytearray(lidar.raw_data)
        for i in range(7, len(raw), 16):
            raw[i] ^= 0x80
        msg.data = bytes(raw)

        self._publish(publisher, msg)

    def _publish_gnss(self, publisher, frame_id, gnss):
        if not self.active:
            return

        msg = self.NavSatFix()
        msg.header.stamp = self._now()
        msg.header.frame_id = frame_id
        msg.latitude = gnss.latitude
        msg.longitude = gnss.longitude
        msg.altitude = gnss.altitude
        self._publish(publisher, msg)

    def _publish_imu(self, publisher, frame_id, imu):
        if not self.active:
            return

        msg = self.Imu()
        msg.header.stamp = self._now()
        msg.header.frame_id = frame_id
        msg.linear_acceleration.x = imu.accelerometer.x
        msg.linear_acceleration.y = imu.accelerometer.y
        msg.linear_acceleration.z = imu.accelerometer.z
        msg.angular_velocity.x = imu.gyroscope.x
        msg.angular_velocity.y = imu.gyroscope.y
        msg.angular_velocity.z = imu.gyroscope.z
        msg.orientation.w = 1.0
        self._publish(publisher, msg)

    def _publish(self, publisher, msg):
        try:
            publisher.publish(msg)
        except RuntimeError as error:
            logging.debug("ROS2 publish skipped: %s", error)

    def spin_once(self, timeout_sec=0.1):
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def shutdown(self):
        self.active = False

        for sensor in self.sensors:
            try:
                sensor.stop()
            except RuntimeError:
                pass

        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main(args):

    world = None
    vehicle = None
    vehicle_owned = False
    sensors = []
    ros2_publisher = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        world = client.get_world()

        if not args.passive:
            original_settings = world.get_settings()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        with open(args.file) as f:
            config = json.load(f)

        # Filter sensors by --sensors argument if provided
        sensors_config = config.get("sensors", [])
        if args.sensors:
            selected = set(args.sensors)
            sensors_config = [s for s in sensors_config if s.get("id") in selected]
            logging.info("Active sensors: %s", [s.get("id") for s in sensors_config])

        if args.attach_existing:
            vehicle = _find_existing_vehicle(
                world, config.get("id"), args.wait_for_vehicle, args.attach_actor_id)
        else:
            vehicle = _setup_vehicle(world, config)
            vehicle_owned = True

        sensors = _setup_sensors(world, vehicle, sensors_config,
                                  enable_native_ros=not args.python_ros2)

        if args.python_ros2:
            ros2_publisher = PythonRos2Publisher(
                config.get("id"), args.base_frame, sensors, sensors_config, vehicle=vehicle)

        if args.passive:
            logging.info("Running in passive mode. Keeping ROS2 sensors alive without ticking the world...")
            while True:
                if ros2_publisher:
                    ros2_publisher.spin_once(0.1)
                else:
                    time.sleep(1.0)

        _ = world.tick()

        if vehicle_owned and args.autopilot:
            vehicle.set_autopilot(True)

        logging.info("Running...")

        while True:
            _ = world.tick()
            if ros2_publisher:
                ros2_publisher.spin_once(0.0)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    finally:
        if ros2_publisher:
            ros2_publisher.shutdown()

        if original_settings:
            world.apply_settings(original_settings)

        for sensor in sensors:
            try:
                sensor.destroy()
            except RuntimeError as error:
                logging.debug("Sensor cleanup skipped: %s", error)

        if vehicle and vehicle_owned:
            try:
                vehicle.destroy()
            except RuntimeError as error:
                logging.debug("Vehicle cleanup skipped: %s", error)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CARLA ROS2 sensor publisher')
    argparser.add_argument('--host', metavar='H', default='localhost',
                           help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int,
                           help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', default='', required=True,
                           help='Sensor config JSON file')
    argparser.add_argument(
        '--sensors',
        nargs='+',
        default=None,
        metavar='SENSOR_ID',
        help='sensor IDs to activate (default: all). Example: --sensors lidar gnss imu')
    argparser.add_argument(
        '--attach-existing',
        action='store_true',
        help='attach sensors to an existing vehicle matching the config id')
    argparser.add_argument(
        '--wait-for-vehicle',
        default=0.0,
        type=float,
        help='seconds to wait for the existing vehicle when using --attach-existing')
    argparser.add_argument(
        '--attach-actor-id',
        default=None,
        type=int,
        help='attach to a specific vehicle actor id')
    argparser.add_argument(
        '--passive',
        action='store_true',
        help='do not change synchronous settings or tick the world')
    argparser.add_argument(
        '--python-ros2',
        action='store_true',
        help='publish ROS2 messages from Python callbacks')
    argparser.add_argument(
        '--base-frame',
        default='base_link',
        help='parent TF frame for the vehicle body when using --python-ros2')
    argparser.add_argument(
        '--no-autopilot',
        action='store_false',
        dest='autopilot',
        help='do not enable autopilot for a vehicle spawned by this script')
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug',
                           help='print debug information')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('Listening to server %s:%s', args.host, args.port)

    main(args)
