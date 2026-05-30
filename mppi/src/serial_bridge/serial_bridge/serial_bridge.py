#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import serial, threading, time

DEFAULT_PORT = '/dev/arduino_bridge'
DEFAULT_BAUD = 57600

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # 파라미터 선언
        self.declare_parameter('throttle_topic', '/auto_throttle')
        self.declare_parameter('steer_cmd_topic', '/auto_steer_angle')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('startup_silence_sec', 3.0)   # ★ 시작 후 송신 차단 시간(초)

        throttle_topic = self.get_parameter('throttle_topic').get_parameter_value().string_value
        steer_topic    = self.get_parameter('steer_cmd_topic').get_parameter_value().string_value
        port           = self.get_parameter('port').get_parameter_value().string_value or DEFAULT_PORT
        baud           = self.get_parameter('baud').get_parameter_value().integer_value or DEFAULT_BAUD
        self.startup_silence_sec = float(self.get_parameter('startup_silence_sec').value)  # ★

        # 시리얼 핸들/락
        self.ser = None
        self._ser_lock = threading.Lock()
        self._stop = False

        # ★ 시작 시각 및 1회성 로그 플래그
        self._start_time = time.time()
        self._silence_logged = False

        # 재연결 스레드 (open)
        self._recon_th = threading.Thread(target=self._reconnect_loop, args=(port, baud), daemon=True)
        self._recon_th.start()

        # 수신 스레드 (아두이노 → 호스트) : 아두이노 Serial.print() 를 ROS 로그로 보여줌
        self._rx_th = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_th.start()

        # 구독자 (ROS2 → 아두이노 명령 전송)
        self.create_subscription(Float32, throttle_topic, self.cb_throttle, 10)
        self.create_subscription(Float32, steer_topic, self.cb_steer, 10)

        # 발행자 (아두이노 → ROS2 오도메트리)
        # 아두이노 시리얼에서 VX(전륜 선속도)와 PS(실제 조향각)를 파싱하여
        # v_rear = VX × cos(PS × π/180) 로 보정한 뒤 /wheel/odom으로 발행한다.
        # 이 토픽은 local_ekf / global_ekf 의 odom0 입력으로 사용된다.
        self._odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        self.get_logger().info(
            f"Subscribed to {throttle_topic} and {steer_topic} → Serial({port}@{baud}), "
            f"startup_silence_sec={self.startup_silence_sec:.1f}s"
        )

    # 시리얼 열기/재연결 루프
    def _reconnect_loop(self, port, baud):
        while not self._stop:
            if self.ser is None:
                try:
                    self.get_logger().info(f"Opening serial: {port}@{baud}")
                    s = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=0.2)
                    time.sleep(0.2)  # 보드 리셋 안정화
                    with self._ser_lock:
                        self.ser = s
                    self.get_logger().info("Serial connected.")
                except Exception as e:
                    self.get_logger().warn(f"Serial open failed: {e}")
                    time.sleep(1.0)
            time.sleep(0.1)

    def _parse_field(self, text: str, key: str):
        """
        시리얼 한 줄에서 "KEY:value" 형식의 숫자를 파싱한다.
        예) "... | VX:0.5123 | PS:-8.45 ..." 에서
            _parse_field(text, "VX") → 0.5123
            _parse_field(text, "PS") → -8.45
        해당 키가 없거나 파싱 실패 시 None 반환.
        """
        tag = f"{key}:"
        idx = text.find(tag)
        if idx == -1:
            return None
        start = idx + len(tag)
        # 값 끝: 공백, '|', 줄끝 중 먼저 오는 위치
        end = len(text)
        for ch in (' ', '|', '\r', '\n'):
            pos = text.find(ch, start)
            if pos != -1 and pos < end:
                end = pos
        try:
            return float(text[start:end])
        except ValueError:
            return None

    def _publish_wheel_odom(self, vx_raw: float, ps_deg: float):
        """
        전륜 엔코더 선속도(vx_raw)와 실제 조향각(ps_deg)으로
        후륜축 기준 선속도를 계산하여 /wheel/odom 을 발행한다.

        보정 공식 (자전거 모델):
            v_encoder = v_rear / cos(δ)
            ∴ v_rear  = v_encoder × cos(δ)

        /wheel/odom 은 EKF 의 속도 측정 입력(vx, vy=0)으로만 사용된다.
        pose 필드는 EKF 가 내부적으로 적분하므로 여기서는 채우지 않는다.
        """
        v_rear = vx_raw * math.cos(math.radians(ps_deg))

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        msg.twist.twist.linear.x  = v_rear   # 후륜축 전방 속도 (m/s)
        msg.twist.twist.linear.y  = 0.0      # 비홀로노믹 제약: 횡방향 속도 없음
        msg.twist.twist.angular.z = 0.0      # yaw rate 는 IMU 에서 별도 공급

        # 공분산 설정 (robot_localization 에서 측정값 가중치로 사용)
        # twist.covariance 는 6×6 행렬을 행 우선으로 펼친 36개 원소
        # [0]  = vx 분산:  RTK급 엔코더 기준 ~0.05 m²/s²
        # [7]  = vy 분산:  vy=0 제약을 강하게 신뢰 → 0.01
        # [35] = wz 분산:  IMU 에서 따로 오므로 크게 설정 (EKF 가 무시)
        cov = [0.0] * 36
        cov[0]  = 0.05   # vx
        cov[7]  = 0.01   # vy = 0 (비홀로노믹 제약)
        cov[35] = 1e6    # wz (IMU 에서 공급, 이 토픽에서는 사용 안 함)
        msg.twist.covariance = cov

        self._odom_pub.publish(msg)

    # 수신 루프: 아두이노에서 오는 시리얼 데이터를 읽어 파싱·발행
    def _reader_loop(self):
        buf = b""
        while not self._stop:
            with self._ser_lock:
                s = self.ser
            if s is None:
                time.sleep(0.1)
                continue
            try:
                data = s.read(128)
                if not data:
                    time.sleep(0.01)
                    continue
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.replace(b'\r', b'')
                    text = line.decode('utf-8', errors='replace')
                    if not text:
                        continue

                    self.get_logger().debug(f"RX: {text}")

                    # VX, PS 가 모두 포함된 줄이면 /wheel/odom 발행
                    vx = self._parse_field(text, 'VX')
                    ps = self._parse_field(text, 'PS')
                    if vx is not None and ps is not None:
                        self._publish_wheel_odom(vx, ps)
                    else:
                        # VX/PS 없는 줄(초기 디버그 줄 등)은 INFO 로그만
                        self.get_logger().info(f"RX: {text}")

            except Exception as e:
                self.get_logger().warn(f"Serial read failed: {e}")
                with self._ser_lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.2)

    # ★ 시작 후 송신 차단 여부
    def _in_startup_silence(self) -> bool:
        elapsed = time.time() - self._start_time
        if elapsed < self.startup_silence_sec:
            # 1회만 남은 시간 안내
            if not self._silence_logged:
                self._silence_logged = True
                remain = self.startup_silence_sec - elapsed
                self.get_logger().info(
                    f"Startup silence... (no serial writes for {remain:.1f}s more)"
                )
            return True
        return False

    # 한 줄 쓰기
    def _write_line(self, line: str):
        # ★ 시작 후 silence 동안은 송신 차단
        if self._in_startup_silence():
            return False
        with self._ser_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(line.encode('utf-8'))
                self.ser.flush()
                return True
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return False

    # 토픽 콜백들 (TX 로깅 포함)
    def cb_throttle(self, msg: Float32):
        val = float(msg.data)
        if val > 1.0:  val = 1.0
        if val < -1.0: val = -1.0
        line = f"TH {val:.3f}\n"
        self.get_logger().info(f"TX: {line.strip()}")
        self._write_line(line)

    def cb_steer(self, msg: Float32):
        ang = float(msg.data)
        line = f"SA {ang:.3f}\n"
        self.get_logger().info(f"TX: {line.strip()}")
        self._write_line(line)

    def destroy_node(self):
        self._stop = True
        # 종료 대기
        try:
            self._recon_th.join(timeout=0.5)
            self._rx_th.join(timeout=0.5)
        except Exception:
            pass
        with self._ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()은 내부에서 두 번 호출되면 에러가 나므로 안전 가드
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
