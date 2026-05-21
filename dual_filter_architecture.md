# ROS 2 Humble: 자율주행 듀얼 필터(Dual Filter) 기반 센서 퓨전 아키텍처 구축 매뉴얼

내가 사용 가능한 ros2 topic은 다음과 같다.

센서	토픽	ROS2 메시지 타입
RGB 카메라	/carla/car/rgb/image	sensor_msgs/msg/Image
LiDAR	/carla/car/lidar/point_cloud	sensor_msgs/msg/PointCloud2
GNSS	/carla/car/gnss/fix	sensor_msgs/msg/NavSatFix
IMU	/carla/car/imu/data	sensor_msgs/msg/Imu


## 1. 시스템 개요 (System Overview)
본 매뉴얼은 실외 자율주행 차량의 정밀 측위 및 제어 안정성 확보를 위해 **REP-105 표준**을 준수하는 듀얼 필터(Dual Filter) 아키텍처를 구축하는 지침이다. 단일 필터에 GNSS를 결합할 경우 발생하는 위치 도약(Jump) 현상으로 인한 제어기 발산을 방지하기 위해 로컬 필터와 글로벌 필터를 분리하여 구성한다.

*   **Target OS / Middleware:** Ubuntu 22.04 / ROS 2 Humble
*   **Target Package:** `robot_localization`
*   **Sensor Inputs:** Wheel Encoder, 6-DOF/9-DOF IMU, RTK GNSS (u-blox F9P 등)

---

## 2. 좌표계(TF Tree) 명세 (REP-105 준수)
시스템은 다음의 계층적 TF 트리를 반드시 유지해야 한다.
`map` $\rightarrow$ `odom` $\rightarrow$ `base_link`

1.  **`base_link`:** 차량의 뒷바퀴 축 중심(또는 무게중심)을 나타내는 로컬 물리 프레임.
2.  **`odom`:** 시작 위치를 원점으로 하는 로컬 기준 프레임. 연속적이고 부드러운 궤적을 보장하나 장기 드리프트(Drift)가 존재함. (단기 제어 및 장애물 회피용)
3.  **`map`:** GNSS 기반의 전역 기준 프레임. 위치의 도약(Jump)이 발생할 수 있으나 지구상의 절대 위치를 보장함. (전역 경로 추종용)

---

## 3. 센서 토픽 및 메시지 타입 명세
시스템 외부(하드웨어 드라이버)에서 발행되어야 하는 필수 토픽 목록이다.

| 센서 | 토픽 이름 | 메시지 타입 | 필수 데이터 필드 |
| :--- | :--- | :--- | :--- |
| **휠 엔코더** | `/wheel/odom` | `nav_msgs/Odometry` | `twist.twist.linear.x`, `twist.twist.angular.z` |
| **IMU** | `/imu/data` | `sensor_msgs/Imu` | `angular_velocity` (x,y,z), `linear_acceleration` (x,y) |
| **GNSS** | `/gps/fix` | `sensor_msgs/NavSatFix` | `latitude`, `longitude`, `altitude`, `status` |

---

## 4. 노드별 파라미터 및 아키텍처 설계

시스템은 총 3개의 핵심 노드로 구성된다.

### Node 1: 로컬 필터 (`ekf_node` - Local)
*   **목적:** 차량의 제어기(DWA, Pure Pursuit)에 사용될 지연 없고 부드러운 오도메트리 생성.
*   **입력:** `/wheel/odom`, `/imu/data`
*   **출력:** `/odometry/local` 토픽, `odom` $\rightarrow$ `base_link` TF 발행
*   **설정 원칙:** 절대 위치(x, y)를 직접 관측하지 않으며, 속도(미분값)와 각속도를 적분하여 궤적을 추정한다.

### Node 2: GNSS 좌표 변환기 (`navsat_transform_node`)
*   **목적:** WGS84(위경도) 좌표계를 시스템이 이해할 수 있는 직교 좌표계(ENU, 미터 단위)로 변환.
*   **입력:** `/gps/fix`, `/imu/data`, `/odometry/global` (글로벌 필터의 피드백)
*   **출력:** `/odometry/gps` 토픽 (GNSS 위치를 x, y 미터 좌표로 변환한 Odometry)

### Node 3: 글로벌 필터 (`ekf_node` - Global)
*   **목적:** 맵 상의 절대 위치 파악 및 로컬 필터의 장기 드리프트 보정.
*   **입력:** `/wheel/odom`, `/imu/data`, `/odometry/gps` (Node 2의 출력)
*   **출력:** `/odometry/global` 토픽, `map` $\rightarrow$ `odom` TF 발행
*   **설정 원칙:** 센서 값들로부터 `map` 좌표계 상의 차량 위치(`base_link`)를 계산한 뒤, 로컬 필터가 이미 발행 중인 `odom` $\rightarrow$ `base_link`를 방해하지 않기 위해 내부적으로 `map` $\rightarrow$ `odom` 간의 오프셋(TF)만을 계산하여 발행한다.

---

## 5. YAML 파라미터 파일 템플릿 (`ekf_params.yaml`)
작업 디렉토리: `[your_package]/config/ekf_params.yaml`

```yaml
local_ekf:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true               # 평면 주행(2D) 강제 적용
    publish_tf: true               # odom -> base_link TF 발행 활성화
    
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom              # 기준 프레임을 odom으로 설정

    # Wheel Encoder 설정 (X축 선속도, Z축 각속도만 사용)
    odom0: /wheel/odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    # IMU 설정 (방위각, 각속도, 선형 가속도 사용)
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]

global_ekf:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true               # map -> odom TF 발행 활성화
    
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: map               # 기준 프레임을 map으로 설정

    odom0: /wheel/odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]

    # 변환된 GPS 데이터 설정 (X, Y 절대 위치만 사용)
    odom1: /odometry/gps
    odom1_config: [true,  true,  false,
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]

navsat_transform:
  ros__parameters:
    frequency: 30.0
    delay: 3.0                     # 초기 TF 트리 형성 대기 시간
    magnetic_declination_radians: 0.1501 # 한국 지역의 자북 편각 (약 8.6도) -> 지역에 맞게 수정 필요
    yaw_offset: 1.5707963          # IMU 0도가 동쪽(ENU)을 바라보도록 맞추는 오프셋 (PI/2)
    zero_altitude: true
    broadcast_cartesian_transform: true
    publish_filtered_gps: true
    use_odometry_yaw: false