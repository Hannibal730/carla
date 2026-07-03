# ROS 2 Humble: 자율주행 듀얼 필터(Dual Filter) 기반 센서 퓨전 아키텍처 구축 매뉴얼

---

## 목차

| 섹션 | 제목 |
| :---: | :--- |
| [1](#1-시스템-개요-system-overview) | 시스템 개요 |
| [2](#2-좌표계tf-tree-명세-rep-105-준수) | 좌표계(TF Tree) 명세 |
| [3](#3-센서-토픽-및-메시지-타입-명세) | 센서 토픽 및 메시지 타입 명세 |
| [4](#4-노드별-파라미터-및-아키텍처-설계) | 노드별 파라미터 및 아키텍처 설계 |
| [5](#5-yaml-파라미터-파일-템플릿-ekf_paramsyaml) | YAML 파라미터 파일 템플릿 |
| [6](#6-구현-결과-implementation) | 구현 결과 |
| [**7**](#7-빌드-및-의존성-설치) | **빌드 및 의존성 설치** |
| [8](#8-nav2-mppi-controller-연동-분석) | Nav2 MPPI Controller 연동 분석 |
| [9](#9-실제-하드웨어-휠-오도메트리-구현) | 실제 하드웨어 휠 오도메트리 구현 |
| [10](#10-구현-컴포넌트-상세) | 구현 컴포넌트 상세 |
| [11](#11-레퍼런스-맵-제작-mppi-추종-경로-생성) | 레퍼런스 맵 제작 |
| [12](#12-시스템-실행-매뉴얼) | 시스템 실행 매뉴얼 |
| [13](#13-파라미터-상세-설명-및-튜닝-가이드) | 파라미터 상세 설명 및 튜닝 가이드 (13.7: ParkingPath 주차 전용 플러그인 포함) |
| [14](#14-tf-시간적-허용-오차-tf-temporal-tolerance) | TF 시간적 허용 오차 |

---

## 1. 시스템 개요 (System Overview)

본 매뉴얼은 실외 자율주행 차량의 정밀 측위 및 제어 안정성 확보를 위해 **REP-105 표준**을 준수하는 듀얼 필터(Dual Filter) 아키텍처를 구축하는 지침이다. 단일 필터에 GNSS를 결합할 경우 발생하는 위치 도약(Jump) 현상으로 인한 제어기 발산을 방지하기 위해 로컬 필터와 글로벌 필터를 분리하여 구성한다.

*   **Target OS / Middleware:** Ubuntu 22.04 / ROS 2 Humble
*   **Target Package:** `robot_localization`
*   **Sensor Inputs:** CARLA Vehicle API (→ `/carla/car/wheel_encoder/data`), 6-DOF IMU, Dual RTK GNSS (f9r / f9p)
*   **Controller:** MPPI

### 1.1 왜 로컬/글로벌 두 필터로 나누는가 (설계 목적)

단일 필터에 GNSS를 넣으면 **"제어 안정성"과 "절대 정확도"를 동시에 만족할 수 없다.** GNSS는 정확하지만 신호 반사·보정으로 위치가 순간적으로 튄다(Jump). 이 튐이 필터 출력에 직접 반영되면 다음 문제가 생긴다.

```
GNSS가 순간 1m 튐
  → 필터가 계산한 차량 위치도 1m 순간이동
  → MPPI가 잘못된 초기 상태에서 rollout 계산 → 조향/가감속 발산
```

반대로 GNSS를 빼면 부드럽지만 시간이 지날수록 실제 위치와 어긋난다(드리프트). **하나의 필터로는 "안 튀는 부드러움"과 "GNSS 절대 정확도"를 둘 다 줄 수 없다.** 그래서 역할을 둘로 분리한다.

| 필터 | 담당 | GNSS | 발행 TF | 소비자 |
| :--- | :--- | :--- | :--- | :--- |
| **로컬 EKF** | **부드러움** — 안 튀는 연속적 오도메트리 | ❌ 안 씀 | `odom → base_link` | MPPI 로컬 제어 |
| **글로벌 EKF** | **절대 정확도** — 지구상 실제 위치 | ✅ 씀 | `utm → odom` | 전역 경로 추종 |

**핵심 트릭 — GNSS의 Jump를 제어기로부터 격리한다.** 글로벌 EKF는 GNSS 보정을 차량 위치가 아니라 **`utm → odom` 오프셋(지도와 출발점 사이의 어긋남)에만** 조용히 반영한다. 그래서 GNSS가 튀어도 제어기가 보는 `odom → base_link`는 전혀 변하지 않는다.

```
utm ──[글로벌 EKF]──> odom ──[로컬 EKF]──> base_link
      GNSS로 여기만 보정          절대 안 튐 (MPPI가 봄)

GNSS 튐 → utm→odom 만 조정 → odom→base_link(제어용)는 그대로
       → MPPI는 아무것도 감지 못 함 → 안정적 제어
```

비유하면 **로컬 EKF는 출발점 기준 "몇 걸음 걸었나"를 세는 만보기**(절대 안 틀리고 부드럽다)이고, **글로벌 EKF는 가끔 GPS를 보고 "출발점 자체가 2m 어긋나 있었네" 하며 지도를 슬쩍 미는 역할**이다. 만보기(제어기가 보는 값)는 건드리지 않고 지도만 민다. 이것이 본 매뉴얼 제목인 **듀얼 필터 아키텍처**의 존재 이유다. 상세 근거는 [4절 Node 2/3](#node-2-로컬-필터-ekf_node---local)에서 다룬다.

### 1.2 활용 가능한 CARLA 센서 토픽

| 센서 | 토픽 | ROS2 메시지 타입 |
| :--- | :--- | :--- |
| CARLA simulation clock | `/clock` | `rosgraph_msgs/msg/Clock` |
| RGB 카메라 | `/carla/car/rgb/image` | `sensor_msgs/msg/Image` |
| 후방 카메라 (주차용) | `/carla/car/rear_cam/image` | `sensor_msgs/msg/Image` |
| LiDAR (3D) | `/carla/car/lidar_3d/point_cloud` | `sensor_msgs/msg/PointCloud2` |
| LiDAR (2D) | `/carla/car/lidar_2d/point_cloud` | `sensor_msgs/msg/PointCloud2` |
| GNSS (후륜축) | `/carla/car/f9r/fix` | `sensor_msgs/msg/NavSatFix` |
| GNSS (전방 1.4m) | `/carla/car/f9p/fix` | `sensor_msgs/msg/NavSatFix` |
| IMU | `/carla/car/imu/data` | `sensor_msgs/msg/Imu` |

---

## 2. 좌표계(TF Tree) 명세 (REP-105 준수)

시스템은 다음의 계층적 TF 트리를 반드시 유지해야 한다.

```
utm ──[global_ekf]──> odom ──[local_ekf]──> base_link
```

본 시스템의 TF 트리는 dual_filter 스택이 담당한다. 경로 파일 처리는 `csv_to_utm` 노드가 `/utm_datum`을 공유받아 `/csv_path`를 `utm` 프레임으로 발행한다.

---

### 2.1 `base_link` — 차량 물리 프레임

**원점:** 차량 후륜축 중심. 차량이 움직이면 프레임도 함께 이동한다.

```
차량이 1m 전진 → base_link 원점도 1m 전진
차량이 우회전  → base_link 원점도 우회전
```

- 방향: X = 차량 전방, Y = 좌측(ROS 표준), Z = 위
- 이 프레임 자체는 "어디에 있는지" 정보를 갖지 않는다. 항상 다른 프레임과의 TF 관계(변환)로만 위치가 정해진다.
- CARLA는 내부적으로 `+Y=right` 를 사용하므로, `ros2_sensor.py`에서 Y축과 yaw 부호를 반전해 ROS 표준(`+Y=left`)으로 맞춘다.

**TF에서의 역할:** `odom → base_link` 변환이 "출발점에서 차량이 얼마나 이동했는가"를 표현한다.

---

### 2.2 `odom` — 로컬 오도메트리 기준점

**원점:** 시스템을 실행한 순간, 차량이 있던 위치와 방향. 이 원점은 이후 절대로 움직이지 않는다.

```
시스템 시작 시
  odom 원점 = 차량의 현재 물리적 위치 (지구상의 어느 한 점)
  odom → base_link = 항등변환(identity), 즉 차량은 odom 원점 위에 있음

500m 주행 후
  odom 원점은 여전히 같은 물리적 위치에 고정
  odom → base_link = 출발점으로부터 500m 이동한 위치
```

- **발행 주체:** `local_ekf` (`world_frame: odom`, `publish_tf: true`)
- **계산 방법:** wheel `vx` + IMU `wz`의 적분
- **장점:** 연속적이고 부드럽다 — GNSS 노이즈나 보정으로 인한 갑작스러운 위치 변화(Jump)가 없다. MPPI 제어기가 이 프레임 기반으로 동작한다.
- **단점:** 적분 오차가 시간과 함께 누적된다(드리프트). 장거리 주행 후에는 `odom` 원점이 실제 물리 위치와 수 미터 이상 어긋날 수 있다.
- **사용처:** 단기 제어, MPPI의 `robot_speed` 및 `robot_pose` 기반, 장애물 회피

---

### 2.3 `utm` — 전역 절대 기준점

**원점:** `gnss_to_odom.py`에서 설정한 `datum_easting / datum_northing` (UTM 좌표). 이 점이 ROS의 `(0, 0)` utm 원점이 된다.

```
현재 설정 (gnss_to_odom.py):
  datum_easting  = 첫 번째 f9r GNSS 수신 시의 UTM easting
  datum_northing = 첫 번째 f9r GNSS 수신 시의 UTM northing

→ datum이 실행마다 달라지므로 utm 원점도 실행마다 달라진다.
  datum을 고정 UTM 값으로 하드코딩하면 utm 원점도 항상 일정해진다.
```

`utm` 프레임을 이해하는 핵심은 **`utm`이 `odom`을 보정하는 프레임**이라는 것이다.

```
[이해하기 어려운 이유]
직관적으로는: utm → base_link 하나면 충분하지 않나?

[실제 이유]
odom → base_link 는 연속적(제어기용)
utm → odom      는 가끔 점프해서 절대 위치 보정

둘을 분리함으로써, GNSS 보정이 제어기에 직접 전달되는 것을 차단한다.
```

**`utm → odom` TF의 물리적 의미:**

```
initial:  utm → odom = 항등변환
          (utm 원점과 odom 원점이 같은 위치)

500m 주행 후 odom이 2m 동쪽으로 드리프트했다면:
  global_ekf가 GNSS로 실제 위치를 파악
  → utm → odom 오프셋을 2m 서쪽으로 조정
  → odom → base_link 는 그대로 (제어기에 영향 없음)
  → utm → odom → base_link 합산으로 실제 절대 위치 계산
```

- **발행 주체:** `global_ekf` (`world_frame: utm`, `publish_tf: true`)
- **계산 방법:** wheel `vx` + IMU `wz` + GNSS UTM `(x,y)` + azimuth yaw의 EKF 융합
- **장점:** 드리프트가 보정된 절대 위치
- **단점:** GNSS 업데이트 시 `utm → odom` 오프셋이 불연속적으로 변할 수 있다(Jump). 단, 이 Jump는 `odom → base_link`에는 전달되지 않으므로 제어기는 영향을 받지 않는다.
- **사용처:** 전역 경로 추종, MPPI의 global plan frame

---

### 2.4 `csv_to_utm` — 경로 파일 → utm 프레임 Path 발행

`csv_to_utm` 노드(`gnss_to_utm` 패키지)는 CSV 파일의 UTM 절대 좌표를 `utm` 프레임의 상대 좌표로 변환하여 `/csv_path`(`nav_msgs/Path`)로 발행한다. 별도 TF 프레임은 발행하지 않는다.

```
변환 규칙 (gnss_to_odom.py와 동일한 datum 사용):
  local_x =  (utm_x - datum_x)
  local_y = -(utm_y - datum_y)   ← CARLA +Y=right 보정

datum 공급 경로:
  gnss_to_odom.py → /utm_datum (transient_local) → csv_to_utm
  → 두 노드가 동일한 datum을 공유하므로 /csv_path는 바로 utm 프레임 경로로 사용 가능
```

- **발행 주체:** `csv_to_utm` 노드 (`gnss_to_utm` 패키지)
- **구독:** `/utm_datum` (`geometry_msgs/PointStamped`, transient_local) — `gnss_to_odom`이 래치한 datum
- **발행:** `/csv_path` (`nav_msgs/Path`, `frame_id: utm`) — 웨이포인트 pose 목록 (yaw 포함)
- **EKF와의 관계:** TF 트리에 직접 참여하지 않지만 `/utm_datum`을 통해 dual_filter 스택과 datum을 공유한다.
- **파라미터:** `csv_file_path` — 절대 경로로 CSV 파일 지정 (`config/csv_to_utm.yaml`)
- **사용처:** `dual_filter` 스택과 함께 실행하여 `/csv_path`를 경로 추종 입력으로 활용

---

### 2.5 프레임 관계 요약

```
[TF 트리 — dual_filter 스택]

  지구상의 절대 위치 (UTM datum 기준)
       │
       ▼
     utm ──────────────────────────────── 전역 고정 좌표계
       │                                  원점: gnss_to_odom.py의 datum
       │ utm → odom TF                    발행: global_ekf
       │ (GNSS로 드리프트 보정)
       ▼
     odom ─────────────────────────────── 출발점 고정 좌표계
       │                                  원점: 시스템 시작 시 차량 위치
       │ odom → base_link TF              발행: local_ekf
       │ (wheel + IMU 연속 적분)
       ▼
   base_link ─────────────────────────── 차량 물리 프레임
                                          원점: 후륜축 중심 (차량과 함께 이동)


[경로 파일 — csv_to_utm (TF 트리 외부)]

  gnss_to_odom → /utm_datum → csv_to_utm → /csv_path (utm frame)
  (datum 공유로 /csv_path는 별도 TF 없이 utm 프레임 경로로 직접 사용 가능)
```

| 프레임 | 원점 | 이동 여부 | 연속성 | 절대 위치 | 발행 주체 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `base_link` | 후륜축 중심 | 차량과 함께 이동 | 연속 | 없음 | local_ekf |
| `odom` | 시스템 시작 위치 | 고정 | 연속 (드리프트 있음) | 없음 | local_ekf |
| `utm` | datum UTM 좌표 | 고정 | 가끔 보정(Jump 가능) | 있음 | global_ekf |

---

## 3. 센서 토픽 및 메시지 타입 명세
EKF 노드에 입력되는 토픽 목록이다. CARLA 원본 토픽명과 EKF 내부 사용 토픽명이 다를 경우 launch 파일에서 리매핑이 필요하다.

| 출처 | CARLA 원본 토픽 | EKF 입력 토픽 | 메시지 타입 | 필수 데이터 필드 |
| :--- | :--- | :--- | :--- | :--- |
| **`ros2_sensor.py`** (`vehicle.get_velocity()` CARLA 물리 엔진 ground truth) | `/carla/car/wheel_encoder/data` | `/wheel_encoder/data` | `nav_msgs/Odometry` | `twist.twist.linear.x`, `twist.twist.linear.y = 0` |
| **IMU** | `/carla/car/imu/data` | `/imu/data` (리매핑) | `sensor_msgs/Imu` | `angular_velocity.z` |
| **GNSS (후륜축)** | `/carla/car/f9r/fix` | `/f9r/fix` (리매핑) | `sensor_msgs/NavSatFix` | `latitude`, `longitude`, `altitude`, `status` |
| **GNSS (전방 1.4m)** | `/carla/car/f9p/fix` | `/f9p/fix` (리매핑) | `sensor_msgs/NavSatFix` | `latitude`, `longitude`, `altitude`, `status` |

### 3.1 EKF가 실제로 사용하는 메시지 필드

아래 표는 전체 파이프라인에서 실제로 필터 계산에 들어가는 필드만 분리한 것이다. 카메라는 RViz 확인용이며 현재 dual EKF 입력으로는 사용하지 않는다.

| 토픽 | 메시지 타입 | 사용하는 필드 | 사용 노드 | 목적 |
| :--- | :--- | :--- | :--- | :--- |
| `/clock` | `rosgraph_msgs/Clock` | `clock` | 모든 `use_sim_time:=true` 노드 | CARLA simulation time 기준으로 EKF 적분 시간 통일 |
| `/wheel_encoder/data` | `nav_msgs/Odometry` | `header.stamp` | local/global EKF | wheel 측정 시각 |
| `/wheel_encoder/data` | `nav_msgs/Odometry` | `twist.twist.linear.x` | local/global EKF | 차량 전방 속도 `vx` |
| `/wheel_encoder/data` | `nav_msgs/Odometry` | `twist.twist.linear.y = 0` | local/global EKF | 비홀로노믹 제약, 옆미끄럼 없음 `vy=0` |
| `/carla/car/imu/data` | `sensor_msgs/Imu` | `header.stamp` | local/global EKF | IMU 측정 시각 |
| `/carla/car/imu/data` | `sensor_msgs/Imu` | `angular_velocity.z` | local/global EKF | yaw rate `wz` |
| `/carla/car/f9r/fix` | `sensor_msgs/NavSatFix` | `header.stamp`, `latitude`, `longitude`, `altitude` | `f9r_to_utm`, `azimuth_angle_calculator` | 후륜축 GNSS 위치와 heading 기준점 |
| `/carla/car/f9p/fix` | `sensor_msgs/NavSatFix` | `header.stamp`, `latitude`, `longitude`, `altitude` | `f9p_to_utm`, `azimuth_angle_calculator` | 전방 GNSS 위치와 heading 벡터 끝점 |
| `/f9r_utm` | `geometry_msgs/PointStamped` | `header.stamp`, `point.x`, `point.y`, `point.z` | `gnss_to_odom` | f9r UTM 위치 |
| `/azimuth_angle` | `std_msgs/Float64` | `data` | `gnss_to_odom` | f9r→f9p geographic bearing |
| `/odometry/gnss` | `nav_msgs/Odometry` | `pose.pose.position.x`, `pose.pose.position.y`, `pose.pose.orientation` | global EKF | 절대 위치와 절대 yaw 보정 |

`robot_localization`의 `*_config` 배열 순서는 다음과 같다.

```text
[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
```

따라서 `/wheel_encoder/data`에서 `vx`, `vy`만 사용한다는 것은 7번째와 8번째 항목이 `true`라는 뜻이고, `/imu/data`에서 `angular_velocity.z`만 사용한다는 것은 `vyaw` 항목만 `true`라는 뜻이다.

---

## 4. 노드별 파라미터 및 아키텍처 설계

시스템은 총 3개의 핵심 노드로 구성된다.

### Node 1: GNSS 좌표 변환 파이프라인 (`gnss_to_utm` 패키지)

*   **목적:** WGS84(위경도) 좌표를 UTM 직교 좌표계(미터 단위)로 변환하고, 듀얼 GNSS 차분으로 차량 헤딩(Azimuth)을 계산.
*   **입력:** `/carla/car/f9r/fix`, `/carla/car/f9p/fix` (`sensor_msgs/NavSatFix`)
*   **출력:**

| 노드 | 출력 토픽 | 타입 | 내용 |
| :--- | :--- | :--- | :--- |
| `f9r_to_utm` | `/f9r_utm` | `geometry_msgs/PointStamped` | f9r의 UTM 좌표 (easting, northing) |
| `f9p_to_utm` | `/f9p_utm` | `geometry_msgs/PointStamped` | f9p의 UTM 좌표 (easting, northing) |
| `azimuth_angle_calculator` | `/azimuth_angle` | `std_msgs/Float64` | f9p − f9r 차분으로 계산된 차량 헤딩(**도°**, geographic N=0 CW+) |
| `gnss_to_odom` | `/odometry/gnss` | `nav_msgs/Odometry` | 글로벌 EKF 입력용 — f9r UTM 위치 + azimuth yaw를 단일 Odometry로 합성 |
| `gnss_to_odom` | `/utm_datum` | `geometry_msgs/PointStamped` | 최초 f9r GNSS 수신 시의 UTM easting/northing을 datum으로 래치 (QoS: transient_local) → `csv_to_utm`이 구독하여 경로 좌표 변환에 사용 |

> **토픽 리매핑:** launch 파일에서 CARLA 토픽명 → gnss_to_utm 내부 토픽명으로 리매핑.
> `/f9r/fix` ← `/carla/car/f9r/fix`, `/f9p/fix` ← `/carla/car/f9p/fix`
>
> **`gnss_to_odom` 구현:** `/f9r_utm` (PointStamped)와 `/azimuth_angle` (Float64, 도°)를 구독.
> `azimuth_angle_calculator`가 발행하는 값은 **geographic bearing (N=0, CW+, 도°)** 이므로, 먼저 **ENU yaw (E=0, CCW+, rad)** 로 변환한 뒤 CARLA의 `+Y=right` 좌표계를 ROS `+Y=left` 좌표계에 맞추기 위해 Y축과 yaw 부호를 반전한다.
>
> ```text
> yaw_enu [rad] = π/2 − bearing_deg × π/180
> yaw_ros [rad] = −yaw_enu
> x_ros = easting − datum_easting
> y_ros = −(northing − datum_northing)
> ```
>
> 변환된 `yaw_ros`를 쿼터니언(`qz = sin(yaw_ros/2)`, `qw = cos(yaw_ros/2)`)으로 변환하여 `pose.pose.orientation`에 설정.

### Node 2: 로컬 필터 (`ekf_node` - Local)

*   **목적:** 차량의 제어기(MPPI)에 넣을 **지연 없고 연속적인 short-term 오도메트리** 생성.
*   **입력:** `/wheel_encoder/data`, `/imu/data`
*   **출력:** `/odometry/local` 토픽, `odom` $\rightarrow$ `base_link` TF 발행
*   **좌표계 역할:** `odom` 프레임 안에서 `base_link`가 얼마나 부드럽게 움직였는지를 표현한다. 전역 절대 위치가 아니라, 출발 이후의 상대 이동량을 누적한 로컬 추정값이다.

#### 로컬 EKF가 실제로 사용하는 입력 성분

| 입력 토픽 | 사용하는 필드 | EKF config 항목 | 역할 |
| :--- | :--- | :--- | :--- |
| `/wheel_encoder/data` | `header.stamp` | — | wheel 속도 측정 시각. 반드시 CARLA simulation time이어야 함 |
| `/wheel_encoder/data` | `twist.twist.linear.x` | `vx` | 차량 전방 속도. 로컬 위치 적분의 주 이동량 |
| `/wheel_encoder/data` | `twist.twist.linear.y = 0` | `vy` | 차량은 옆으로 미끄러지지 않는다는 비홀로노믹 제약 |
| `/carla/car/imu/data` → `/imu/data` | `header.stamp` | — | IMU 측정 시각. wheel odom과 같은 `/clock` 기준이어야 함 |
| `/carla/car/imu/data` → `/imu/data` | `angular_velocity.z` | `vyaw` | 차량의 상대 yaw rate. 회전 적분의 유일한 각속도 입력 |
| `/carla/car/imu/data` → `/imu/data` | `linear_acceleration.x` | `ax` | 차량 종방향 선가속도. `vx` 예측을 고주기로 보강 (아래 "IMU 선가속도 성분(ax) 사용" 참고) |

`/wheel_encoder/data`의 pose, `/wheel_encoder/data.twist.twist.angular.z`, IMU orientation, IMU `linear_acceleration.y/z`는 로컬 EKF에서 사용하지 않는다. 로컬 회전량은 오직 `/imu/data.angular_velocity.z`에서 오며, 선속도는 `/wheel_encoder/data.twist.twist.linear.x`(+ `linear.y=0` 제약)를 주 입력으로, `/imu/data.linear_acceleration.x`(ax)를 예측 보강 입력으로 사용한다.

#### EKF 파라미터의 의미

로컬 EKF는 `ekf_params.yaml`에서 다음 구조로 설정된다.

```yaml
local_ekf:
  ros__parameters:
    two_d_mode: true
    world_frame: odom
    publish_tf: true

    odom0: /wheel_encoder/data
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]

    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  true,  false, false]   # vyaw + ax
```

`odom0_config`에서 `vx`, `vy`만 `true`이므로 `/wheel_encoder/data`은 위치가 아니라 속도 측정으로만 쓰인다. `imu0_config`에서 `vyaw`와 `ax`가 `true`이므로 IMU는 yaw rate 측정과 종방향 선가속도 측정으로 쓰인다. `world_frame: odom`과 `publish_tf: true` 때문에 로컬 EKF는 `/odometry/local`과 함께 `odom → base_link` TF를 발행한다.

#### IMU 선가속도 성분(ax) 사용 — 로컬/글로벌 대칭 원칙

`imu0_config`의 13번째 항목 `ax`를 `true`로 두어 IMU 종방향 선가속도(`linear_acceleration.x`)를 EKF 예측 단계에 추가한다. EKF는 이 가속도를 적분하여 `vx` 추정을 고주기(IMU 주기)로 보강한다. `ay`, `az`는 켜지 않는다 — `ay`는 `/wheel_encoder/data`의 `vy=0` 비홀로노믹 제약과 충돌하고(코너에서 원심 가속도 대 `vy=0`이 서로 싸움), `az`는 `two_d_mode: true`에서 무의미하기 때문이다.

**핵심 — 이 설정은 `local_ekf`와 `global_ekf`에 반드시 동일하게 적용한다.** [1.1절](#11-왜-로컬글로벌-두-필터로-나누는가-설계-목적)에서 정의했듯 두 필터의 **유일한 차이는 GNSS 사용 여부**여야 한다. 두 EKF는 cascade(로컬 출력을 글로벌이 이어받음)가 아니라 **병렬(parallel)** 구조로, 각자 동일한 raw `/wheel_encoder/data` + `/imu/data`를 독립적으로 융합하고 글로벌만 `/odometry/gnss`(odom1)를 추가로 보정한다.

```text
로컬  = 예측(wheel vx + IMU wz + IMU ax)
글로벌 = 예측(wheel vx + IMU wz + IMU ax) + 보정(GNSS x, y, yaw)
         └────────────┬────────────┘
              예측 모델은 두 필터가 동일해야 한다 → ax도 대칭
```

만약 `ax`를 로컬에만 켜면 두 필터의 예측 모델이 달라져 "차이는 GNSS뿐"이라는 설계 원칙이 깨진다. 따라서 `ax`를 도입할 때는 `local_ekf.imu0_config`와 `global_ekf.imu0_config`를 **함께** `vyaw + ax`로 맞춘다.

> **참고 — cascade가 아닌 이유:** 로컬 출력 `/odometry/local`은 이미 wheel+IMU가 섞인 상관된(correlated) 추정치다. 이를 글로벌에 측정값으로 넣으면 정보가 이중 계산(double counting)되어 공분산이 과신되고 필터가 뒤틀린다. `robot_localization` 표준 듀얼 EKF(REP-105)가 두 필터를 병렬로 두는 이유가 이것이다.
>
> **효과에 대한 주의:** 현재 시뮬에서는 `/wheel_encoder/data`의 `vx`가 CARLA ground-truth라 `ax` 추가의 정확도 이득은 거의 없다. `ax`는 노이즈 있는 실차 휠 엔코더로 전환할 때(고주기 예측 보강) 실익이 커진다. 로컬은 보정이 없어 가속도 적분 드리프트 위험이 있으므로, `/path/odom` 궤적으로 개선 여부를 검증할 것.

#### 로컬 EKF가 계산하는 움직임

개념적으로 로컬 EKF는 매 시간 간격 `dt`마다 다음 정보를 누적한다.

```text
전방 이동량  ≈ vx × dt
회전 변화량  ≈ wz × dt
측면 속도    = 0으로 제약
```

즉 차량은 현재 바라보는 방향으로 `vx`만큼 전진하고, IMU의 `wz`만큼 회전한다고 가정한다. 이 추정은 짧은 시간에는 매우 부드럽고 제어에 적합하지만, GNSS 보정이 없으므로 장시간 운행하면 위치와 yaw가 조금씩 드리프트한다.

#### 왜 단순 적분(dead-reckoning) 대신 EKF를 사용하는가

로컬 EKF가 하는 핵심 연산은 사실상 dead-reckoning 적분이다. 가장 단순한 형태로 표현하면 다음과 같다.

```text
naive dead-reckoning (단순 적분):
  yaw  += wz  × dt
  x    += vx × cos(yaw) × dt
  y    += vx × sin(yaw) × dt
```

EKF의 Prediction step도 내부적으로 이 적분을 수행한다. 그렇다면 왜 단순 적분 대신 EKF를 쓰는가?

##### 이유 1 — 센서 주기가 다르다 (비동기 처리)

```text
IMU:           100Hz → wz 측정값 도착
wheel encoder:  10Hz → vx 측정값 도착

타임라인:
  t=0ms:   IMU wz만 도착  (vx 없음)
  t=10ms:  IMU wz만 도착
  ...
  t=100ms: wheel vx + IMU wz 동시 도착
  t=110ms: IMU wz만 도착  (이때 vx는 무엇을 써야 하는가?)
```

단순 concatenate는 두 센서 값이 동시에 있을 때만 동작한다는 암묵적 가정이 있다. EKF는 Prediction/Correction 단계를 분리하여, 각 센서가 도착할 때마다 독립적으로 처리한다. 센서가 없는 구간에서는 마지막 추정값으로 상태 예측을 유지한다.

##### 이유 2 — vy=0 비홀로노믹 제약을 소프트하게 적용할 수 있다

차량은 옆으로 미끄러지지 않는다. 이 제약을 단순 적분에 넣으면 그냥 y 방향 적분을 생략하는 것이 된다. EKF에서는 `vy = 0`을 신뢰도가 높은 가짜 측정값(covariance=0.01)으로 매 주기 주입하여, 횡방향 드리프트가 생길 때 EKF가 이를 보정한다.

```text
단순 적분: vy를 무시 → 횡방향 오차가 조용히 누적될 수 있음
EKF:       vy=0을 측정값으로 주입 → 횡방향 상태가 능동적으로 보정됨
```

##### 이유 3 — 노이즈 가중치를 수치화한다

wheel encoder는 저속에서 양자화 노이즈가 크고, IMU 자이로는 온도·진동 drift가 있다. 단순 적분은 두 센서를 동등하게 신뢰한다. EKF는 공분산 행렬로 각 센서의 신뢰도를 수치화하여, 노이즈가 큰 측정값일수록 Kalman Gain을 줄여 덜 반영한다.

##### 요약

```text
dead-reckoning (단순 적분) ⊂ EKF의 Prediction step

EKF = dead-reckoning (Prediction)
    + 비동기 다중 센서 지원    ← 가장 실용적인 이유
    + vy=0 비홀로노믹 제약     ← 두 번째 실용적인 이유
    + 노이즈 가중치 (Kalman Gain)
    + 상태 불확실성 추적 (공분산 행렬) ← global_ekf와 연동 시 필요
```

만약 두 센서가 동일 주기이고 노이즈가 적다면 단순 적분으로도 동작할 수 있다. 실제 하드웨어에서는 IMU(100Hz)와 wheel encoder(10Hz)의 비동기 타이밍, 그리고 vy=0 제약을 올바르게 처리하기 위해 EKF를 사용한다.

#### GNSS를 로컬 EKF에 넣지 않는 이유

로컬 EKF의 가장 중요한 요구사항은 **절대 정확도보다 연속성**이다. MPPI는 현재 차량 상태를 초기 조건으로 수많은 rollout을 계산하므로, 오도메트리가 순간적으로 튀면 제어 입력도 불안정해진다.

만약 GNSS 위치 보정이 로컬 EKF에 직접 들어가면:

```text
GNSS 위치가 순간적으로 1m 튐
→ /odometry/local이 순간이동
→ odom → base_link TF가 불연속
→ MPPI rollout 초기 상태가 갑자기 바뀜
→ 조향/가감속 명령이 튀거나 발산
```

따라서 로컬 EKF는 GNSS를 쓰지 않고 wheel+IMU만 적분한다. GNSS로 누적 드리프트를 보정하는 일은 global EKF가 `utm → odom` TF를 조정하는 방식으로 담당한다.

#### 왜 `odom → base_link` TF를 발행하는가

`odom` 프레임은 "시작 위치"를 원점으로 하는 로컬 좌표계이다. 로컬 EKF는 _"출발점에서 지금 어디까지 이동했는가?"_ 를 적분으로 계산하여, 그 결과를 `odom → base_link` TF로 표현한다.

```text
시작(odom 원점) ---[wheel vx + IMU wz 적분]--> 현재 차량 위치(base_link)
```

*   이 TF는 GNSS 보정이 없으므로 **연속적이고 부드러움** → 제어기가 안정적으로 동작.
*   단점: GNSS 없이 적분만 하므로 장시간 운행 시 오차 누적(드리프트) → **글로벌 필터가 `utm → odom`으로 보정.**

#### 로컬 EKF에서 특히 조심해야 하는 오류

| 오류 | 증상 | 원인 |
| :--- | :--- | :--- |
| `/clock` 미사용 또는 stamp 불일치 | 90도 회전이 유턴처럼 과적분됨 | 속도는 simulation second 기준인데 EKF 적분 `dt`가 wall time으로 계산됨 |
| CARLA/ROS yaw 부호 불일치 | 좌회전/우회전 방향이 뒤집힘 | CARLA `+Y=right`, ROS `+Y=left` 미러링 누락 |
| `/wheel_encoder/data.angular.z`와 IMU `angular_velocity.z` 동시 사용 | 회전량이 과하게 들어감 | yaw rate를 두 센서에서 중복 융합 |
| `vy=0` 제약 미사용 | 코너에서 옆으로 미끄러지는 궤적 | 차량 비홀로노믹 특성이 EKF에 반영되지 않음 |

### Node 3: 글로벌 필터 (`ekf_node` - Global)

*   **목적:** GNSS 절대 위치로 로컬 필터의 장기 드리프트를 보정하고, 맵 상의 절대 위치를 파악.
*   **입력:** `/wheel_encoder/data`, `/imu/data`, `/odometry/gnss` (Node 1 출력 — UTM 위치 + azimuth yaw)
*   **출력:** `/odometry/global` 토픽, `utm` $\rightarrow$ `odom` TF 발행

#### 글로벌 EKF — 각 입력이 필요한 이유 (예측/보정 단계)

EKF는 **예측(Prediction)** + **보정(Correction)** 2단계로 동작한다.

| 입력 | 단계 | 역할 | 없으면? |
| :--- | :--- | :--- | :--- |
| `/wheel_encoder/data(vx, vy=0)` + `/imu/data(wz)` | 예측 | GNSS 업데이트(10Hz) 사이 구간에서 차량 이동을 물리 모델로 추정 | GNSS가 없는 구간(1/10초)마다 위치를 전혀 모름 |
| `/odometry/gnss` (UTM x, y) | 보정 | 절대 위치로 누적된 드리프트를 보정 | 예측만 하고 보정이 없으므로 로컬 EKF와 동일하게 드리프트 누적 |
| `/odometry/gnss` (azimuth yaw) | 보정 | 절대 헤딩으로 방향 드리프트를 보정 | 위치는 보정되지만 헤딩 오차가 남아, GNSS 업데이트마다 필터가 진동 |

#### `/odometry/gnss`의 yaw를 왜 global EKF에서 사용하는가

`ekf_params.yaml`의 `odom1_config`에서 `yaw` 항목을 `true`로 둔다.

```yaml
odom1: /odometry/gnss
odom1_config: [true,  true,  false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false]
```

여기서 `yaw`는 `nav_msgs/Odometry` 메시지에 `pose.pose.orientation.yaw`라는 필드가 실제로 존재한다는 뜻이 아니다. `pose.pose.orientation`은 `geometry_msgs/Quaternion`이므로 실제 필드는 `x`, `y`, `z`, `w`뿐이다. `robot_localization`은 이 quaternion을 내부에서 roll/pitch/yaw로 변환하고, 그중 Z축 회전 성분인 yaw만 사용한다.

현재 `/odometry/gnss.pose.pose.orientation`에는 `/azimuth_angle`에서 온 dual GNSS heading이 들어간다.

```text
/azimuth_angle
  geographic bearing, degree, north=0, clockwise+

→ gnss_to_odom
  yaw_enu = π/2 − bearing
  yaw_ros = −yaw_enu
  quaternion(qz, qw)

→ /odometry/gnss.pose.pose.orientation
  robot_localization이 quaternion에서 yaw 추출
```

global EKF에서 yaw를 쓰는 이유는 절대 heading을 보정하기 위해서다. `/wheel_encoder/data(vx, vy=0)`와 `/imu/data(wz)`만 있으면 global EKF도 local EKF처럼 상대 적분만 수행한다. 위치 x, y를 GNSS로 보정하더라도 yaw가 틀어져 있으면 다음 예측 단계에서 진행 방향이 잘못되어 위치 보정과 예측이 서로 싸우게 된다. 특히 코너 구간에서는 GNSS 위치 업데이트마다 경로가 흔들리거나, `utm→odom` 보정이 불안정해질 수 있다. dual GNSS yaw를 함께 넣으면 위치와 방향이 같은 절대 좌표계에서 동시에 보정된다.

#### `/azimuth_angle`을 `pose.pose.orientation` yaw 대신 직접 쓸 수 있는가

`/azimuth_angle`을 global EKF에 직접 넣을 수는 없다. 이유는 `robot_localization`이 `std_msgs/Float64` heading 토픽을 직접 입력으로 받지 않기 때문이다. `robot_localization`이 yaw pose 측정으로 받아들일 수 있는 형태는 대표적으로 다음과 같다.

| 입력 형태 | yaw 전달 방식 | 현재 구조에서의 적합성 |
| :--- | :--- | :--- |
| `nav_msgs/Odometry` | `pose.pose.orientation` quaternion | 현재 사용 중, 가장 적합 |
| `geometry_msgs/PoseWithCovarianceStamped` | `pose.pose.orientation` quaternion | 가능하지만 위치와 yaw를 별도 메시지로 나누게 됨 |
| `sensor_msgs/Imu` orientation | `orientation` quaternion | IMU orientation처럼 보이므로 dual GNSS heading 의미가 흐려질 수 있음 |
| `std_msgs/Float64` | scalar degree/radian | `robot_localization` 입력으로 직접 사용 불가 |

따라서 `/azimuth_angle`을 "대체"하려면 raw Float64를 그대로 넣는 것이 아니라, 별도 브리지 노드에서 quaternion orientation과 covariance를 가진 `Odometry` 또는 `PoseWithCovarianceStamped`로 변환해야 한다. 현재 `gnss_to_odom.py`가 이미 이 역할을 수행한다.

둘 중 무엇이 더 좋은가? 현재 구조에서는 **`/azimuth_angle`을 `/odometry/gnss.pose.pose.orientation` quaternion으로 변환해서 global EKF에 넣는 방식이 더 좋다.** 이유는 다음과 같다.

| 비교 항목 | `/azimuth_angle` 직접 사용 | `/odometry/gnss.pose.pose.orientation` 사용 |
| :--- | :--- | :--- |
| `robot_localization` 호환성 | 직접 입력 불가 | 바로 입력 가능 |
| 좌표계 변환 | EKF 밖에서 따로 처리 필요 | `gnss_to_odom.py`에서 일관 처리 |
| 단위 | degree, N=0, CW+ | quaternion, ROS yaw 기준 |
| covariance | Float64에 없음 | `pose.covariance[35]`로 yaw 신뢰도 지정 가능 |
| position과 heading 동기화 | 별도 관리 필요 | 하나의 `/odometry/gnss` 메시지로 함께 전달 |

즉 `/azimuth_angle`은 좋은 원천 데이터이고, global EKF에는 그것을 ROS 좌표계 quaternion yaw로 변환한 `/odometry/gnss.pose.pose.orientation`을 넣는 것이 정답에 가깝다.

#### 왜 `odom → base_link` 대신 `utm → odom` TF를 발행하는가

이것이 듀얼 필터 아키텍처의 핵심이다. 두 가지 이유가 있다.

**이유 1 — TF 충돌 방지:**
로컬 EKF가 이미 `odom → base_link`를 발행 중이다. ROS TF는 동일한 parent-child 쌍에 대해 두 퍼블리셔를 허용하지 않으므로, 글로벌 EKF는 다른 TF를 발행해야 한다.

**이유 2 — GNSS 도약(Jump)이 제어기에 전달되는 것을 차단 (핵심):**

만약 GNSS 보정이 `odom → base_link`에 직접 적용된다면:

```text
GNSS 노이즈로 위치가 1m 튐
→ odom 안에서 base_link 위치가 순간이동
→ MPPI: 잘못된 초기 상태에서 모든 롤아웃 계산 → 제어 입력 발산
→ 차량 발산
```

`utm → odom` 오프셋을 조정하면:

```text
GNSS 보정 발생
→ utm → odom 오프셋만 조용히 변경됨
→ odom → base_link는 전혀 변하지 않음 (로컬 EKF가 계속 부드럽게 발행 중)
→ MPPI는 아무것도 감지하지 못함 → 안정적 제어
→ 전역 경로 추종 노드만 utm → base_link를 새로 계산하여 장거리 오차 보정
```

**전체 TF 관계 요약:**

```text
utm ──[글로벌 EKF]──> odom ──[로컬 EKF]──> base_link
     (절대 위치 오프셋)       (부드러운 이동)

utm → base_link = (utm→odom) + (odom→base_link)
                   ^글로벌EKF    ^로컬EKF

전역 경로 추종: utm → base_link 사용 (절대 위치 기반)
로컬 제어기:   odom → base_link 사용 (부드러운 이동 기반)
```

---

## 5. YAML 파라미터 파일 템플릿 (`ekf_params.yaml`)
작업 디렉토리: `[your_package]/config/ekf_params.yaml`

```yaml
local_ekf:
  ros__parameters:
    use_sim_time: true              # /clock(CARLA simulation time) 사용
    frequency: 50.0
    two_d_mode: true               # 평면 주행(2D) 강제 적용
    publish_tf: true               # odom -> base_link TF 발행 활성화
    
    map_frame: utm
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom              # 기준 프레임을 odom으로 설정

    # Wheel Encoder 설정 (X축 선속도 + Y축 비홀로노믹 제약 vy=0)
    odom0: /wheel_encoder/data
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    # IMU 설정 (Z축 각속도 wz + 종방향 선가속도 ax / orientation은 dual GNSS azimuth로 대체)
    # ay/az는 vy=0 제약·2D모드와 충돌하므로 false. global_ekf와 반드시 동일하게 유지.
    # launch에서 리매핑: /imu/data <- /carla/car/imu/data
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  true,  false, false]   # vyaw + ax
    imu0_queue_size: 10
    imu0_nodelay: true
    imu0_differential: false
    imu0_relative: false

global_ekf:
  ros__parameters:
    use_sim_time: true              # /clock(CARLA simulation time) 사용
    frequency: 50.0
    two_d_mode: true
    publish_tf: true               # utm -> odom TF 발행 활성화
    
    map_frame: utm
    odom_frame: odom
    base_link_frame: base_link
    world_frame: utm               # 기준 프레임을 utm으로 설정

    odom0: /wheel_encoder/data
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    # IMU 설정 (wz + ax) — local_ekf와 동일한 예측 모델 유지 (차이는 GNSS odom1뿐)
    # launch에서 리매핑: /imu/data <- /carla/car/imu/data
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  true,  false, false]   # vyaw + ax
    imu0_queue_size: 10
    imu0_nodelay: true
    imu0_differential: false
    imu0_relative: false

    # GNSS (UTM x, y 절대 위치 + azimuth yaw) — Node 1 브리지 노드 출력
    # odom1_config: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    odom1: /odometry/gnss
    odom1_config: [true,  true,  false,   # x, y 절대 위치 사용
                   false, false, true,    # yaw (azimuth) 사용
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_queue_size: 10
    odom1_nodelay: true
    odom1_differential: false
    odom1_relative: false
```

---

## 6. 구현 결과 (Implementation)

### 6.1 워크스페이스 구조

구현 워크스페이스: `/home/hannibal/carla/mppi_ws/`

```text
mppi_ws/
├── src/
│   ├── gnss_to_utm/             ← ament_cmake C++ 패키지
│   │   ├── src/
│   │   │   ├── f9r_to_utm.cpp           ← NavSatFix → /f9r_utm (PointStamped)
│   │   │   ├── f9p_to_utm.cpp           ← NavSatFix → /f9p_utm (PointStamped)
│   │   │   ├── azimuth_angle_calculator.cpp ← dual GNSS → /azimuth_angle (Float64)
│   │   │   ├── csv_to_utm.cpp           ← /utm_datum → /csv_path (Path, utm frame)
│   │   │   └── f9r_to_csv.py            ← 오프라인 도구: rosbag → UTM CSV 변환
│   │   ├── config/csv_to_utm.yaml       ← csv_file_path 파라미터
│   │   └── launch/csv_to_utm.launch.py
│   └── dual_filter/             ← ament_python 패키지
│       ├── package.xml
│       ├── setup.py / setup.cfg
│       ├── dual_filter/
│       │   ├── gnss_to_odom.py        ← Node 1d: /f9r_utm + /azimuth_angle → /odometry/gnss + /utm_datum
│       │   ├── path_visualizer.py     ← Odometry → Path 누적 발행 (3개 인스턴스)
│       │   ├── cmd_vel_to_carla.py    ← /cmd_vel (Twist) → CARLA VehicleControl
│       │   ├── follow_path_client.py  ← IDLE/CSV_FOLLOWING/PARKING 상태 머신 — CSV 추종 ↔ 주차 모드 전환
│       │   └── mppi_speed_calc.py     ← MPPI 속도 계산 보조 노드
│       ├── config/
│       │   ├── ekf_params.yaml        ← Section 5 파라미터 (local_ekf + global_ekf)
│       │   └── nav2_carla_params.yaml ← controller_server + MPPI + local_costmap 설정
│       └── launch/
│           ├── dual_filter.launch.py  ← EKF + GNSS 파이프라인 런치
│           └── controller.launch.py   ← controller_server + lifecycle_manager 런치
├── build/
├── install/
└── log/
```

### 6.2 `gnss_to_odom` 노드

파일: `dual_filter/dual_filter/gnss_to_odom.py`

* **역할:** Node 1의 최종 브리지 — UTM 위치와 방위각을 하나의 `nav_msgs/Odometry`로 묶어 글로벌 EKF에 전달. 최초 수신 시 datum을 래치하여 `/utm_datum`으로 발행.
* **구독:**
  * `/f9r_utm` (`geometry_msgs/PointStamped`) — f9r의 UTM easting/northing
  * `/azimuth_angle` (`std_msgs/Float64`) — geographic bearing, **도°**, N=0 CW+
* **발행:**
  * `/odometry/gnss` (`nav_msgs/Odometry`) — 글로벌 EKF 입력
    * `header.frame_id = "utm"`, `child_frame_id = "base_link"`
    * `pose.pose.position.x` = `easting - datum_easting`
    * `pose.pose.position.y` = `-(northing - datum_northing)` — CARLA `+Y=right`를 ROS `+Y=left`로 미러링
    * `pose.pose.orientation` = azimuth → ENU yaw 변환 → yaw 부호 반전 후 쿼터니언
  * `/utm_datum` (`geometry_msgs/PointStamped`, transient_local) — 최초 UTM fix를 datum으로 래치 → `csv_to_utm`과 공유
* **공분산 설정 (robot_localization 가중치 제어):**

| 요소 | 인덱스 (6×6 행렬) | 설정값 | 근거 |
| :--- | :--- | :--- | :--- |
| xx (위치 x) | [0] | 0.01 m² | RTK 정확도 ~10 cm |
| yy (위치 y) | [7] | 0.01 m² | RTK 정확도 ~10 cm |
| yaw | [35] | 0.05 rad² | 듀얼 GNSS 1.4 m 기선의 heading을 쓰되, 회전 중 과신하지 않도록 완화 |
| z, roll, pitch (미사용) | [14],[21],[28] | 1e9 | 높은 값 → EKF가 해당 측정값 무시 |

### 6.3 `ros2_sensor.py` 센서 브리지 노드

파일: `ros2_sensor/ros2_sensor.py`

이 노드는 CARLA 센서 데이터를 ROS 2 토픽으로 발행하고, EKF 입력에 필요한 `/carla/car/wheel_encoder/data`과 `/clock`도 함께 만든다.

#### `/clock`

| 항목 | 내용 |
| :--- | :--- |
| 토픽 | `/clock` |
| 타입 | `rosgraph_msgs/Clock` |
| 원천 | `vehicle.get_world().get_snapshot().timestamp.elapsed_seconds` |
| 목적 | ROS 전체를 CARLA simulation time 기준으로 동작시킴 |

CARLA synchronous/passive 환경에서는 simulation time과 wall time이 다를 수 있다. 이때 속도는 simulation second 기준인데 EKF가 wall time으로 적분하면 회전과 이동량이 과적분된다. 따라서 `/clock`을 발행하고 EKF, path publisher, RViz를 `use_sim_time:=true`로 실행한다.

#### `/carla/car/wheel_encoder/data`

| 필드 | 값 | EKF 사용 여부 |
| :--- | :--- | :--- |
| `header.stamp` | CARLA simulation timestamp | 사용 |
| `header.frame_id` | `odom` | 참조 프레임 |
| `child_frame_id` | `base_link` | twist 프레임 |
| `twist.twist.linear.x` | CARLA world velocity를 차량 전방축으로 투영한 `vx` | 사용 |
| `twist.twist.linear.y` | `0.0` | 사용, 비홀로노믹 제약 |
| `twist.twist.angular.z` | 발행하지 않음 | 사용 안 함 |

`twist.covariance[0] = 0.05`로 `vx` 신뢰도를 지정하고, `twist.covariance[7] = 0.01`로 `vy=0` 제약을 비교적 강하게 준다. yaw-rate는 IMU에서만 사용하므로 `/wheel_encoder/data`의 angular 축 covariance는 크게 둔다.

#### `/carla/car/imu/data`

| 필드 | 값 | EKF 사용 여부 |
| :--- | :--- | :--- |
| `header.stamp` | CARLA IMU timestamp | 사용 |
| `angular_velocity.z` | `-imu.gyroscope.z` | 사용 |
| `orientation` | 채우지 않음, `orientation_covariance[0] = -1` | 사용 안 함 |
| `linear_acceleration.x` | CARLA acceleration을 ROS 좌표계로 변환 | 사용 (`ax`, EKF 예측 보강) |
| `linear_acceleration.y/z` | CARLA acceleration을 ROS Y-left로 변환 | 사용 안 함 (`vy=0` 제약·2D 모드와 충돌) |

CARLA는 `X=front, Y=right, Z=up`이고 ROS `base_link`는 `X=front, Y=left, Z=up`이다. 따라서 yaw-rate와 Y축 성분은 부호를 반전한다. `linear_acceleration.x`(ax)는 [4절 대칭 원칙](#imu-선가속도-성분ax-사용--로컬글로벌-대칭-원칙)에 따라 local/global EKF에 동일하게 입력된다.

#### `/carla/car/f9r/fix`, `/carla/car/f9p/fix` — GNSS 발행 주기

f9r/f9p GNSS는 `ros2_sensor/stack.json`의 각 센서 `attributes.sensor_tick`으로 발행 주기를 정한다. 현재 **`sensor_tick: 0.1` (10Hz)** 로 설정돼 있다.

| 항목 | 값 |
| :--- | :--- |
| 설정 위치 | `ros2_sensor/stack.json` → `sensor.other.gnss`(f9r, f9p)의 `attributes.sensor_tick` |
| 현재 값 | `0.1` 초 = **10Hz** |
| 적용 방식 | `ros2_sensor.py`가 `bp.set_attribute("sensor_tick", ...)`로 CARLA blueprint에 전달 |

* `sensor_tick`을 지정하지 않으면 GNSS는 매 시뮬레이션 틱마다(월드 FPS대로) 발행되어 불필요하게 빠르다. 실제 RTK 수신기 주기(보통 5~20Hz)에 맞춰 10Hz로 제한한다.
* **f9r과 f9p는 반드시 같은 `sensor_tick`을 사용한다.** `azimuth_angle_calculator`가 두 fix의 타임스탬프 차이(`max_time_diff_sec`)로 동기 여부를 판정해 heading을 계산하므로, 주기가 다르면 dual GNSS azimuth 동기화가 깨진다.
* global EKF는 이 10Hz GNSS 사이 구간(0.1초)을 wheel `vx` + IMU `wz`/`ax` 예측으로 메운다. 주기를 낮출수록 예측 의존 구간이 길어지므로, 너무 낮추면 GNSS 업데이트 순간의 위치 보정 폭(Jump 가능성)이 커진다. 10Hz는 실제 수신기 주기와 예측 안정성의 절충값이다.

#### 센서 QoS (BEST_EFFORT vs RELIABLE)

`ros2_sensor.py`는 토픽마다 DDS **Reliability** QoS를 다르게 발행한다. Reliability는 "퍼블리셔가 메시지 전달을 보장하느냐"를 정하는 정책이다.

| 정책 | 전달 보장 | 손실 메시지 | 지연/부하 | 비유 |
| :--- | :--- | :--- | :--- | :--- |
| **BEST_EFFORT** | ❌ 없음 (보내고 잊음) | 그냥 버림 | 낮음 | UDP |
| **RELIABLE** | ✅ 보장 (ACK·재전송) | 받을 때까지 재전송 | 높음 (ACK 왕복·버퍼·백프레셔) | TCP |

핵심 원리는 **"늦게 도착한 데이터는 이미 쓸모없다"** 이다. 고주기 스트림(카메라·라이다)은 다음 프레임이 곧 오므로, 누락분을 재전송받아봤자 stale이고 그 대기가 지연·백프레셔만 만든다. 따라서 이런 스트림은 최신 프레임을 흘려보내는 BEST_EFFORT가 적합하며, 이것이 ROS 2 표준 `SensorDataQoS`(BEST_EFFORT, KEEP_LAST, depth 5)이다. 반대로 경로·맵·명령처럼 빠짐없는 전달이 중요한 저대역폭 데이터는 RELIABLE을 쓴다.

**호환성 규칙 (비대칭):**

* **BEST_EFFORT 구독자**는 BEST_EFFORT·RELIABLE 퍼블리셔 **둘 다** 수신 가능(호환).
* **RELIABLE 구독자**는 BEST_EFFORT 퍼블리셔를 **수신 불가**(불일치, `No messages will be sent` 경고). → 구독 QoS는 퍼블리셔에 맞춰야 한다.

**본 프로젝트의 퍼블리셔 QoS (`ros2_sensor.py` `_start_publishers`):**

| 토픽 | QoS | 이유 |
| :--- | :--- | :--- |
| `rgb/image`, `rear_cam/image` | **BEST_EFFORT** (`_sensor_qos`) | 이미지 ≈ 921 KB/frame·20 FPS ≈ 18 MB/s. RELIABLE 시 백프레셔로 전체 렉 → 국룰대로 BEST_EFFORT |
| `lidar_2d`, `lidar_3d/point_cloud` | **RELIABLE** (`_lidar_qos`) | 소비자(Nav2 costmap `obstacle_layer`)가 RELIABLE 구독. 국룰(BEST_EFFORT)과 다르지만, 2D 라이다는 1000 pts/s로 데이터가 작아 RELIABLE 비용 ≈ 0이고 장애물 관측은 드롭 없이 받는 편이 안전 |
| `f9r/fix`, `f9p/fix` | **RELIABLE** (`_gnss_qos`) | `f9r_to_utm`/`f9p_to_utm` C++ 노드가 RELIABLE 구독 |
| `imu/data`, `wheel_encoder/data` | **RELIABLE** (`_ekf_qos`) | `robot_localization` EKF가 RELIABLE 구독 |
| `/clock` | **BEST_EFFORT** (`_clock_qos`) | 최신 시각만 유효, depth 1 |

> **RViz 설정:** `ros2_sensor.rviz`의 이미지·라이다 디스플레이는 모두 `Reliability Policy: Best Effort`로 설정돼 있다. BEST_EFFORT 구독은 위 규칙상 두 종류 퍼블리셔를 모두 받으므로, 카메라(BEST_EFFORT pub)와 라이다(RELIABLE pub)를 한 설정으로 안전하게 시각화한다. 반대로 카메라 디스플레이를 RELIABLE로 두면 QoS 불일치로 영상이 뜨지 않는다.
>
> **국룰 요약:** 고대역폭·고주기 센서 = BEST_EFFORT(`SensorDataQoS`) / 제어·경로·맵 = RELIABLE / 래치성 데이터(map, static_layer, `/utm_datum`) = RELIABLE + `transient_local`. 라이다를 RELIABLE로 둔 것은 소비자(costmap)와 통일하기 위한 의도된 예외다.

### 6.4 `dual_filter.launch.py` 토픽 리매핑 요약

파일: `dual_filter/launch/dual_filter.launch.py`

| 노드 | 내부 토픽 (코드 하드코딩) | 실제 CARLA 토픽 | 리매핑 방법 |
| :--- | :--- | :--- | :--- |
| `f9r_to_utm` | `/f9r/fix` | `/carla/car/f9r/fix` | `remappings=` |
| `f9p_to_utm` | `/f9p/fix` | `/carla/car/f9p/fix` | `remappings=` |
| `azimuth_angle_calculator` | `gnss1_topic`, `gnss2_topic` | `/carla/car/f9r/fix`, `/carla/car/f9p/fix` | `parameters=` (파라미터 오버라이드) |
| `local_ekf` | `/imu/data` | `/carla/car/imu/data` | `remappings=` |
| `global_ekf` | `/imu/data` | `/carla/car/imu/data` | `remappings=` |
| `local_ekf` | `odometry/filtered` | `/odometry/local` | `remappings=` |
| `global_ekf` | `odometry/filtered` | `/odometry/global` | `remappings=` |
| `local_ekf` | `/wheel_encoder/data` | `/carla/car/wheel_encoder/data` | `remappings=` |
| `global_ekf` | `/wheel_encoder/data` | `/carla/car/wheel_encoder/data` | `remappings=` |

> **`/carla/car/wheel_encoder/data` → `/wheel_encoder/data`**: `ros2_sensor.py`가 `/carla/car/wheel_encoder/data`로 발행하고, `local_ekf` / `global_ekf` 노드의 `remappings=`에서 `/wheel_encoder/data` → `/carla/car/wheel_encoder/data`로 연결한다. EKF yaml의 `odom0: /wheel_encoder/data`가 내부 토픽명 역할을 한다.
> 이 토픽은 **전진 선속도 + 비홀로노믹 제약 입력**으로 사용한다. `twist.twist.angular.z`는 `/imu/data.angular_velocity.z`와 중복되므로 EKF에서 사용하지 않으며, `ros2_sensor.py`에서도 yaw-rate covariance를 크게 설정해 회전 입력으로 선택되지 않게 한다.
> `header.stamp`는 ROS wall time이 아니라 CARLA simulation timestamp를 사용한다. `/imu/data`, `/odometry/gnss`도 같은 시간 기준을 사용해야 local/global EKF가 속도를 올바른 시간 간격으로 적분한다.
> 따라서 `ros2_sensor.py`는 `/clock`을 발행하고, dual filter launch의 모든 노드는 `use_sim_time:=true`로 실행한다.

### 6.5 Path 출력

`path_visualizer`는 Odometry 메시지를 누적하여 RViz용 `nav_msgs/Path`를 발행한다.

| 출력 Path | 입력 Odometry | Path frame | 의미 |
| :--- | :--- | :--- | :--- |
| `/path/odom` | `/odometry/local` | `odom` | GNSS 없이 wheel+IMU만 적분한 부드러운 odom-frame dead-reckoning 궤적 |
| `/path/gnss` | `/odometry/gnss` | `utm` | EKF를 거치지 않은 GNSS 위치와 dual GNSS yaw 기반 절대 궤적 |
| `/path/global_ekf` | `/odometry/global` | `utm` | wheel+IMU+GNSS를 융합한 global EKF 추정 궤적 |

세 Path의 이름은 의미를 분리하기 위해 명확하게 둔다.

| RViz 표시 이름 | 토픽 | 해석 |
| :--- | :--- | :--- |
| Odom Path | `/path/odom` | local EKF dead-reckoning 결과 |
| GNSS Path | `/path/gnss` | dual GNSS 기반 절대 궤적 |
| Global EKF Path | `/path/global_ekf` | global EKF 융합 결과 |

`/path/odom`은 제어 안정성 확인용이고, `/path/gnss`는 GNSS 변환 결과가 CARLA 주행 궤적과 맞는지 확인하는 전역 기준 궤적이다. `/path/global_ekf`는 global EKF가 GNSS 원천 궤적을 얼마나 부드럽게 따라가며 `utm→odom` 보정을 만드는지 확인하는 용도이다. RViz는 `use_sim_time:=true`로 실행해야 Path와 TF가 같은 시간축에서 표시된다.

---

## 7. 빌드 및 의존성 설치

dual_filter 스택을 실행하기 전에 아래 패키지를 설치하고 빌드한다.

### 7.1 `robot_localization` 설치

```bash
sudo apt install ros-humble-robot-localization
```

### 7.2 Nav2 패키지 설치

#### 방법 A — apt 설치 (권장, 빠름)

```bash
sudo apt update
sudo apt install \
  ros-humble-nav2-controller \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-core \
  ros-humble-nav2-util \
  ros-humble-nav2-msgs \
  ros-humble-nav2-bringup
```

설치 확인:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep nav2_mppi_controller
# nav2_mppi_controller
```

#### 방법 B — 소스 빌드 (OpenMP 병렬화 활성화 시)

`nav2_mppi_controller`의 OpenMP 활성화가 필요하면 소스 빌드가 필요하다.
상세 방법은 [Section 12.7](#127-nav2_mppi_controller-openmp-빌드-연산-과다-근본-해결)을 참고한다.

```bash
cd ~/carla/navigation2
source /opt/ros/humble/setup.bash

# 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 빌드 (시간이 오래 걸림, 별도 워크스페이스 권장)
colcon build --packages-select \
  nav2_msgs nav2_core nav2_util nav2_costmap_2d \
  nav2_controller nav2_mppi_controller \
  --symlink-install

source install/setup.bash
```

### 7.3 `dual_filter` / `gnss_to_utm` 패키지 빌드

```bash
cd ~/carla/mppi_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_filter gnss_to_utm --symlink-install
source install/setup.bash
```

> **`No executable found` 오류 시**: `setup.py`에 entry point를 추가한 뒤 빌드하지 않으면
> 발생한다. 위 명령으로 재빌드하면 `cmd_vel_to_carla`, `follow_path_client`가 등록된다.

---

## 8. Nav2 MPPI Controller 연동 분석

> **참고**: 이 섹션은 구현 전 요구사항 분석 결과다. 구현 완료 후 실제 상태는 [Section 10.1](#101-구현-완료-현황)을 참고한다.

분석 대상:

```text
/home/hannibal/carla/navigation2/nav2_mppi_controller
/home/hannibal/carla/navigation2/nav2_controller
/home/hannibal/carla/navigation2/nav2_util
```

결론부터 말하면, `nav2_mppi_controller`는 단독으로 센서 토픽을 직접 구독하는 노드가 아니라 **Nav2 controller server 안에서 실행되는 controller plugin**이다. 따라서 MPPI에 필요한 입력은 controller server가 준비해서 `MPPIController::computeVelocityCommands()`에 넘겨준다.

```text
controller_server
  ├─ TF/costmap에서 robot_pose 계산
  ├─ odom_topic → OdomSmoother → robot_speed 계산
  ├─ FollowPath action의 global path → path handler → transformed_global_plan 생성
  ├─ goal checker / progress checker 관리
  └─ MPPIController::computeVelocityCommands(
       robot_pose,
       robot_speed,
       goal_checker,
       transformed_global_plan,
       global_goal)
```

즉 MPPI가 필요로 하는 것은 단순히 차량 odometry 하나가 아니라, **pose, speed, path, goal, TF, costmap, motion model, critic 설정, cmd_vel 소비자**까지 포함한 전체 Nav2 제어 환경이다.

### 8.1 MPPI Controller의 필수 입력/조건 요약

| 요구사항 | 코드상 형태 | 실제 공급 주체 | 현재 파이프라인 충족 여부 | 권장 설정/조치 |
| :--- | :--- | :--- | :--- | :--- |
| 현재 차량 pose | `geometry_msgs/PoseStamped robot_pose` | `controller_server`가 TF/costmap으로 계산 | 충족 가능. `utm→odom→base_link` TF가 있음 | Nav2 frame을 `global_frame: utm`, `robot_base_frame: base_link` 기준으로 맞춤 |
| 현재 차량 속도 | `geometry_msgs/Twist robot_speed` | `odom_topic`의 `nav_msgs/Odometry.twist.twist` | 충족. `/odometry/local`이 가장 적합 | `controller_server.odom_topic: /odometry/local` |
| 로컬 제어용 path | `nav_msgs/Path transformed_global_plan` | `FollowPath` action의 path를 path handler가 변환 | dual filter만으로는 미충족 | Nav2 planner 또는 외부 global path publisher가 FollowPath action에 path 제공 필요 |
| 최종 goal | `geometry_msgs/PoseStamped global_goal` | FollowPath action / path handler | dual filter만으로는 미충족 | 목표 pose 또는 path 마지막 pose 제공 필요 |
| goal checker | `nav2_core::GoalChecker *` | `controller_server` plugin | Nav2 설정 필요 | `SimpleGoalChecker`, `StoppedGoalChecker` 등 설정 |
| progress checker | progress checker plugin | `controller_server` plugin | Nav2 설정 필요 | 주행 중 진행 여부 판단용 plugin 설정 |
| local costmap | `nav2_costmap_2d::Costmap2DROS` | Nav2 local costmap | 별도 설정 필요 | LiDAR/obstacle layer/inflation layer 구성 필요 |
| TF buffer | `tf2_ros::Buffer` | Nav2 stack | 충족 가능 | 모든 노드 `use_sim_time:=true`, TF tree 유지 |
| motion model | `motion_model` parameter | MPPI plugin | 설정으로 충족 | 차량형이면 `ackermann` 권장 |
| velocity/acceleration limits | `vx_max`, `vx_min`, `wz_max`, `ax_max`, `az_max` 등 | MPPI parameter | 설정 필요 | CARLA 차량 동역학에 맞게 튜닝 |
| critics | `critics` parameter | MPPI critic plugins | 설정 필요 | path/goal/obstacle 관련 critic 선택 |
| trajectory validator | `TrajectoryValidator.plugin` | MPPI validator plugin | 기본값 사용 가능 | 기본 `mppi::DefaultOptimalTrajectoryValidator` 사용 가능 |
| command output | `geometry_msgs/TwistStamped` → `cmd_vel` | controller server publisher | 소비자 별도 필요 | `/cmd_vel`을 CARLA throttle/steer/brake로 변환하는 노드 필요 |

### 8.2 MPPI 내부에서 실제로 쓰는 데이터

`MPPIController::computeVelocityCommands()`가 받는 입력은 다음 5개이다.

| 함수 입력 | 의미 | MPPI 내부 사용 |
| :--- | :--- | :--- |
| `robot_pose` | 현재 차량 위치와 yaw | rollout 시작 pose. trajectory 적분의 초기 `x`, `y`, `yaw` |
| `robot_speed` | 현재 차량 속도 | rollout 시작 속도. `state.speed.linear.x`, `state.speed.angular.z`에 들어감 |
| `goal_checker` | 목표 도달 판정 | goal 관련 critic/validator에서 사용 |
| `transformed_global_plan` | 로컬 프레임으로 변환된 path | path follow/align/angle critic이 평가 |
| `global_goal` | 최종 목표 pose | goal/goal_angle critic이 평가 |

Optimizer 내부에서는 다음처럼 현재 속도와 pose를 초기 상태로 사용한다.

```text
state.pose = robot_pose
state.speed = robot_speed
state.vx.col(0) = state.speed.linear.x
state.wz.col(0) = state.speed.angular.z
```

그리고 출력은 다음 형태로 나온다.

```text
geometry_msgs/TwistStamped cmd_vel
  header.frame_id = base_link
  twist.linear.x  = 선택된 vx
  twist.angular.z = 선택된 wz
```

holonomic model일 때만 `twist.linear.y`도 출력한다. Ackermann/DiffDrive model에서는 `linear.x`, `angular.z` 중심이다.

### 8.3 `odom_topic`에는 무엇을 넣어야 하는가

`controller_server`는 `odom_topic` 파라미터를 읽어 `nav2_util::OdomSmoother`를 만들고, 매 control loop에서 `getRawTwist()`로 최신 twist를 가져온다.

```text
odom_topic
  → nav_msgs/Odometry
  → twist.twist
  → getRawTwist()
  → robot_speed
  → MPPIController::computeVelocityCommands()
```

따라서 `odom_topic`의 핵심은 `pose.pose`가 아니라 `twist.twist`이다.

| 후보 topic | `odom_topic` 사용 가능? | 장점 | 문제점 | 판단 |
| :--- | :--- | :--- | :--- | :--- |
| `/odometry/local` | 가능 | wheel `vx`, `vy=0`, IMU `wz`가 융합된 연속적 twist. GNSS jump 없음 | 장기 위치 drift는 있지만 MPPI의 현재 속도 입력에는 큰 문제 없음 | **권장** |
| `/wheel_encoder/data` | 부분 가능 | 전방 속도 `linear.x`가 직접적이고 지연이 작음 | yaw rate를 쓰지 않도록 만든 topic이라 `angular.z`가 부정확하거나 0이 될 수 있음 | 비권장 |
| `/odometry/global` | 가능은 함 | global EKF 융합 결과 | GNSS 보정 영향이 섞이며 제어용 현재 속도에는 불필요 | 비권장 |
| `/odometry/gnss` | 부적합 | 절대 위치와 dual GNSS yaw가 있음 | `gnss_to_odom`는 pose 브리지이며 twist를 제공하지 않음 | 사용 금지 |

권장 설정:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    odom_topic: /odometry/local
    odom_duration: 0.3
```

`/odometry/local`은 `/wheel_encoder/data.twist.twist.linear.x`, `linear.y=0`, `/imu/data.angular_velocity.z`를 EKF로 융합하므로 MPPI의 `robot_speed` 입력으로 가장 안정적이다. 또한 `/odometry/local`은 GNSS 보정을 받지 않으므로 제어 루프에 GNSS jump를 전달하지 않는다.

### 8.4 Pose와 TF 요구사항

MPPI의 `robot_pose`는 odometry topic의 pose가 아니라 controller server/costmap/TF 경로에서 들어온다. 따라서 다음 TF가 반드시 살아 있어야 한다.

```text
utm ──> odom ──> base_link
```

| TF | 발행 주체 | MPPI 관점에서 필요한 이유 | 현재 충족 여부 |
| :--- | :--- | :--- | :--- |
| `odom → base_link` | `local_ekf` | local frame에서 차량 pose를 연속적으로 제공 | 충족 |
| `utm → odom` | `global_ekf` | global path/utm frame과 local odom frame 연결 | 충족 |
| `utm → base_link` | TF 합성 결과 | global plan을 local control frame으로 변환할 때 필요 | 위 두 TF가 있으면 충족 |

Nav2 costmap frame 설정은 보통 다음 구성이 자연스럽다.

| Nav2 frame parameter | 권장값 | 이유 |
| :--- | :--- | :--- |
| `global_frame` | `utm` 또는 local costmap에서는 `odom` | global planner/path와 local controller 구성에 따라 선택 |
| `robot_base_frame` | `base_link` | CARLA 차량 기준 프레임 |
| `transform_tolerance` | `0.1` 이상에서 시작 | sim time/TF 지연을 흡수 |

중요한 점은 `robot_pose`와 `transformed_global_plan`이 같은 local control 기준에서 일관되게 계산되어야 한다는 것이다.

### 8.5 Path와 Goal 요구사항

MPPI는 path를 직접 만들지 않는다. `controller_server`가 FollowPath action으로 받은 `nav_msgs/Path`를 path handler로 자르고 변환한 뒤 MPPI에 넘긴다.

| 요구사항 | 필요 메시지/객체 | 현재 dual filter가 제공? | 추가 필요 |
| :--- | :--- | :--- | :--- |
| 추종할 global path | `nav_msgs/Path` | 아니오. `/path/gnss`, `/path/global_ekf`는 시각화용 주행 궤적임 | planner 또는 별도 path publisher |
| path frame | 보통 `utm` | 가능 | path header frame과 TF tree 일치 필요 |
| 최종 goal | path 마지막 pose 또는 action goal | 아니오 | FollowPath action goal 제공 |
| transformed local plan | controller server 내부 생성 | Nav2가 생성 | TF와 path가 정상이어야 함 |

주의: `/path/odom`, `/path/gnss`, `/path/global_ekf`는 RViz에서 실제 주행 궤적을 확인하기 위한 출력이다. 이들은 “따라가야 할 계획 경로”가 아니라 “이미 지나온 경로 기록”이므로 MPPI의 global plan으로 넣으면 안 된다.

### 8.6 Costmap과 장애물 정보

MPPI는 critics를 통해 trajectory cost를 계산하고, obstacle 관련 critic은 local costmap을 사용한다. 따라서 장애물 회피까지 하려면 local costmap이 필요하다.

| 요구사항 | 현재 파이프라인 충족 여부 | 추가 필요 |
| :--- | :--- | :--- |
| local costmap | 별도 설정 필요 | Nav2 local_costmap 구성 |
| obstacle layer 입력 | 가능성 있음 | `/carla/car/lidar_2d/point_cloud`를 costmap observation source로 연결 |
| inflation layer | 별도 설정 필요 | 차량 footprint와 inflation radius 튜닝 |
| robot footprint | 별도 설정 필요 | CARLA 차량 크기에 맞는 footprint 설정 |

costmap 없이도 목표 추종만 실험할 수는 있지만, `CostCritic`, `ObstaclesCritic`을 제대로 쓰려면 costmap 구성이 필수다.

### 8.7 Motion Model과 차량 제약

`nav2_mppi_controller`는 세 가지 motion model plugin을 제공한다.

| motion model | plugin | 특성 | CARLA 차량에 대한 판단 |
| :--- | :--- | :--- | :--- |
| `diff_drive` | `mppi::DiffDriveMotionModel` | `vx`, `wz` 사용. 회전반경 제약 없음 | 임시 실험 가능 |
| `ackermann` | `mppi::AckermannMotionModel` | `vx`, `wz` 사용. `min_turning_r`로 회전반경 제한 | **권장** |
| `omni` | `mppi::OmniMotionModel` | `vx`, `vy`, `wz` 사용 | 일반 차량에는 부적합 |

CARLA 일반 차량은 lateral velocity를 독립적으로 명령할 수 없으므로 `omni`는 맞지 않는다. 차량형 플랫폼에서는 `ackermann`을 쓰고, 실제 최소 회전반경에 맞춰 `min_turning_r`를 조정하는 것이 좋다.

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Ackermann"
  ackermann:
    plugin: "mppi::AckermannMotionModel"
    min_turning_r: 3.3
```

> **주의:** Nav2는 `motion_model` 값에 대소문자를 구분한다. `"ackermann"` (소문자)으로 설정하면 `Model ackermann is not valid!` 오류가 발생한다. 반드시 `"Ackermann"` (첫 글자 대문자)으로 설정해야 한다.

### 8.8 MPPI 주요 파라미터

→ 파라미터 상세 설명 및 튜닝 가이드: [Section 13](#13-파라미터-상세-설명-및-튜닝-가이드)

### 8.9 출력 명령과 CARLA 제어 변환

MPPI의 최종 출력은 `cmd_vel`이다.

```text
MPPI output
  geometry_msgs/TwistStamped or Twist
  linear.x  = 목표 전후진 속도
  angular.z = 목표 yaw rate
```

하지만 CARLA 차량 제어 입력은 일반적으로 `throttle`, `brake`, `steer`이다. 따라서 다음 변환 노드가 별도로 필요하다.

| MPPI 출력 | CARLA 제어로 변환 | 필요 여부 |
| :--- | :--- | :--- |
| `cmd_vel.linear.x` | 목표 속도 → throttle/brake PID | 필요 |
| `cmd_vel.angular.z` | 목표 yaw rate 또는 curvature → steer | 필요 |
| `cmd_vel.linear.y` | Ackermann/DiffDrive에서는 사용 안 함 | 불필요 |

즉 localization 파이프라인이 `/odometry/local`을 제공하더라도, 실제 CARLA 차량을 움직이려면 `cmd_vel`을 CARLA `VehicleControl`로 바꾸는 low-level controller가 있어야 한다.

### 8.10 현재 시스템 기준 충족/미충족 최종 표

> **참고:** 이 표는 구현 이전의 요구사항 분석 결과다. 구현 완료 후 실제 상태는 [Section 10.1](#101-현재-상태-요약)을 참고한다.

| 항목 | 필요 여부 | 현재 제공 topic/구성 | 상태 | 다음 작업 |
| :--- | :--- | :--- | :--- | :--- |
| 현재 속도 odometry | 필수 | `/odometry/local` | 충족 | `controller_server.odom_topic`에 지정 |
| 연속 TF | 필수 | `odom→base_link` | 충족 | local EKF 유지 |
| 전역 보정 TF | 필수에 가까움 | `utm→odom` | 충족 | global EKF 유지 |
| 현재 pose | 필수 | TF 합성 `utm/odom→base_link` | 충족 가능 | Nav2 frame 설정 필요 |
| global path | 필수 | 없음. `/path/*`는 시각화용 | 미충족 | planner 또는 FollowPath용 path 생성 |
| goal pose | 필수 | 없음 | 미충족 | Nav2 action goal 제공 |
| local costmap | 장애물 회피 시 필수 | LiDAR topic은 있음 | 부분 충족 | Nav2 costmap 설정 필요 |
| motion model | 필수 | MPPI plugin 제공 | 설정 필요 | `ackermann` 권장 |
| vehicle constraints | 필수 | 파라미터로 제공 | 설정 필요 | 속도/가속/회전반경 튜닝 |
| cmd_vel 소비자 | 실제 주행에 필수 | 없음 | 미충족 | `cmd_vel` → CARLA control 노드 필요 |
| sim time | 필수 | `/clock` | 충족 | Nav2 전체 `use_sim_time:=true` |

최소 실행 관점에서 MPPI controller에 먼저 연결해야 하는 것은 다음 순서다.

```text
1. controller_server.odom_topic = /odometry/local
2. Nav2 TF frame: utm/odom/base_link 일치
3. FollowPath에 넣을 계획 경로 생성
4. local costmap 구성
5. motion_model = ackermann 및 제약 튜닝
6. cmd_vel을 CARLA VehicleControl로 변환
```

---

## 9. 실제 하드웨어 휠 오도메트리 구현

CARLA 시뮬레이션에서는 `ros2_sensor.py`가 `/carla/car/wheel_encoder/data`를 발행하고 launch 파일 리매핑으로 EKF의 `/wheel_encoder/data`에 연결한다(Section 6.3). 실제 하드웨어에서는 `serial_bridge` 노드가 아두이노 시리얼에서 전륜 엔코더(VX)와 조향각(PS)을 파싱하여 자전거 모델 보정을 적용한 뒤 직접 `/wheel_encoder/data`로 발행한다.

### 9.1 전륜 엔코더와 조향각 보정의 필요성

차량의 구동 모터와 조향 모터는 독립적이지만, 전륜 엔코더가 측정하는 속도는 **조향각의 영향을 받는다**.

```text
[자전거 모델 기준 차량 운동학]

후륜(차량 중심축 기준):
  v_rear  = 차량의 실제 전진 속도

전륜(조향된 상태):
  전륜은 조향각 δ만큼 꺾여있으므로, 전진 방향으로 투영되는 속도는
  v_encoder = v_rear / cos(δ)

  → 조향각이 클수록 엔코더가 더 큰 속도를 측정함
  → δ = 0°일 때 : v_encoder = v_rear        (직진, 동일)
  → δ = 30°일 때: v_encoder = v_rear / 0.866 = v_rear × 1.155 (15.5% 과대 측정)
```

따라서 EKF에 입력할 실제 후륜축 속도를 구하려면:

```text
v_rear = v_encoder × cos(δ)
```

이 보정을 위해 실제 조향각 δ가 필요하다. MPPI가 출력하는 **조향 명령(SA)** 을 쓰면 순환 의존성(Ouroboros)이 생기지만, **POT(가변저항)가 측정하는 실제 조향각(PS)** 은 MPPI와 독립적이므로 이 문제가 없다.

### 9.2 아두이노 시리얼 프로토콜

파일: `/home/hannibal/carla/final.ino`

시리얼 통신 방향에 따라 키워드가 구분된다.

| 방향 | 키워드 | 형식 | 의미 |
| :--- | :--- | :--- | :--- |
| ROS2 → 아두이노 | `TH` | `TH <float>\n` | 쓰로틀 명령 (−1.0 ~ 1.0) |
| ROS2 → 아두이노 | `SA` | `SA <float>\n` | 조향 명령, 도° (Steering Angle command) |
| 아두이노 → ROS2 | `VX` | `\| VX:<float>` | 전륜 엔코더 선속도, m/s (보정 전 raw) |
| 아두이노 → ROS2 | `PS` | `\| PS:<float>` | POT 측정 실제 조향각, 도° (POT Steering) |

아두이노 시리얼 출력 한 줄 예시 (100ms 주기):

```text
TH:0.000 | SA:0.00 | Enc:1234 | VX:0.5123 | PS:-8.45
```

#### 아두이노 측 핵심 상수 및 계산

```cpp
#define ENCODER_PPR     360    // 엔코더 1회전당 펄스 수 (실제 하드웨어 값으로 수정 필요)
#define WHEEL_RADIUS_M  0.135f // 타이어 반지름, m (실측값)

// 100ms 주기 시리얼 블록에서:
float dt_odom_s = (float)(now_ms - lp) / 1000.0f;   // 이전 출력 이후 경과 시간 (s)
long  d_encoder = encoder_count - prev_encoder;       // 경과 시간 동안의 펄스 변화량
float v_wheel_ms = (float)d_encoder
                   * (2.0f * PI * WHEEL_RADIUS_M)
                   / ((float)ENCODER_PPR * dt_odom_s); // 엔코더 선속도 (m/s)

Serial.print(" | VX:"); Serial.print(v_wheel_ms, 4);
Serial.print(" | PS:"); Serial.println(raw_deg, 2);   // raw_deg: 데드밴드 미적용 조향각
```

`raw_deg`(데드밴드 미적용)을 PS로 내보내는 이유: 소각도에서도 `cos(δ)` 보정이 필요하므로, 데드밴드로 0이 된 `deg` 대신 실측값 그대로를 사용한다.

#### 쿼드러처 엔코더 채널 설명

`ENCODER_A`와 `ENCODER_B`는 하나의 엔코더 센서의 두 채널이다. A채널이 인터럽트를 발생시키고, B채널의 상태로 회전 방향을 판별한다.

```cpp
void encoderISR() {
    if (digitalRead(ENCODER_B) == HIGH) encoder_count++;  // 전진
    else                                 encoder_count--;  // 후진
}
```

### 9.3 시리얼 브리지 (`serial_bridge`) 구현

파일: `mppi_ws/src/serial_bridge/serial_bridge/serial_bridge.py`

패키지: `mppi_ws/src/serial_bridge/`

#### 역할

| 방향 | 동작 |
| :--- | :--- |
| `/auto_throttle` → 아두이노 | Float32 수신 → `TH <val>\n` 시리얼 전송 |
| `/auto_steer_angle` → 아두이노 | Float32 수신 → `SA <val>\n` 시리얼 전송 |
| 아두이노 → `/wheel_encoder/data` | 시리얼 수신 → VX/PS 파싱 → 보정 → Odometry 발행 |

#### 핵심 처리 흐름

```text
아두이노 시리얼 한 줄 수신
  "... | VX:0.5123 | PS:-8.45"
         ↓ _parse_field()
  vx_raw = 0.5123 m/s
  ps_deg = -8.45°
         ↓ _publish_wheel_odom()
  v_rear = 0.5123 × cos(−8.45° × π/180)
         = 0.5123 × 0.9892
         = 0.5068 m/s
         ↓
  /wheel_encoder/data.twist.twist.linear.x = 0.5068
  /wheel_encoder/data.twist.twist.linear.y = 0.0   (비홀로노믹 제약)
         ↓
  local_ekf / global_ekf odom0 입력
```

#### 발행 메시지 상세

```python
msg = Odometry()
msg.header.frame_id = 'odom'
msg.child_frame_id  = 'base_link'
msg.twist.twist.linear.x  = v_rear   # 후륜축 전방 속도 (m/s)
msg.twist.twist.linear.y  = 0.0      # 비홀로노믹 제약
msg.twist.twist.angular.z = 0.0      # yaw rate는 IMU에서 별도 공급
```

공분산 설정 (Section 6.3의 CARLA 구성과 동일):

| 인덱스 (6×6 행렬) | 값 | 의미 |
| :--- | :--- | :--- |
| `[0]` (vx 분산) | `0.05` | 엔코더+보정 기준 vx 신뢰도 |
| `[7]` (vy 분산) | `0.01` | vy=0 비홀로노믹 제약, 강하게 신뢰 |
| `[35]` (wz 분산) | `1e6` | yaw rate는 이 토픽에서 사용 안 함 |

#### 시리얼 포트 파라미터

```yaml
serial_bridge:
  ros__parameters:
    port: /dev/arduino_bridge   # udev 심볼릭 링크 또는 /dev/ttyUSB0
    baud: 57600
    throttle_topic: /auto_throttle
    steer_cmd_topic: /auto_steer_angle
    startup_silence_sec: 3.0    # 시작 직후 아두이노 초기화 동안 송신 차단
```

### 9.4 CARLA 시뮬레이션과 실제 하드웨어의 `/wheel_encoder/data` 비교

| 항목 | CARLA 시뮬레이션 (ros2_sensor.py) | 실제 하드웨어 (serial_bridge) |
| :--- | :--- | :--- |
| vx 원천 | CARLA world velocity → base_link 투영 | 전륜 엔코더 펄스 → 선속도 변환 |
| 조향 보정 | 불필요 (CARLA가 직접 후륜축 기준 속도 제공) | 필요: v_rear = v_encoder × cos(δ) |
| 조향각 원천 | 없음 | POT 측정 실측값 (raw_deg, PS) |
| 타임스탬프 | CARLA simulation time (/clock) | ROS2 wall time (get_clock().now()) |
| 발행 노드 | `ros2_sensor.py` | `serial_bridge` |
| 패키지 위치 | `ros2_sensor/` | `mppi_ws/src/serial_bridge/` |

> **주의:** 실제 하드웨어 실행 시 `use_sim_time: false`로 설정해야 한다. CARLA 시뮬레이션에서만 `use_sim_time: true`를 사용한다. 두 환경을 혼합하면 EKF 적분 시간 오류가 발생한다.

### 9.5 패키지 의존성

`mppi_ws/src/serial_bridge/package.xml`:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>nav_msgs</depend>   <!-- Odometry 메시지 타입 -->
```

빌드:

```bash
cd ~/carla/mppi_ws
colcon build --packages-select serial_bridge --symlink-install
source install/setup.bash
```

실행:

```bash
ros2 run serial_bridge serial_bridge
```

### 9.6 ENCODER_PPR 검증

`final.ino`의 `ENCODER_PPR` 값은 실제 하드웨어 엔코더 데이터시트 값으로 반드시 확인해야 한다.

```cpp
#define ENCODER_PPR 360  // ← 엔코더 1회전당 실제 펄스 수로 수정 필요
```

검증 방법: 차량 바퀴를 정확히 1바퀴 수동 회전시키면서 `encoder_count` 변화량을 시리얼 모니터로 확인한다. 이 값이 `ENCODER_PPR`과 일치해야 한다.

---

## 10. 구현 컴포넌트 상세

### 10.1 구현 완료 현황

아래 분석은 `/home/hannibal/carla/mppi_ws/` 워크스페이스의 빌드 결과물과 소스 코드를 실제로 비교하여 작성한 것이다.

#### 충족 항목 (dual_filter 스택)

| 항목 | 상태 | 비고 |
| :--- | :---: | :--- |
| `/clock` 발행 | ✅ | `ros2_sensor.py`: CARLA simulation time |
| `/wheel_encoder/data` 발행 | ✅ | `ros2_sensor.py`: vx, vy=0, sim time stamp, covariance 설정 |
| `/carla/car/imu/data` 발행 | ✅ | `ros2_sensor.py`: `angular_velocity.z` CARLA→ROS 부호 반전 |
| `/carla/car/f9r/fix`, `/f9p/fix` 발행 | ✅ | `ros2_sensor.py` |
| `gnss_to_utm` 소스 구현 | ✅ | `f9r_to_utm`, `f9p_to_utm`, `azimuth_angle_calculator`, `csv_to_utm` |
| `dual_filter` 소스 구현 | ✅ | `gnss_to_odom.py`, `path_visualizer.py`, CARLA Y축 부호 반전 |
| `ekf_params.yaml` 설정 | ✅ | local: `world_frame=odom`, global: `world_frame=utm`, covariance 완성 |
| `dual_filter.launch.py` 리매핑 | ✅ | CARLA 토픽명 → EKF 내부 토픽명 전체 설정 |
| TF tree 설계 | ✅ | `utm → odom → base_link` (REP-105 준수) |
| `robot_localization` 설치 | ✅ | `ros-humble-robot-localization 3.5.4` |


#### MPPI 구성 항목

| 항목 | 상태 | 설명 |
| :--- | :---: | :--- |
| Nav2 패키지 설치 | ✅ | `nav2_controller`, `nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_lifecycle_manager` 1.1.20 apt 설치 완료 |
| controller_server 설정 파일 | ✅ | `mppi_ws/src/dual_filter/config/nav2_carla_params.yaml` 작성 완료. controller_server 3계층(제어루프·MPPI플러그인·local_costmap) + 7개 critic 설정. 차량별 튜닝 필수값: `min_turning_r`, `vx_max` |
| `cmd_vel` → CARLA 제어 변환 | ✅ | `mppi_ws/src/dual_filter/dual_filter/cmd_vel_to_carla.py` 작성 완료. 자전거 모델 역변환(δ = atan2(-wz·L, vx), CARLA steer 부호 반전 적용) + P 속도 제어 → CARLA `VehicleControl`. microlino 기본 wheelbase 1.47 m, max_steer는 physics_control에서 자동 조회 |
| global path 공급 + 모드 전환 | ✅ | `mppi_ws/src/dual_filter/dual_filter/follow_path_client.py` 작성 완료. IDLE/CSV_FOLLOWING/PARKING 상태 머신. CSV 경로 추종 + RViz 2D Goal Pose 기반 주차 모드 전환 (ComputePathToPose → FollowPath) + 주차 완료 후 자동 CSV 복귀 |
| local costmap 설정 | ✅ | `nav2_carla_params.yaml` 안에 포함. obstacle_layer(`/carla/car/lidar_2d/point_cloud`) + inflation_layer(1.5 m) + `CostCritic` 구성 완료. lidar_2d → obstacle_layer → inflation_layer → CostCritic 파이프라인 활성화. costmap은 `/local_costmap/costmap`으로 10 Hz 발행 중 |

---

### 10.2 `nav2_carla_params.yaml` 상세

파일 위치: `mppi_ws/src/dual_filter/config/nav2_carla_params.yaml`

파일은 `controller_server` → `MPPI 플러그인(FollowPath)` → `local_costmap` 의 3계층으로 구성된다.

#### controller_server 계층

| 파라미터 | 값 | 설명 |
| :--- | :--- | :--- |
| `controller_frequency` | 10.0 Hz | local_ekf(50 Hz)보다 낮게. `model_dt = 1/10 = 0.1 s`와 반드시 일치시킬 것 |
| `odom_topic` | `/odometry/local` | local EKF 출력(GNSS 미포함) 사용. GNSS 오차 도약에 면역 |
| `costmap_update_timeout` | 0.30 s | sim time TF 지연 흡수 |
| `failure_tolerance` | 1.5 s | 유효 cmd_vel 미생성 허용 시간 |
| `progress_checker` | `SimpleProgressChecker` | 10 s 동안 0.5 m 이상 이동 없으면 stuck 판정 |
| `goal_checker` | `SimpleGoalChecker` | CSV 추종 전용 — 목표 0.5 m / 0.3 rad(≈17°) 이내 도달 시 성공 |
| `parking_goal_checker` | `SimpleGoalChecker` | 주차 전용 — 0.25 m / 0.1 rad(≈6°) 엄격한 허용값. 화살표 직선 위 정렬 강제 |
| `PathHandler` | `FeasiblePathHandler` | 이미 지나친 waypoint prune_distance 5.0 m 이상이면 제거 |

> MPPI 플러그인·Critics·local_costmap 파라미터 상세: [Section 13](#13-파라미터-상세-설명-및-튜닝-가이드)

---

### 10.3 `cmd_vel_to_carla.py` 상세

파일 위치: `mppi_ws/src/dual_filter/dual_filter/cmd_vel_to_carla.py`

MPPI가 출력하는 `/cmd_vel` (`geometry_msgs/Twist`)을 CARLA `VehicleControl`로 변환하는 노드.

#### 변환 로직

##### 속도 → throttle / brake (P 제어)

전진/후진 방향에 따라 분기한다. `ctrl.reverse` 설정이 핵심으로, 이를 누락하면 CARLA가 전진 기어 상태를 유지해 후진 명령이 완전히 무시된다.

```text
is_reverse = (target_vx < 0)

[전진 모드] ctrl.reverse = False
  err = target_vx − current_vx
  err > 0 → throttle = min(KP × err, 1),  brake = 0
  err ≤ 0 → throttle = 0,  brake = min(−KP × err, 1)

[후진 모드] ctrl.reverse = True
  current_vx > 0.1 → 아직 전진 중: throttle=0, brake=1  (완전 제동 후 기어 전환)
  else:
    err = target_vx − current_vx   (둘 다 음수)
    err < 0 → throttle = min(−KP × err, 1),  brake = 0   (후진 가속)
    err ≥ 0 → throttle = 0,  brake = min(KP × err, 1)    (후진 감속)
```

`KP_SPEED = 0.8` 기본값. 오버슈트 발생 시 0.5로 낮춘다.

> **후진 P 제어 부호 이유**: `target_vx = −1.0`, `current_vx = 0.0` 이면 `err = −1.0` (음수).
> 이 시점은 "아직 목표 후진 속도에 미달" 상태이므로 스로틀을 밟아야 한다.
> 전진 P 제어와 반대로 `err < 0 → throttle` 이 올바른 방향이다.

##### yaw rate → 조향각 (자전거 모델 역변환)

전진과 후진에서 `atan2`의 4사분면 처리가 달라야 한다.

```text
[전진] vx > 0:
  δ = atan2(−wz × L, vx)        → 1·4사분면, 부호 정상
  steer = clip(δ / max_steer_rad, −1, 1)

[후진] vx < 0:
  atan2(−wz × L, vx) 에서 vx < 0 이면 atan2가 2·3사분면 (±90°~±180°)
  → 최대 조향각(≈±0.6 rad)을 크게 초과해 클리핑 → 조향이 항상 최대로 고착

  올바른 후진 공식: δ = atan2(+wz × L, −vx)
    −vx > 0 이므로 atan2가 1·4사분면으로 돌아옴
    wz 부호를 반전해 경로 회전 방향과 조향 방향을 일치시킴
```

수식으로 정리:

| 상황 | 공식 |
| :--- | :--- |
| 전진 (`vx > 0`) | `δ = atan2(−wz × L, vx)` |
| 후진 (`vx < 0`) | `δ = atan2(+wz × L, −vx)` |
| 정지 (`abs(vx) < 0.05`) | `δ = 0` (정지 중 급선회 방지) |

> **부호 반전 이유 (전진):** ROS/MPPI 표준에서 `wz > 0 = CCW = 좌회전`이지만 CARLA에서는 `steer > 0 = 우회전`이므로 wz에 `-1`을 곱해 방향을 일치시킨다.
>
> **부호 이중 반전 (후진):** 후진 시 핸들을 오른쪽으로 꺾으면 차의 경로는 왼쪽으로 휜다. 이 물리적 반전을 보정하기 위해 wz 부호와 vx 부호를 모두 반전한다.

* `L`: 축간거리(wheelbase). CLI `--wheelbase` 로 지정 (기본값: microlino 1.47 m).
* `max_steer_rad`: 실행 시 `vehicle.get_physics_control().wheels[:2]`에서 자동 조회.

#### 실행 방법

```bash
cd ~/carla
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source mppi_ws/install/setup.bash
ros2 run dual_filter cmd_vel_to_carla \
  --ros-args -p use_sim_time:=true \
  -- --rolename car --wheelbase 1.47
```

`--` 이후는 CARLA/argparse 인수, 이전은 ROS 인수.

#### 차량별 조정 필요 파라미터

| 파라미터 | 조정 방법 |
| :--- | :--- |
| `--wheelbase` | CARLA physics_control 또는 차량 blueprint 스펙으로 확인 |
| `_KP_SPEED` | 코드 내 상수 직접 수정. 오버슈트 시 낮춤 |

---

### 10.4 `follow_path_client.py` 상세

파일 위치: `mppi_ws/src/dual_filter/dual_filter/follow_path_client.py`

ROS 2 노드 이름: `follow_path_client` (launch 파일의 `name` 파라미터)

CSV 경로 추종과 주차 모드를 IDLE / CSV_FOLLOWING / PARKING 세 가지 상태로 관리하는 상태 머신 노드. 단순히 CSV path를 전달하는 것에서 나아가, RViz "2D Goal Pose" 클릭으로 주차 모드 전환 → 경로 계산 → 주차 실행 → 자동 복귀까지의 전체 흐름을 조율한다.

---

#### 상태 머신

```text
IDLE
  │  /csv_path 수신 + /odometry/local 수신 시 자동 전환
  ▼
CSV_FOLLOWING ──────────── FollowPath action 전송 중 (/csv_path 경로 추종)
  │  RViz "2D Goal Pose" 클릭 (/goal_pose 수신)
  │  → 현재 FollowPath 취소 → PARKING 전환
  ▼
PARKING ────────────────── ComputePathToPose → FollowPath 순으로 주차 기동
  │  주차 FollowPath 완료 (성공/실패/취소 무관)
  │  → IDLE → CSV_FOLLOWING 자동 복귀
  ▼
(CSV_FOLLOWING 재진입)
```

| 상태 | 의미 | controller_server에 goal 유무 |
| :--- | :--- | :---: |
| `IDLE` | 대기. CSV나 odometry가 아직 없을 때, 또는 전환 직전 순간 | 없음 |
| `CSV_FOLLOWING` | `/csv_path`를 따라 MPPI 자율주행 중 | FollowPath 실행 중 |
| `PARKING` | RViz로 지정한 목표로 주차 기동 중 | ComputePathToPose → FollowPath 순 |

---

#### 토픽 인터페이스

| 방향 | 토픽 | 타입 | QoS | 설명 |
| :---: | :--- | :--- | :---: | :--- |
| 구독 | `/csv_path` | `nav_msgs/Path` | transient_local RELIABLE | CSV 경로. `csv_to_utm` 발행 |
| 구독 | `/odometry/local` | `nav_msgs/Odometry` | depth 10 | 현재 위치. `local_ekf` 발행 |
| 구독 | `/goal_pose` | `geometry_msgs/PoseStamped` | depth 10 | RViz "2D Goal Pose" 클릭 시 발행 |
| 발행 | `/mode_status` | `std_msgs/String` | depth 10 | 현재 모드 문자열 (1 Hz 타이머) |

---

#### Action 클라이언트

| Action | 서버 | 용도 |
| :--- | :--- | :--- |
| `follow_path` | `controller_server` | CSV 추종 및 주차 경로 실행 |
| `compute_path_to_pose` | `planner_server` | 주차 목표까지의 Reeds-Shepp 경로 계산 |

---

#### 주차 동작 흐름

```text
① RViz "2D Goal Pose" 클릭
      ↓ /goal_pose (PoseStamped, 위치 + 화살표 방향=최종 heading)

② 현재 CSV FollowPath goal 취소 (cancel_goal_async)
      ↓ 취소 확인 후

③ planner_server.server_is_ready() 확인
      ↓ ComputePathToPose goal 전송
         goal.goal       = goal_pose (위치 + heading 포함)
         goal.planner_id = 'GridBased'  (SmacPlannerHybrid REEDS_SHEPP)
         goal.use_start  = False        (현재 로봇 위치를 시작점으로 사용)

④ 경로 계산 완료 → path (nav_msgs/Path) 수신
      ↓ 경로가 비어있으면 → IDLE → CSV 복귀

⑤ controller_server.server_is_ready() 확인
      ↓ FollowPath goal 전송 (계산된 주차 경로)
         controller_id = 'ParkingPath'  ← 주차 전용 MPPI 플러그인 (전진·후진 혼합)

⑥ 주차 FollowPath 완료 (status 4/5/6 무관) → IDLE → CSV 복귀
```

> **화살표 방향과 주차 heading**: RViz에서 드래그한 화살표 머리 방향 = 차량이 도착했을 때 전면이 향하는 방향. SmacPlannerHybrid(REEDS_SHEPP)가 이 최종 heading을 반드시 만족하는 경로를 계산하므로, 화살표 방향을 정확히 지정해야 원하는 진입 방향으로 주차된다.
>
> **controller_id 분리 이유**: CSV 추종에는 `FollowPath`(`vx_max=5.0`, `PreferForwardCritic` 포함), 주차에는 `ParkingPath`(`vx_max=2.0`, `PreferForwardCritic` 제거)를 사용해 런타임 파라미터 변경 없이 모드를 전환한다. 주차는 SmacPlannerHybrid(REEDS_SHEPP)가 전진·후진 혼합 경로를 계획하므로 MPPI도 두 방향 모두 허용해야 하지만, CSV 추종 중 불필요한 후진을 억제하는 `PreferForwardCritic`은 제거해 패널티 없이 경로를 따른다.

##### 주차 성공 판별 기준

Nav2 `SimpleGoalChecker`가 다음 두 조건을 동시에 만족할 때 성공 판정을 내린다:

| 조건 | 설정값 | 기준점 |
| :--- | :---: | :--- |
| 위치 오차 | 0.5 m 이내 | `base_link` 원점 (후륜축 중심) |
| heading 오차 | 0.3 rad (≈17°) 이내 | 화살표 방향 대비 차량 yaw |

`base_link` 원점이 **후륜축 중심**이므로, RViz에서 클릭한 위치는 "후륜축이 도달해야 할 좌표"다. 차량 뒤범퍼를 특정 지점에 붙이려면 뒤범퍼~후륜축 거리(약 0.5~0.7 m)만큼 앞으로 이동한 지점을 클릭해야 한다.

---

#### CSV 경로 트리밍

`/csv_path` 수신 또는 CSV 복귀 시, 로봇 현재 위치에서 가장 가까운 waypoint 인덱스를 찾아 그 이후 구간만 잘라서 `FollowPath`에 전송한다. 경로 시작점이 로봇 현재 위치보다 멀리 있어도 정상 동작한다.

```text
로봇 현재 위치 (UTM x, y)
  → 전체 /csv_path 를 순회 → 가장 가까운 waypoint index 탐색
  → trimmed_path = poses[closest_idx:]
  → FollowPath(trimmed_path)
```

---

#### QoS 주의사항

`csv_to_utm`은 `/csv_path`를 `transient_local RELIABLE KeepLast(1)`로 발행한다. `follow_path_client`가 **다른 QoS로 구독하면 경로를 영원히 수신하지 못한다**. 코드 내 QoS가 동일하게 설정되어 있으므로 수정하지 않는다.

---

#### Action 결과 코드

ROS 2 `GoalStatus` 표준값 (`action_msgs/msg/GoalStatus`):

| status | 의미 |
| :---: | :--- |
| 4 | 성공 (SUCCEEDED — 목표 도달) |
| 5 | 취소됨 (CANCELED) |
| 6 | 중단 (ABORTED — progress_checker 실패 또는 controller 오류) |

주차 결과(`_on_parking_result`)는 status 값에 무관하게 항상 CSV 복귀를 실행한다. 주차가 ABORTED로 실패해도 자동으로 CSV 추종으로 돌아간다.

---

#### `server_is_ready()` — 논블로킹 서버 확인

`_start_parking()` 및 `_send_follow_path()` 내부에서 `server_is_ready()`(비동기, 즉시 반환)로 서버 준비 여부를 확인한다. 콜백 컨텍스트 내에서 `wait_for_server(timeout_sec=N)` (블로킹 호출)을 사용하면 SingleThreadedExecutor 전체가 N초간 멈추어 타이머 등 모든 콜백이 중단되는 문제가 발생한다. 이로 인해 PARKING 상태가 mode_status에 기록되지 않고 IDLE로 즉시 전환되는 버그가 생긴다. 서버가 준비되지 않은 경우(lifecycle_manager가 아직 activate 중)에는 에러를 로깅하고 IDLE로 복귀한다.

---

### 10.5 CSV 경로 스플라인 보간 도구 (`csv_interpolater.py`)

파일 위치: `mppi_ws/src/gnss_to_utm/src/csv_interpolater.py`

#### MPPI에서 스플라인 보간이 필수인 이유

MPPI는 수천 개의 후보 궤적을 경로와 비교해 비용을 평가하는 방식으로 동작한다. 이 과정에서 경로 자체의 품질이 추종 성능을 직접 결정한다.

GNSS 수록 주기가 1~5 Hz라면 원시 CSV의 waypoint 간격이 1~3 m 이상이 된다. MPPI가 이처럼 간격이 넓은 경로를 입력받으면 다음 문제가 구조적으로 발생한다:

##### ① PathAlignCritic 방향 불연속 → 조향 진동

```text
PathAlignCritic 은 인접 waypoint 간 방향 벡터를 "경로 방향"으로 간주한다.
원시 CSV에서 waypoint 간격이 크면 GPS 노이즈가 방향 계산에 크게 반영되어
waypoint마다 경로 방향이 불연속으로 꺾인다.

MPPI 는 이 방향 변화를 "실제 커브"로 인식하므로,
직선 구간에서도 불필요한 조향 명령이 생성된다.
```

##### ② offset_from_furthest 의 실거리가 불균일

```text
PathFollowCritic.offset_from_furthest = 5 의 의미:
  "예측 구간 내 최원 waypoint에서 5번째 앞 waypoint까지의 거리"

원시 CSV (1~3 m 간격): offset=5 → 실거리 5~15 m (매우 불균일)
보간 CSV (0.1 m 간격): offset=5 → 실거리 항상 0.5 m (완전 균일)

간격이 불균일하면 MPPI가 커브 구간에서는 너무 가까운 점을,
직선 구간에서는 너무 먼 점을 추적해 속도가 불안정해진다.
```

##### ③ PathAngleCritic heading 계산 오차

```text
PathAngleCritic은 "현재 차량 heading vs 경로 방향" 오차를 최소화한다.
경로 방향이 waypoint 간 직선으로만 정의되면,
커브 진입 시 경로 방향이 계단식으로 변해 적절한 사전 조향이 불가능하다.
스플라인 보간 후에는 경로 방향이 연속적이므로 커브 진입 수 m 전부터
부드럽게 heading을 틀기 시작한다.
```

**결론**: 스플라인 보간을 적용하면 MPPI critic 들이 이상적인 입력을 받게 되어 파라미터 튜닝 없이도 추종 품질이 즉각적으로 개선된다. `csv_interpolater.py` 를 경로 녹화 직후 한 번 실행해 두면 이후 모든 주행에서 같은 dense CSV를 재사용할 수 있다.

#### 동작 원리

```text
입력 CSV (원시 GNSS UTM)
  예) 점 수: 300, 총 거리: 500 m, 평균 간격: 1.67 m
      ↓
1. 중복점 제거 (인접 점 거리 < 1e-6 m 제거)
2. 누적 호 길이 파라미터 s 계산
   s[0]=0, s[i] = s[i-1] + dist(pt[i-1], pt[i])
3. CubicSpline(s, easting), CubicSpline(s, northing) 피팅
   → C² 연속성 보장 (매끄러운 2차 미분 = 곡률 연속)
4. s_new = [0, 0.1, 0.2, ..., total_len] (0.1 m 등간격)
5. (e_new, n_new) = (cs_e(s_new), cs_n(s_new))
      ↓
출력 CSV (보간된 UTM, 10 cm 간격)
  예) 점 수: 5000, 총 거리: 500 m, 간격: 0.1 m
```

#### csv_interpolater.py 실행 방법

```bash
# ROS 환경 불필요. 시스템 Python3 또는 .venv 어디서나 실행 가능.
cd ~/carla/mppi_ws/src/gnss_to_utm/src

# 기본 (10 cm 간격)
python3 csv_interpolater.py /path/to/input.csv /path/to/output_10cm.csv

# 간격 변경 (50 cm)
python3 csv_interpolater.py input.csv output_50cm.csv --interval 0.5

# 시각화 포함 (matplotlib 필요)
python3 csv_interpolater.py input.csv output.csv --interval 0.1 --plot
```

#### 보간 결과 csv_to_utm 에 적용

```yaml
# mppi_ws/src/gnss_to_utm/config/csv_to_utm.yaml
csv_to_utm:
  ros__parameters:
    use_sim_time: true
    csv_file_path: "/path/to/output_10cm.csv"   # ← 보간된 파일로 교체
```

#### MPPI 경로 품질 개선 효과

| 항목 | 원시 CSV (1~3 m 간격) | 보간 CSV (10 cm 간격) |
| :--- | :--- | :--- |
| PathAlignCritic 방향 연속성 | waypoint마다 불연속 꺾임 | C² 연속 (완전 부드러움) |
| 조향 진동 | 직선에서도 발생 | 대폭 감소 |
| offset_from_furthest 실거리 | 1~15 m (불균일) | 항상 `offset × 0.1 m` (균일) |
| PathAngleCritic 사전 회전 | 커브 직전에야 반응 | 수 m 전부터 부드럽게 선회 |
| RViz 경로 시각화 | 꺾은선 | 부드러운 곡선 |

> **10 cm 간격의 trade-off**: 원시 300점 → 보간 후 5000점. `csv_to_utm`이 발행하는 `/csv_path`의 pose 수가 증가하므로 `PathHandler`의 pruning 빈도가 올라간다. 경로가 매우 길어(5 km 이상) 메모리 사용이 걱정될 경우 `--interval 0.2`(20 cm)로 타협 가능.

---

### 10.6 `controller.launch.py` 상세

파일 위치: `mppi_ws/src/dual_filter/launch/controller.launch.py`

`controller_server`, `planner_server`, `lifecycle_manager`, `follow_path_client` 네 노드를 하나의 launch 파일로 묶어 실행한다. 이 파일을 사용하는 이유는 `controller_server`와 `planner_server`가 Nav2 lifecycle 노드이기 때문이다 — `ros2 run`으로 단독 실행하면 UNCONFIGURED 상태로 멈춰 action server가 활성화되지 않는다.

---

#### 포함 노드 목록

| 노드 | 패키지 | 실행파일 | 역할 |
| :--- | :--- | :--- | :--- |
| `controller_server` | `nav2_controller` | `controller_server` | MPPI 제어 루프. `/follow_path` action server 제공 |
| `planner_server` | `nav2_planner` | `planner_server` | SmacPlannerHybrid. `/compute_path_to_pose` action server 제공 |
| `lifecycle_manager_controller` | `nav2_lifecycle_manager` | `lifecycle_manager` | 두 서버를 `configure → activate` 로 자동 전환 (`autostart: True`) |
| `follow_path_client` | `dual_filter` | `follow_path_client` | IDLE/CSV_FOLLOWING/PARKING 상태 머신 |

---

#### 파라미터

모든 노드는 `use_sim_time: True`로 실행된다. `controller_server`와 `planner_server`는 `nav2_carla_params.yaml`을 공통 파라미터 파일로 사용한다.

```python
params_file = os.path.join(
    get_package_share_directory('dual_filter'),
    'config', 'nav2_carla_params.yaml',
)
```

---

#### lifecycle_manager 동작

```text
ros2 launch dual_filter controller.launch.py
  ↓
lifecycle_manager_controller 시작 (autostart=True)
  ↓ node_names: ['controller_server', 'planner_server']
  ↓ configure → activate 순으로 두 서버 전환
  ↓ "Managed nodes are active" 로그 출력

이후:
  controller_server: /follow_path action server 활성화
  planner_server:    /compute_path_to_pose action server 활성화
  follow_path_client: 두 action server 발견 후 CSV 추종 시작
```

확인 명령:

```bash
ros2 lifecycle get /controller_server    # active [3] 이어야 정상
ros2 lifecycle get /planner_server       # active [3] 이어야 정상
ros2 action info /follow_path            # Action servers: 1 이어야 정상
ros2 action info /compute_path_to_pose   # Action servers: 1 이어야 정상
```

---

#### launch 실행 명령

```bash
source /opt/ros/humble/setup.bash
source ~/carla/nav2_ws/install/setup.bash   # OpenMP 빌드 버전 (없으면 생략)
source ~/carla/mppi_ws/install/setup.bash
export OMP_NUM_THREADS=8
ros2 launch dual_filter controller.launch.py
```

> **`follow_path_client` 중복 실행 주의**: `controller.launch.py`에 `follow_path_client`가 포함되어 있으므로, `ros2 run dual_filter follow_path_client`를 별도 터미널에서 추가 실행하면 두 인스턴스가 동시에 `/goal_pose`를 구독해 주차 모드 전환이 오동작한다.

---

## 11. 레퍼런스 맵 제작 (MPPI 추종 경로 생성)

MPPI가 추종할 경로는 **실제 주행 데이터를 녹화 → UTM CSV 변환 → 스플라인 보간**의 3단계로 제작한다.

관련 스크립트 위치:

| 스크립트 | 경로 |
| :--- | :--- |
| `f9r_to_csv.py` | `mppi_ws/src/gnss_to_utm/src/f9r_to_csv.py` |
| `csv_interpolater.py` | `mppi_ws/src/gnss_to_utm/src/csv_interpolater.py` |

---

### Step 1 — 주행 경로 ROS2 bag 녹화

레퍼런스 경로를 주행하면서 F9R GNSS 토픽을 bag으로 기록한다.

```bash
# 저장 경로는 자유롭게 지정
ros2 bag record /carla/car/f9r/fix \
  -o ~/carla/mppi_ws/src/gnss_to_utm/gnss_data/ros2bag/route_1
```

| 항목 | 내용 |
| :--- | :--- |
| 녹화 토픽 | `/carla/car/f9r/fix` |
| 메시지 타입 | `sensor_msgs/NavSatFix` |
| 필드 | `latitude`, `longitude` (WGS84 도 단위) |
| storage | sqlite3 (ROS2 Humble 기본값) |

> **CARLA 환경**: 토픽명은 `stack.json`의 vehicle `id` 필드에 따라 결정된다.
> 현재 `id: car` → 토픽명 `/carla/car/f9r/fix`.
> 실차에서는 GNSS 드라이버가 발행하는 `NavSatFix` 토픽명으로 변경한다.

---

### Step 2 — bag → 원시 UTM CSV 변환 (`f9r_to_csv.py`)

```bash
# f9r_to_csv.py 상단의 경로를 녹화한 bag에 맞게 수정 후 실행
# bag_path  : 녹화한 bag 디렉토리 경로 (확장자 없이)
# csv_path  : 출력 CSV 경로 (자동 생성됨)

cd ~/carla/mppi_ws/src/gnss_to_utm/src
python3 f9r_to_csv.py
```

스크립트 상단의 두 경로를 직접 편집해야 한다:

```python
# f9r_to_csv.py 17~24번째 줄
bag_path = "/home/hannibal/carla/mppi_ws/src/gnss_to_utm/gnss_data/ros2bag/route_1"
csv_path = "/home/hannibal/carla/mppi_ws/src/gnss_to_utm/gnss_data/csv/route_1.csv"
```

실행 결과:

```text
Input bag path:  .../gnss_data/ros2bag/route_1
Output CSV path: .../gnss_data/csv/route_1.csv
Successfully processed 1847 messages and saved to ...
```

출력 CSV 형식:

```text
X(E/m),Y(N/m)
316842.123456789012345,3946210.987654321098765
316842.234567890123456,3946211.098765432109876
...
```

> **UTM 존 자동 감지**: 첫 번째 메시지의 경도로 UTM zone을 계산하므로 별도 설정 불필요.
> 출력 정밀도는 소수점 15자리 (약 0.1 nm 정밀도).

---

### Step 3 — 스플라인 보간 + 등간격 재샘플링 (`csv_interpolater.py`)

원시 CSV는 주행 속도에 따라 점 간격이 1~3 m로 불균일하다.
MPPI critic이 안정적으로 작동하려면 10 cm 등간격으로 재샘플링이 필요하다
(이유는 [Section 10.5](#105-csv-경로-스플라인-보간-도구-csv_interpolaterpy) 참고).

```bash
cd ~/carla/mppi_ws/src/gnss_to_utm/src

# 기본 (10 cm 간격)
python3 csv_interpolater.py \
  ../gnss_data/csv/route_1.csv \
  ../gnss_data/csv/route_1_10cm.csv

# 간격 변경 (20 cm)
python3 csv_interpolater.py \
  ../gnss_data/csv/route_1.csv \
  ../gnss_data/csv/route_1_20cm.csv \
  --interval 0.2

# 보간 결과 시각화 확인 (matplotlib 필요)
python3 csv_interpolater.py \
  ../gnss_data/csv/route_1.csv \
  ../gnss_data/csv/route_1_10cm.csv \
  --plot
```

실행 출력 예:

```text
입력: route_1.csv
  점 수    : 1847
  총 길이  : 2304.187 m
  평균 간격: 1.248 m

재샘플링 간격: 0.1 m
  보간 후 점 수: 23043

저장 완료: route_1_10cm.csv
```

---

### Step 4 — csv_to_utm 노드에 적용

보간된 CSV를 `csv_to_utm.yaml`에 등록한다.

```yaml
# mppi_ws/src/gnss_to_utm/config/csv_to_utm.yaml
csv_to_utm:
  ros__parameters:
    use_sim_time: true
    csv_file_path: "/home/hannibal/carla/mppi_ws/src/gnss_to_utm/gnss_data/csv/route_1_10cm.csv"
```

---

### 전체 흐름 요약

```text
① ros2 bag record /carla/car/f9r/fix
        │  (주행 중 GNSS NavSatFix 녹화)
        ▼
   route_1/  (sqlite3 bag)
        │
② python3 f9r_to_csv.py
        │  (bag → UTM 변환, 점 간격 ~1 m)
        ▼
   route_1.csv  [X(E/m), Y(N/m)]
        │
③ python3 csv_interpolater.py route_1.csv route_1_10cm.csv
        │  (cubic spline 보간 + 10 cm 등간격 재샘플링)
        ▼
   route_1_10cm.csv  [X(E/m), Y(N/m), 10 cm 간격]
        │
④ csv_to_utm.yaml 의 csv_file_path 업데이트
        │
        ▼
   csv_to_utm 노드 → /csv_path (nav_msgs/Path) 발행
        │
        ▼
   follow_path_client → FollowPath action → MPPI 추종 시작
```

---

## 12. 시스템 실행 매뉴얼

dual_filter 스택이 빌드된 상태이고 Nav2가 설치된 상태를 전제로 한다. 각 구성 파일의 상세 내용은 섹션 10.1.1–10.1.3을 참고한다.

### 사전 준비 (빌드)

```bash
# robot_localization 설치 (최초 1회)
sudo apt install ros-humble-robot-localization

# dual_filter / gnss_to_utm 패키지 빌드 (최초 1회 또는 소스 수정 후)
cd ~/carla/mppi_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_filter gnss_to_utm --symlink-install
source install/setup.bash
```

> **`No executable found` 오류 시**: `setup.py`에 entry point를 추가한 뒤 빌드하지 않으면
> 발생한다. 위 명령으로 재빌드하면 `cmd_vel_to_carla`, `follow_path_client` 가 등록된다.

### 12.1 전체 실행 순서

아래 순서를 정확히 따라야 한다. 특히 `ros2_sensor.py`가 `/clock`을 먼저 발행해야 EKF 시간 동기화가 올바르게 동작한다.

```text
터미널 1: CARLA 시뮬레이터
터미널 2: manual_control (맵 로드 + 차량 스폰)
터미널 3: ros2_sensor.py (/clock + 센서 토픽)
터미널 4: dual_filter launch (EKF + GNSS 파이프라인)
터미널 5 (선택): RViz2 시각화
터미널 6: csv_to_utm launch (경로 파일 → /csv_path)
터미널 7: controller.launch.py
           ├─ controller_server (MPPI, Nav2 lifecycle)
           ├─ planner_server    (SmacPlannerHybrid, Nav2 lifecycle)
           ├─ lifecycle_manager (autostart: 두 서버 자동 activate)
           └─ follow_path_client (CSV 추종 ↔ 주차 모드 전환)
터미널 8: cmd_vel_to_carla (MPPI → CARLA 제어)

※ 터미널 9는 불필요 — follow_path_client 가 controller.launch.py 에 통합됨
  (ros2 run dual_filter follow_path_client 를 별도 실행하면 중복 인스턴스 발생)
```

#### 터미널 1 — CARLA 시뮬레이터

```bash
cd ~/carla
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ./CarlaUE4.sh -RenderOffScreen -quality-level=Low
```

> `--ros2` 제거: CARLA 내장 ROS2 브리지와 터미널 3의 `ros2_sensor.py --python-ros2`가 동시에
> LibCarla 스트리밍 서버에 붙으면 "Invalid session: no stream available" 에러가 발생하므로 반드시 제외.

#### 터미널 2 — 차량 스폰

스폰 좌표는 `--spawn-x/--spawn-y/--spawn-z/--spawn-yaw` 인자로 지정한다(맵에 맞게 좌표만 교체).
커스텀 맵 적용·좌표 산출 방법은 [Section 15.7](#157-step-e--section-121-연동-맵--스폰-좌표)을 참고한다.

**카를라맵 (Town01_Opt) 버전:**

```bash
cd ~/carla
source .venv/bin/activate
python PythonAPI/util/config.py --map Town01_Opt \
  && python PythonAPI/examples/manual_control.py --rolename car \
     --filter vehicle.micro.microlino --generation 2 --sync \
     --spawn-x 299.4 --spawn-y 133.24 --spawn-z 0.3 --spawn-yaw 0.0
```

**만도맵 (Mando1 / Mando2 / Mando3) 버전:** `--map`에 `Mando1` / `Mando2` / `Mando3` 중 하나를 지정한다(사용할 맵이 import 돼 있어야 함, Section 15). `Mando3`은 레퍼런스 경로 기록을 위해 장애물을 제거한 버전(`CustomMap/MandoParking3`)으로, 도로 형상·스폰 좌표는 `Mando1`/`Mando2`와 동일하다.

```bash
cd ~/carla
source .venv/bin/activate
python PythonAPI/util/config.py --map Mando2 \
  && python PythonAPI/examples/manual_control.py --rolename car \
     --filter vehicle.micro.microlino --generation 2 --sync \
     --spawn-x -93.6 --spawn-y 0.0 --spawn-z 0.3 --spawn-yaw -90.0
```

#### 터미널 3 — 센서 브리지 (`/clock` 포함)

```bash
cd ~/carla
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python ros2_sensor/ros2_sensor.py \
  -f ros2_sensor/stack.json \
  --attach-existing --passive --python-ros2 \
  --base-frame base_link --wait-for-vehicle 30 \
  --sensors f9r f9p imu lidar_2d rear_cam
```

> **후방 카메라(`rear_cam`)**: 주차 시 차량 뒷모습 확인용 모니터링 카메라.
> `stack.json`에 `spawn_point: {x:-1.3, y:0, z:1.0, pitch:12, yaw:180}` 으로 정의(후면 범퍼 높이,
> `yaw:180`으로 후방을 바라봄, `pitch:12`로 지면 쪽을 살짝 내려다봄 → 주차선 식별). 부착 위치·각도는
> `ros2_sensor/stack.json`의 `spawn_point`에서 조절한다(`x` +앞/−뒤, `y` +좌/−우, `z` 높이, `yaw` CCW+).
> 발행 토픽은 `/carla/car/rear_cam/image`(`sensor_msgs/Image`)이며 EKF·Nav2 파이프라인에는 관여하지 않는다.
> 확인: `ros2 run rqt_image_view rqt_image_view /carla/car/rear_cam/image`
>
> **QoS 주의**: 카메라 이미지는 고대역폭 스트림이라 `BEST_EFFORT`로 발행한다(`RELIABLE`로 하면
> ACK·재전송 백프레셔로 전체 렉 유발). 따라서 **RViz2 Image 디스플레이의 `Reliability Policy`를
> `Best Effort`로** 맞춰야 한다(기본 `Reliable`이면 QoS 불일치로 `No messages will be sent` 경고와
> 함께 안 보임). 제공된 `ros2_sensor.rviz`에는 이미 반영돼 있고, `rqt_image_view`는 자동으로 맞춰진다.
> 자세한 QoS 규칙은 [Section 6.3](#63-ros2_sensorpy-센서-브리지-노드)의 "센서 QoS" 항목을 참고한다.

#### 터미널 4 — Dual Filter (EKF + GNSS)

```bash
source /opt/ros/humble/setup.bash
source ~/carla/mppi_ws/install/setup.bash
ros2 launch dual_filter dual_filter.launch.py
```

#### 터미널 5 (선택) — RViz2

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/carla/ros2_sensor/rviz/ros2_sensor.rviz \
  --ros-args -p use_sim_time:=true
```

#### 터미널 6 — 경로 파일 → `/csv_path`
<!-- csv_to_utm.yaml에서 csv_file_path를 실제 경로로 설정 후: -->

```bash
source /opt/ros/humble/setup.bash
source ~/carla/mppi_ws/install/setup.bash
ros2 launch gnss_to_utm csv_to_utm.launch.py
```

#### 터미널 7 — Nav2 controller_server (MPPI)
<!--
source /opt/ros/humble/setup.bash
source ~/carla/nav2_ws/install/setup.bash    # OpenMP 빌드 버전 로드 (apt 버전보다 우선)
source ~/carla/mppi_ws/install/setup.bash
export OMP_NUM_THREADS=8                     # MPPI 병렬 스레드 수 (권장 시작값)
ros2 launch dual_filter controller.launch.py
-->

```bash
source /opt/ros/humble/setup.bash
source ~/carla/nav2_ws/install/setup.bash
source ~/carla/mppi_ws/install/setup.bash
export OMP_NUM_THREADS=8
ros2 launch dual_filter controller.launch.py
```

> **소싱 순서 중요**: `nav2_ws`를 `mppi_ws`보다 먼저 소싱해야 한다. 나중에 소싱한 워크스페이스가 앞의 것을 덮어쓰기 때문에, `nav2_ws` → `mppi_ws` 순서로 해야 OpenMP 버전 `libmppi_controller.so`가 `mppi_ws`의 `dual_filter` 패키지와 함께 올바르게 로드된다.
> **`OMP_NUM_THREADS` 기준**: 24코어 기준 8로 시작. `ros2 topic hz /cmd_vel`이 설정한 `controller_frequency`에 미달하면 12로 높인다. 다른 노드(EKF, costmap 등)가 느려지면 4~6으로 낮춘다.

> **`ros2 run` 대신 `ros2 launch`를 사용하는 이유**: `controller_server`는 Nav2 lifecycle node다.
> `ros2 run`으로 직접 실행하면 UNCONFIGURED 상태로 머물러 `/follow_path` action server가
> 활성화되지 않는다 (Action servers: 0). `controller.launch.py`는 `lifecycle_manager`를
> 함께 기동해 자동으로 configure → activate 전환을 수행한다.
>
> **중복 실행 주의**: 같은 명령을 두 터미널에서 실행하면 `/controller_server` 노드가 2개
> 생겨 action server가 동작하지 않는다. 실행 전 `ros2 node list | grep controller`로
> 기존 인스턴스가 없는지 확인한다.

#### 터미널 8 — `cmd_vel` → CARLA 제어

```bash
cd ~/carla
source /opt/ros/humble/setup.bash
source ~/carla/mppi_ws/install/setup.bash
source .venv/bin/activate
ros2 run dual_filter cmd_vel_to_carla \
  --ros-args -p use_sim_time:=true \
  -- --rolename car --wheelbase 1.47
```

#### 주차 사용법

RViz2 툴바에서 "2D Goal Pose" 선택(단축키 G) → 목표 위치에서 드래그해 진입 방향 지정 → 마우스 놓으면 `follow_path_client` 가 자동으로 플래너 호출 → 주차 기동.


### 12.2 동작 확인

EKF 스택 확인:

```bash
# sim time 확인
ros2 topic echo --once /clock
ros2 param get /local_ekf use_sim_time
ros2 param get /global_ekf use_sim_time

# 주요 입력 stamp가 /clock과 동일한지 확인
ros2 topic echo --once /wheel_encoder/data --field header.stamp
ros2 topic echo --once /carla/car/imu/data --field header.stamp
ros2 topic echo --once /odometry/local --field header.stamp
ros2 topic echo --once /odometry/global --field header.stamp

# TF 트리 확인 (utm → odom → base_link 구조인지)
ros2 run tf2_tools view_frames

# wheel / IMU 입력 확인
ros2 topic echo --once /wheel_encoder/data --field twist.twist.linear
ros2 topic echo --once /carla/car/imu/data --field angular_velocity

# 로컬 EKF 출력 (MPPI 제어 입력용 — GNSS jump 없이 부드러워야 함)
ros2 topic echo /odometry/local
# 글로벌 EKF 출력 (utm → odom 보정용)
ros2 topic echo /odometry/global
# GNSS 브리지 출력
ros2 topic echo /odometry/gnss
```

MPPI / 경로 추종 확인:

```bash
# controller_server lifecycle 상태 확인 — active [3] 이어야 정상
ros2 lifecycle get /controller_server

# /follow_path action server 활성화 확인 — Action servers: 1 이어야 정상
ros2 action info /follow_path
# Action servers: 0 이면 controller_server 가 UNCONFIGURED 상태임
# → 터미널 7을 ros2 launch dual_filter controller.launch.py 로 재실행

# MPPI가 cmd_vel을 발행하는지 확인
ros2 topic echo /cmd_vel

# FollowPath action 상태 확인
ros2 action list
ros2 action info /follow_path

# MPPI trajectory 시각화 (visualize: true 설정 시)
ros2 topic echo /trajectories --no-arr

# TF tree 전체 확인
ros2 run tf2_tools view_frames

# costmap 발행 확인
ros2 topic hz /local_costmap/costmap
```

### 12.3 최소 실행 (costmap 없이 경로 추종만 테스트)

장애물 회피 없이 경로 추종 기능만 먼저 테스트하려면 `nav2_carla_params.yaml`의 `obstacle_layer`를 비활성화하고 critics에서 `CostCritic`을 제거한다.

```yaml
# nav2_carla_params.yaml 수정:
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["inflation_layer"]   # obstacle_layer 제거

# FollowPath.critics 수정:
critics:
  [
    "ConstraintCritic",
    "GoalCritic",
    "GoalAngleCritic",
    "PathAlignCritic",
    "PathFollowCritic",
    "PathAngleCritic",
    "PreferForwardCritic",
  ]
  # CostCritic 제거 (costmap 없이 동작 가능)
```

이 구성으로 터미널 7-9만 실행하면 costmap 없이 경로 추종을 테스트할 수 있다.

---

### 12.4 전체 데이터 흐름

```text
CARLA Simulator
  ├─ sim time ──────────────→ ros2_sensor.py ──→ /clock ──→ use_sim_time nodes
  │
  ├─ /carla/car/f9r/fix ──→ f9r_to_utm ──────────→ /f9r_utm ──────────┐
  │                     └──→ azimuth_calc ─────────→ /azimuth_angle ──┤
  │                                                                    ▼
  ├─ /carla/car/f9p/fix ──→ f9p_to_utm ──→ /f9p_utm          gnss_to_odom
  │                                                           │         │
  │                                               /odometry/gnss   /utm_datum
  │                                                           │         │
  ├─ /carla/car/imu/data ─────────────────────────────────────┼─────────┼──┐
  │                                                           │         │  │
  └─ ros2_sensor.py ──→ /carla/car/wheel_encoder/data ───┼─────────┼──┤
                                │                             │         │  │
                                └─────────────┬──────────────┘         │  │
                                              │                    csv_to_utm
                                              │                    /csv_path
                                              ▼                        │
                         local_ekf  ◄─ wheel(vx) + imu(wz)           │
                         global_ekf ◄─ wheel(vx) + imu(wz) + gnss    │
                              │                       │               │
                              ▼                       ▼               ▼
                    /odometry/local           utm → odom TF     /csv_path
                    odom → base_link TF       (전역 보정)       (레퍼런스 경로)
                          │                                          │
                          └─────────────────┬────────────────────────┘
                                            ▼
                                   controller_server (MPPI)
                                            │
                                            ▼
                                        /cmd_vel
                                            │
                                            ▼
                                   cmd_vel_to_carla → CARLA VehicleControl
```

---

### 12.5 종료

```bash
pkill -TERM -f 'follow_path_client'
pkill -TERM -f 'controller_server'
pkill -TERM -f 'cmd_vel_to_carla'
pkill -TERM -f 'ros2_sensor.py'
pkill -TERM -f 'manual_control.py'
pkill -TERM -f 'rviz2'
pkill -TERM -f 'CarlaUE4-Linux-Shipping'
pkill -9 -f 'CarlaUE4-Linux-Shipping'
```

---

### 12.6 알려진 오류 및 해결

아래는 실제 실행 중 발생한 오류와 적용한 수정 사항이다.

---

**① `FollowPath action 서버에 연결하지 못했습니다` — 10 초 타임아웃**

| 항목 | 내용 |
| :--- | :--- |
| 원인 | `follow_path_client` 가 10 초 내 서버 응답 없으면 영구 종료. `controller_server` lifecycle 활성화에 10 초 이상 소요될 수 있음 |
| 수정 파일 | `dual_filter/follow_path_client.py` |
| 수정 내용 | 10 초 1회 타임아웃 → 5 초 간격 무한 재시도 루프로 변경 |
| 확인 | `ros2 lifecycle get /controller_server` → `active [3]` 출력 후 자동 연결 |

---

**② `Model ackermann is not valid! Valid options are ... Ackermann`**

| 항목 | 내용 |
| :--- | :--- |
| 원인 | `nav2_carla_params.yaml` 에서 `motion_model: "ackermann"` (소문자) — Nav2 는 대소문자 구분 |
| 수정 파일 | `nav2_carla_params.yaml` |
| 수정 내용 | `motion_model: "ackermann"` → `motion_model: "Ackermann"` |
| 확인 | controller_server 로그에 `Model Ackermann is valid` 출력 |

---

**③ `/follow_path` action server 없음 — `controller_server` UNCONFIGURED**

| 항목 | 내용 |
| :--- | :--- |
| 원인 | `ros2 run nav2_controller controller_server` 만 실행하면 lifecycle_manager 없어서 UNCONFIGURED 상태 유지. `/follow_path` 서버 미생성 |
| 수정 파일 | `dual_filter/launch/controller.launch.py` (신규 생성) |
| 수정 내용 | `controller_server` + `lifecycle_manager(autostart: True)` 를 함께 실행하는 launch 파일 생성 |
| 실행 명령 | `ros2 launch dual_filter controller.launch.py` |
| 확인 | `ros2 action info /follow_path` → `Action servers: 1` |

---

**④ `Resulting plan has 0 poses in it` — CSV 경로 시작점과 로봇 위치 불일치**

**원인 분석:**

CSV 경로 파일(`route_1.csv`)은 GPS 녹화를 차량 스폰 위치가 아니라 **11.35 m 앞에서 시작**했다. path_handler 는 경로 파일의 첫 번째 pose 부터 스캔하므로, 첫 pose 가 `max_robot_pose_search_dist` 보다 멀면 즉시 0 poses 를 반환한다.

```text
CSV row 1    : datum 기준 +11.35 m (경로 파일 시작)   ← path_handler 시작 위치
  ...
  (약 250 m 루프)
  ...
CSV row 1835 : datum 기준  +0.31 m                   ← 로봇 실제 현재 위치
CSV row 1924 : datum 기준 +22.65 m (경로 파일 끝)
```

* `max_robot_pose_search_dist` 기본값 = `getMaxCostmapDist()` = 10 m
* row 1 이 11.35 m > 10 m → 즉시 반환 → **0 poses**

**수정 내용 (2가지 병행 적용):**

| 수정 파일 | 수정 내용 |
| :--- | :--- |
| `nav2_carla_params.yaml` | `FollowPath:` 섹션에 `max_robot_pose_search_dist: 30.0` 추가 → row 1 (11.35 m < 30 m) 포함 |
| `follow_path_client.py` | 경로 전송 전 `/odometry/local` 로 로봇 위치 확인 후, 가장 가까운 pose 부터 잘라서 전송 → **row 1835 (0.31 m) 부터 추종 시작** |

| 진단 | 명령 |
| :--- | :--- |
| 로봇 위치 확인 | `ros2 topic echo --once /odometry/local --field pose.pose.position` |
| 경로 첫 pose 확인 | `ros2 topic echo /csv_path \| grep -A3 "position:" \| head -6` |
| 수정 후 확인 | `follow_path_client` 로그: `가장 가까운 waypoint: index=1834, 거리=0.31 m` → `FollowPath goal 수락됨.` |

---

**⑤ CARLA `bind: Address already in use` (포트 2000)**

| 항목 | 내용 |
| :--- | :--- |
| 원인 | 이전 CARLA 프로세스가 포트 2000 점유 중. `pkill -TERM` 으로는 CARLA 가 종료되지 않음 (SIGTERM 무시) |
| 수정 내용 | `pkill -9 -f 'CarlaUE4-Linux-Shipping'` 사용 (SIGKILL) |
| 확인 | `ss -tlnp \| grep 2000` → 빈 결과 확인 후 CARLA 재시작 |

---

**⑥ `Transform data too old when converting from utm to odom` → 즉시 `Reached the goal!`**

| 항목 | 내용 |
| :--- | :--- |
| 증상 | `FollowPath goal 수락됨` → 수십 ms 만에 `Reached the goal!`. 차량이 전혀 이동하지 않음. |
| 에러 | `[tf_help]: Transform data too old … Data time: 1780166575s, Transform time: 463s` |
| 원인 | `csv_to_utm` 노드가 `use_sim_time` 미설정 → `path.header.stamp = this->get_clock()->now()` 가 **벽시계(wall clock) 시간**(~1780166575 s)을 반환. 반면 `controller_server` / 글로벌 EKF는 `use_sim_time: true` → TF는 **CARLA 시뮬레이션 시간**(~463 s)으로 발행. 두 클록이 완전히 달라 `utm→odom` TF 조회 실패. TF 조회 실패 시 MPPI 는 유효한 path pose 를 0개로 보고, SimpleGoalChecker 가 즉시 도달 판정. |
| 수정 파일 | `mppi_ws/src/gnss_to_utm/launch/csv_to_utm.launch.py` |
| 수정 내용 | `csv_to_utm` 노드 실행 시 `use_sim_time: True` 파라미터 추가 |

`csv_to_utm.launch.py`의 Node 선언에 `{'use_sim_time': True}`를 파라미터로 추가하면, `this->now()`가 벽시계 대신 CARLA 시뮬레이션 시간을 반환하여 `controller_server` / EKF의 TF 타임스탬프와 일치하게 된다.

```python
# 수정 전
csv_to_utm_node = Node(
    package='gnss_to_utm',
    executable='csv_to_utm',
    name='csv_to_utm',
    output='screen',
    parameters=[params_file],          # use_sim_time 미설정 → 벽시계 사용
)

# 수정 후
csv_to_utm_node = Node(
    package='gnss_to_utm',
    executable='csv_to_utm',
    name='csv_to_utm',
    output='screen',
    parameters=[params_file, {'use_sim_time': True}],  # sim time 사용
)
```

| 확인 방법 | 내용 |
| :--- | :--- |
| 빌드 불필요 | launch 파일 수정만으로 즉시 적용 (재빌드 불필요) |
| 정상 로그 | controller_server 에서 `Reached the goal!` 없이 MPPI 제어 루프 지속 실행 |

---

**⑦ `Control loop missed its desired rate` 연속 발생 → TF extrapolation → ABORT**

| 항목 | 내용 |
| :--- | :--- |
| 증상 | `[WARN] Control loop missed its desired rate of 20.0000Hz` 가 수십 초 연속 출력된 후, `[ERROR] Lookup would require extrapolation into the future` → `[WARN] Aborting handle.` |
| 에러 | `Requested time T+0.05 but the latest data is at time T, when looking up transform from frame [odom] to frame [utm]` |
| 원인 | MPPI 궤적 계산(CPU 단일 스레드)이 한 제어 주기(50 ms)를 초과. 제어 루프가 실제 시간보다 뒤처지면서 TF 조회 시 요청 시각이 TF 버퍼의 최신 데이터보다 50 ms 미래가 됨 → 하드 예외 발생 → 즉시 ABORT. `failure_tolerance` 타이머와 무관한 hard-error 경로로 종료됨 |
| 원인 수치 | `batch_size=2000, time_steps=56, controller_frequency=20 Hz` 기준 약 60 ms 소요 → 20 Hz 예산(50 ms) 초과 |
| 해결 A (즉시 적용) | `nav2_carla_params.yaml` 에서 계산량 감소: `batch_size: 2000 → 1000`, `time_steps: 56 → 40`, `visualize: false`, `controller_frequency: 10.0`, `model_dt: 0.10` |
| 해결 B (근본 해결) | nav2_mppi_controller 소스 빌드 + OpenMP 활성화 → [Section 12.7](#127-nav2_mppi_controller-openmp-빌드-연산-과다-근본-해결) 참고. i7-13700HX 24 스레드 기준 약 10x 속도 향상 기대 |
| 확인 | `ros2 topic hz /cmd_vel` → 설정한 `controller_frequency` 에 근접하는지 확인 |

---

**⑧ PARKING 모드로 전환되지 않음 — `server_is_ready()` DDS 발견 전 즉시 False**

| 항목 | 내용 |
| :--- | :--- |
| 증상 | RViz "2D Goal Pose" 클릭 후 `/mode_status` 에코에서 `PARKING`이 나타나지 않고 `CSV_FOLLOWING` / `IDLE` 만 반복됨 |
| 에러 | 없음 (로그에 오류 없이 조용히 실패) |
| 원인 | `_start_parking()` 내부에서 `planner_client.server_is_ready()`(timeout=0)를 사용. DDS 발견이 아직 완료되지 않은 경우 즉시 `False` 반환 → 함수 즉시 종료 → `PARKING → IDLE` 전환이 수 ms 만에 발생 → 1 초 주기 `/mode_status` 타이머에서 PARKING 상태가 관측되지 않음 |
| 수정 파일 | `dual_filter/follow_path_client.py` |
| 수정 내용 | `server_is_ready()` → `wait_for_server(timeout_sec=5.0)` 로 변경. `_send_follow_path()` 내의 `follow_client.server_is_ready()` 도 동일하게 수정 |
| 확인 | `follow_path_client` 로그에 `[PARK] planner_server 에 경로 요청 중 ...` → `[PARK] planner_server goal 수락.` 출력 |

---

**⑨ `global_costmap` 범위 오류 — planner가 경로 계산 불가**

| 항목 | 내용 |
| :--- | :--- |
| 증상 | 주차 시도 시 플래너가 빈 경로를 반환, `[PARK] 주차 경로 계산 실패 (빈 경로)` 로그 |
| 에러 | `[costmap_2d]: Sensor origin at (36.20, -117.72) is out of map bounds (0.00, 0.00) to (4.95, 4.95)` |
| 원인 | `global_costmap`에 `rolling_window`, `width`, `height`, `resolution` 미설정 → Nav2 기본값 적용: **5 m × 5 m 고정 격자, 원점 UTM (0,0)에 앵커됨**. 로봇의 실제 UTM 위치(예: 36.20, -117.72)가 맵 완전 밖에 있어 플래너가 시작점을 찾지 못함 |
| 핵심 개념 | `global_frame: utm`은 좌표계 선택이고, costmap 크기/원점은 별도 파라미터. `rolling_window: false`(기본값)이면 costmap 원점이 UTM (0,0)에 고정됨 |
| 수정 파일 | `dual_filter/config/nav2_carla_params.yaml` |
| 수정 내용 | `global_costmap` 섹션에 추가: `rolling_window: true`, `width: 200`, `height: 200`, `resolution: 0.2`, `transform_tolerance: 0.5` |
| 확인 | `ros2 topic echo /global_costmap/costmap --no-arr` → `info.origin.position` 이 로봇 위치 근처로 업데이트되는지 확인 |

---

**⑩ `AttributeError: 'FollowPath_Result' has no attribute 'error_code'` — 노드 크래시**

| 항목 | 내용 |
| :--- | :--- |
| 증상 | 2D Goal Pose 클릭 후 CSV FollowPath 취소 완료 시점에 `follow_path_client` 프로세스가 즉시 종료. 차량이 정지한 채로 멈춤 |
| 에러 | `AttributeError: 'FollowPath_Result' object has no attribute 'error_code'` (`_on_csv_result` 콜백) |
| 원인 | Nav2 Humble 1.1.x 의 `FollowPath` action result 에는 `error_code` 필드가 없음. 취소된 CSV goal 의 result 콜백이 호출될 때 직접 필드 접근으로 AttributeError 발생 → 노드 죽음 |
| 수정 파일 | `dual_filter/follow_path_client.py` |
| 수정 내용 | `_on_csv_result`, `_on_parking_result` 두 곳에서 `result.result.error_code` → `getattr(result.result, 'error_code', 'N/A')` 로 변경 |
| 확인 | 로그에 `[CSV] FollowPath 완료. status=5, error_code=N/A` 출력 후 노드 정상 유지 |

---

### 12.7 nav2_mppi_controller OpenMP 빌드 (연산 과다 근본 해결)

오류 ⑦에서 소개한 "해결 B — 근본 해결"의 구체적인 방법이다. apt로 설치된 기본 nav2_mppi_controller는 OpenMP가 비활성화된 상태로 빌드되어 있어 CPU 코어를 1개만 사용한다. 소스를 수정하여 OpenMP를 활성화하면 24코어(i7-13700HX 기준) 병렬 연산으로 약 10x 속도 향상을 기대할 수 있다.

#### 워크스페이스 구조

```text
~/carla/nav2_ws/
├── src/
│   └── navigation2/
│       └── nav2_mppi_controller/
│           └── CMakeLists.txt    ← 아래와 같이 수정
├── build/
├── install/
└── log/
```

#### CMakeLists.txt 수정 내용

파일: `nav2_ws/src/navigation2/nav2_mppi_controller/CMakeLists.txt`

```cmake
# 수정 전 (기본값)
set(XTENSOR_USE_OPENMP 0)

# 수정 후 — 3가지를 모두 추가/변경해야 한다
add_definitions(-DXTENSOR_USE_OPENMP)    # C++ 전처리기 매크로 직접 정의 (핵심)
set(XTENSOR_USE_OPENMP 1)
find_package(OpenMP REQUIRED)            # OpenMP 패키지 탐색

# foreach 루프 내 target_link_libraries 수정:
target_include_directories(${lib} PUBLIC ${xsimd_INCLUDE_DIRS} ${OpenMP_CXX_INCLUDE_DIRS})
target_link_libraries(${lib} xtensor xtensor::optimize xtensor::use_xsimd OpenMP::OpenMP_CXX)
```

> **`set(XTENSOR_USE_OPENMP 1)` 만으로는 불충분**: 이 값은 CMake 변수로만 존재하며 C++ 전처리기(`#ifdef XTENSOR_USE_OPENMP`)에 전달되지 않는다. 반드시 `add_definitions(-DXTENSOR_USE_OPENMP)`로 C++ 매크로를 직접 정의해야 xtensor의 OpenMP 분기가 활성화된다.

#### 빌드

cbr 별칭에 반영된 빌드 옵션들:

| 옵션 | 이유 |
| :--- | :--- |
| `-DBUILD_TESTING=OFF` | `test_msgs` 의존성 제거 |
| `-DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized"` | `dwb_plugins` 컴파일 경고 억제 |
| `--parallel-workers 4` | 동시 패키지 빌드 수 제한 (OOM 방지) |
| `--packages-ignore nav2_system_tests` | `gazebo_ros_pkgs` 의존성 제거 |

```bash
# nav2_ws 전체 빌드 (최초 또는 의존 패키지 변경 시)
cd ~/carla/nav2_ws
sr && sv
cbr --packages-ignore nav2_system_tests

# nav2_mppi_controller 단독 재빌드 (CMakeLists.txt 수정 후)
rm -rf build/nav2_mppi_controller install/nav2_mppi_controller
cbr --packages-select nav2_mppi_controller --allow-overriding nav2_mppi_controller
```

#### OpenMP 활성화 확인

```bash
# libgomp.so 동적 링크 확인 (출력이 있으면 성공)
ldd ~/carla/nav2_ws/build/nav2_mppi_controller/libmppi_controller.so | grep gomp
# 예상 출력: libgomp.so.1 => /lib/x86_64-linux-gnu/libgomp.so.1

# GOMP_parallel 심볼 확인
nm -D ~/carla/nav2_ws/build/nav2_mppi_controller/libmppi_controller.so | grep GOMP
# 예상 출력: U GOMP_parallel@GOMP_4.0
```

#### 실행 시 OMP 스레드 수 설정

`OMP_NUM_THREADS` 환경변수로 MPPI 연산에 사용할 스레드 수를 제한한다. 전체 코어를 사용하면 다른 ROS 노드(EKF, costmap 등)와 CPU를 두고 경합할 수 있다.

```bash
# controller_server 실행 전 설정 (터미널 7)
export OMP_NUM_THREADS=8    # 24코어 중 8개 할당 (권장 시작값)
ros2 launch dual_filter controller.launch.py
```

| 값 | 적합한 상황 |
| :--- | :--- |
| `4` | 다른 노드가 많이 실행되는 경우, 안정 우선 |
| `8` | 일반 주행 (권장 시작값) |
| `12` | MPPI 계산 성능 우선, GPU 없는 환경 |
| 미설정 | 전체 코어 사용 → 다른 노드와 경합 가능성 |

#### nav2_ws 소싱 (apt 버전 오버라이드)

```bash
# 매 터미널마다 nav2_ws를 mppi_ws보다 먼저 소싱해야 apt 버전이 아닌 OpenMP 버전이 로드됨
source ~/carla/nav2_ws/install/setup.bash
source ~/carla/mppi_ws/install/setup.bash
```

> nav2_ws를 소싱하지 않으면 apt 설치 버전(OpenMP 비활성)이 사용되어 오류 ⑦이 재발한다.

---

## 13. 파라미터 상세 설명 및 튜닝 가이드

> 모든 파라미터 현재값은 `mppi_ws/src/dual_filter/config/nav2_carla_params.yaml` 기준.
> 차량: `vehicle.micro.microlino` (wheelbase 1.47 m, 차폭 1.475 m)

---

### 13.1 controller_server 파라미터

---

#### `controller_frequency` (현재: 10.0 Hz)

MPPI 제어 루프가 1초에 몇 번 실행되는지를 결정하는 가장 근본적인 파라미터다. 이 값은 **반드시 `model_dt = 1 / controller_frequency`와 함께 수정**해야 한다.

**물리적 의미**: `controller_frequency = 10 Hz`이면 매 100 ms마다 MPPI가 batch_size × time_steps 개의 궤적을 샘플링·평가하고 최적 cmd_vel을 출력한다. 100 ms가 MPPI 연산의 전체 예산이다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 20 Hz) | 차량 상태 더 자주 갱신 → 급격한 경로 변화·장애물에 빠르게 반응 | MPPI 연산이 50 ms 예산을 초과하면 "Control loop missed" 경고 발생; 반드시 `batch_size` / `time_steps` 감소와 병행 |
| **낮추면** (예: 5 Hz) | 연산 예산 200 ms → batch_size·time_steps을 더 크게 설정 가능, 또는 OpenMP 없는 환경에서 안정 실행 | 저속 반응 → 급커브·장애물에 늦게 반응; 느린 차량(< 2 m/s)에만 적합 |

> **판단 기준**: `ros2 topic hz /cmd_vel`로 실제 출력 주파수 확인. 설정값의 80% 이하이면 연산 과부하 → `batch_size` 줄이거나 주파수 낮춤.

---

#### `costmap_update_timeout` (현재: 0.30 s)

controller_server가 costmap 갱신을 이 시간 안에 받지 못하면 제어를 실패 처리한다. CARLA sim_time 기반에서 LiDAR 콜백 지연이 발생할 수 있으므로 여유를 충분히 둬야 한다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** | 일시적 LiDAR 지연에도 제어 지속 | costmap이 오래된 상태로 사용될 수 있음 |
| **낮추면** | 센서 장애를 빠르게 감지해 실패 처리 | 정상 주행 중에도 잦은 타임아웃 발생 위험 |

---

#### `failure_tolerance` (현재: 1.5 s)

유효한 cmd_vel을 이 시간 동안 생성하지 못하면 FollowPath action 실패로 처리한다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** | 일시적 costmap 블랙아웃·연산 스파이크를 더 오래 기다림 | 실제 막힌 상황에서 실패 감지가 느려짐 |
| **낮추면** | 막힌 상황을 빠르게 감지해 recovery behavior 트리거 | 일시적 지연에도 goal abort 발생 |

---

#### `prune_distance` (현재: 5.0 m) — PathHandler

이미 지나친 waypoint를 경로에서 제거하는 기준 거리. 차량의 현재 위치보다 이 거리 이상 뒤에 있는 waypoint는 MPPI에 전달되는 로컬 경로에서 제거된다.

**오실레이션 연결고리**: `prune_distance`가 너무 작으면 위치 추정 오차가 조금만 커져도 아직 지나지 않은 waypoint가 갑자기 pruning된다. 이 순간 경로 참조점이 불연속적으로 앞으로 점프 → PathAlignCritic·PathAngleCritic 비용 스파이크 → 급격한 조향 명령 → 오실레이션.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 10 m) | 지나친 waypoint를 더 오래 유지 → 참조점 점프 완화 | 오래된 waypoint를 계속 참조하면 역방향으로 추종 시도 가능 |
| **낮추면** (예: 2 m) | 빠른 waypoint 갱신 → 최신 경로 앞부분 참조 | 위치 노이즈에 민감 → 참조점 불연속 → 조향 진동 유발 |

---

### 13.2 MPPI Core 파라미터 (FollowPath)

---

#### MPPI 샘플링 구조와 warm-start

MPPI는 매 제어 주기마다 `batch_size`개의 후보 궤적을 병렬 생성한다. 각 후보의 제어 입력은 이전 주기 최적 해(warm-start)에 가우시안 노이즈를 더한 것이다:

```text
u_k[i] = u_optimal[i] + ε[i],   ε[i] ~ N(0, Σ),   Σ = diag(vx_std², wz_std²)

u_optimal : 이전 주기 최적 제어 시퀀스 (warm-start)
ε         : 가우시안 노이즈
```

**`vx_mean`이라는 별도 파라미터는 존재하지 않는다.** 샘플링 분포의 평균은 고정된 값이 아니라 이전 주기의 최적 해(warm-start)가 자동으로 담당한다:

```text
1주기: warm-start ≈ 0 m/s   → 샘플 N(0,   vx_std²) → 최적해 ≈ 0.4 m/s
2주기: warm-start ≈ 0.4 m/s → 샘플 N(0.4, vx_std²) → 최적해 ≈ 0.7 m/s
3주기: warm-start ≈ 0.7 m/s → 샘플 N(0.7, vx_std²) → ...
```

"얼마나 빠른 궤적을 탐색하는가"는 `vx_std`가 결정하고, "얼마나 빠른 궤적을 선택하는가"는 Critic 가중치들이 결정한다.

---

#### `time_steps` (현재: 40) × `model_dt` (현재: 0.10 s)

예측 수평선(prediction horizon) = `time_steps × model_dt`를 결정한다.

* 현재: 40 × 0.10 s = **4.0 s 앞을 예측**
* `vx_max = 5 m/s` 기준 4.0 s 동안 최대 20 m 이동 예측

| 파라미터 | 올리면 | 낮추면 |
| :--- | :--- | :--- |
| `time_steps` ↑ | 더 먼 미래까지 커브·장애물 사전 감지 → 부드러운 감속·회피 | 연산 비용 선형 증가 |
| `time_steps` ↓ | 연산 빨라짐 | 급커브 직전에야 감지 → 늦은 반응·경로 이탈 |
| `model_dt` ↑ | 같은 time_steps에서 더 먼 미래 예측; 연산량 변화 없음 | 한 스텝당 시간 간격이 커져 궤적 이산화 오차 증가; `controller_frequency`와 반드시 일치 |
| `model_dt` ↓ | 더 세밀한 궤적 이산화 → 고속·급커브에서 정확 | 예측 수평선이 짧아짐 → `time_steps` 함께 늘려야 함 |

> **규칙**: `model_dt`는 반드시 `1 / controller_frequency`와 같아야 한다. 불일치 시 MPPI 내부 시간 인덱스와 실제 제어 주기가 어긋나 속도 오차 발생.

---

#### 속도 상향 시 연동 확인 — 예측 거리·제동 거리·costmap

예측 수평선은 **시간** 단위다. `time_steps × model_dt = 4.0 s`는 속도와 무관하게 고정된다. 그러나 물리적 예측 거리(`vx × 4.0 s`)는 속도에 비례해 자동으로 늘어난다.

```text
예측 거리 = vx × time_steps × model_dt

vx = 1 m/s →  1 × 40 × 0.1 =  4 m
vx = 3 m/s →  3 × 40 × 0.1 = 12 m
vx = 5 m/s →  5 × 40 × 0.1 = 20 m  (현재 설정)
vx = 8 m/s →  8 × 40 × 0.1 = 32 m
```

→ **`time_steps` 자체를 올릴 필요는 없는 경우가 많다.** 단, 아래 두 조건을 반드시 확인해야 한다.

##### 조건 1 — 제동 거리 < 예측 거리

```text
제동 거리 = vx² / (2 × |ax_min|) = vx² / 6.0   (ax_min = −3.0 m/s² 기준)

vx = 5 m/s →  25 / 6 =  4.2 m  (예측 거리 20 m의 21%)  ✓
vx = 8 m/s →  64 / 6 = 10.7 m  (예측 거리 32 m의 33%)  ✓
vx = 10 m/s → 100 / 6 = 16.7 m  (예측 거리 40 m의 42%)  ✓
```

현재 `ax_min = -3.0 m/s²` 설정에서는 `vx_max`를 10 m/s까지 올려도 제동 거리 < 예측 거리 조건을 만족한다. 만약 `ax_min`을 완만하게 (예: -1.0 m/s²) 설정하면 `vx=5`에서도 제동 거리가 12.5 m로 커져 `time_steps` 증가가 필요해진다.

##### 조건 2 — costmap 반경 ≥ 예측 거리의 절반

CostCritic은 로컬 costmap 범위 내의 장애물만 평가한다. 현재 costmap은 20 m × 20 m → 차량 전방 최대 **10 m**까지만 장애물 인식 가능하다.

```text
vx = 5 m/s → 예측 거리 20 m → costmap 전방 10 m 커버 → 10~20 m 구간 장애물 미인식  ⚠️
vx = 8 m/s → 예측 거리 32 m → costmap 전방 10 m 커버 → 10~32 m 구간 장애물 미인식  ✗
```

`vx_max`를 올릴 때는 **costmap `width`/`height`를 함께 키워야** 전방 장애물 인식 범위가 예측 거리를 따라간다.

| `vx_max` | 필요 costmap 크기 (전방 커버 기준) | `raytrace_max_range` 권장값 |
| :--- | :--- | :--- |
| 5 m/s | 20 m × 20 m (현재 — 한계) | 18 m |
| 8 m/s | 30 m × 30 m | 25 m |
| 10 m/s | 40 m × 40 m | 35 m |

> costmap 크기를 키우면 격자 수가 면적에 비례해 증가(2배 크기 → 4배 격자)하므로 CPU 부하를 반드시 확인한다.

---

#### `batch_size` (현재: 1000)

MPPI가 한 제어 주기에 샘플링하는 후보 궤적의 수.

**물리적 의미**: `batch_size`개(현재 1000)의 서로 다른 노이즈 시퀀스를 생성하고 각각의 비용을 계산해 weighted average로 최적 제어를 구한다. 많을수록 최적 궤적에 가까운 해를 찾지만 연산 시간이 선형으로 늘어난다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 1000) | 더 다양한 궤적 탐색 → 좁은 통로·급커브에서 더 나은 해 발견; 장애물 회피 품질 향상 | 연산 시간 선형 증가 → 연산 예산 초과 시 루프 누락 발생 |
| **낮추면** (예: 200) | 연산 빨라짐 → 고주파 제어 가능 | 탐색 공간 축소 → 지역 최적(local optimum)에 빠질 확률 증가; 좁은 통로 통과 실패 |

> **권장 튜닝 순서**: 먼저 `batch_size: 200`으로 실시간 동작을 확인한 뒤, 제어 루프 여유가 있을 때마다 500 → 1000으로 올린다. OpenMP 빌드 후에는 4096까지 사용 가능.

---

#### `open_loop` (현재: false)

- `false` (닫힌루프): 각 time_step마다 실제 차량 odometry를 반영해 롤아웃. CARLA처럼 물리 피드백이 있는 환경에서는 **false 권장**.
- `true` (열린루프): 차량 모델만으로 롤아웃. 빠르지만 실제 차량 거동과 괴리 발생.

---

#### `ax_max`, `ax_min`, `az_max` (가속도 한계)

| 파라미터 | 현재값 | 의미 |
| :--- | :--- | :--- |
| `ax_max` | 2.0 m/s² | MPPI가 생성하는 궤적의 최대 전방 가속도 |
| `ax_min` | -3.0 m/s² | 최대 감속도 (브레이크 기준) |
| `az_max` | 1.5 rad/s² | 최대 yaw 가속도 (급격한 조향 변화 제한) |

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| `ax_max` **올리면** | 더 공격적인 가속 궤적 허용 → 빠른 속도 도달 | 실제 차량이 따라가지 못하면 cmd_vel 추종 오차 증가 |
| `ax_min` **내리면** (더 음수) | 더 강한 제동 궤적 허용 → 장애물 앞에서 빠른 감속 | 급제동이 샘플 궤적에 포함되어 비용이 낮아지면 실제 차량이 급정거 |
| `az_max` **올리면** | MPPI가 더 급격한 조향 변화 궤적 허용 → 좁은 커브에서 유리 | 고속에서 과도한 조향 변화 → 조향 진동 가능성 증가 |

---

#### `vx_std` (현재: 0.3 m/s) ← **속도 탐색의 핵심**

각 후보 궤적의 속도 제어 입력에 추가하는 가우시안 노이즈의 표준편차.

**물리적 의미**: warm-start 값 주변 ±`vx_std` 범위에서 새 속도 궤적을 샘플링한다. MPPI가 처음 수렴하면 warm-start ≈ 저속이 될 수 있는데, 이 상태에서 `vx_std`가 작으면 고속 궤적을 탐색하지 못해 저속에 갇힌다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 1.0) | 더 넓은 속도 범위 탐색 → 저속 고착 탈출 가능; MPPI가 0~`vx_max` 전체를 탐색 | 속도 불안정 증가 (급가속·급감속 궤적 혼재) |
| **낮추면** (예: 0.3) | 현재 속도 근처만 탐색 → 안정적이지만 고속 도달 불가; 0.7 m/s 고착 문제의 원인 | 경로 앞에 고속 구간이 있어도 탐색 불가 |

> **현재 문제**: `vx_std: 0.3`이면 warm-start ~0.7 m/s ± 0.3 = 0.4~1.0 m/s만 탐색. PathFollowCritic 보상이 고속 궤적에서 더 크더라도 탐색 공간 밖이라 수렴 불가 → 0.5로 먼저 올려 테스트 권장.

---

#### `wz_std` (현재: 0.3 rad/s) ← **조향 진동의 핵심**

각 후보 궤적의 yaw rate 제어 입력에 추가하는 가우시안 노이즈의 표준편차.

**물리적 의미**: 0.15 rad/s의 노이즈가 추가되면 "왼쪽으로 약간 돌기"와 "오른쪽으로 약간 돌기" 궤적이 생성된다. 고속에서 이 두 종류의 궤적이 비슷한 비용을 가지면 weighted average가 좌우를 왔다갔다 → S자 진동.

**횡방향 이탈 크기**: 이탈 ∝ `vx × wz × Δt²`. 속도가 높을수록 같은 `wz_std`도 더 큰 횡이탈을 유발한다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 0.4) | 더 공격적인 조향 탐색 → 급커브 통과 능력 향상 | 고속에서 S자 오실레이션 발생; 직선 경로에서도 좌우 흔들림 |
| **낮추면** (예: 0.15) | 조향 노이즈 최소화 → 직선·완만한 커브에서 매우 안정적 | 급커브 탐색 불충분 → 좁은 코너에서 경로 이탈 가능 |

> **튜닝 기준**: 직선 경로에서 오실레이션 발생 → `wz_std` 낮춤 (0.3 → 0.2 → 0.15). 커브에서 경로 이탈 → `wz_std` 높임 (0.3 → 0.4). 고속(5 m/s)에서는 0.15~0.2가 안전권.

---

#### `vx_max` / `vx_min` / `wz_max`

| 파라미터 | 현재값 | 의미 |
| :--- | :--- | :--- |
| `vx_max` | 5.0 m/s | MPPI 샘플 궤적의 최대 속도 상한; 실제 차량 안전 최고 속도 |
| `vx_min` | -0.5 m/s | 소폭 후진 허용 (경로 재접근 등); 완전히 막으려면 0.0으로 설정 |
| `wz_max` | 1.5 rad/s | 최대 yaw rate; `vx_max / min_turning_r = 5.0 / 3.3 ≈ 1.5`로 물리적 일관성 유지 |

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| `vx_max` **올리면** | MPPI가 고속 궤적을 탐색·선택 가능 | 실제 차량 물리 한계 초과 시 cmd_vel 추종 실패; 반드시 CARLA에서 실증 후 점진적으로 올림 |
| `wz_max` **올리면** | 더 급격한 회전 허용 → 좁은 커브 통과 | `min_turning_r`과 불일치 시 물리적으로 불가능한 궤적 생성 |

---

#### `iteration_count` (현재: 1)

한 제어 주기 안에서 MPPI 최적화 반복 횟수.

| 값 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| 1 | 연산 예산 아낌; 실시간 필수 환경 표준값 | |
| 2~3 | 같은 batch_size로 더 정밀한 최적 궤적 수렴 | 연산 시간 배증 → 예산 초과 주의 |

---

#### `temperature` (현재: 0.3) ← **속도·오실레이션의 핵심 균형 파라미터**

MPPI softmax 가중치의 날카로움(sharpness)을 조절한다. MPPI 알고리즘의 λ(온도 역수)에 해당.

**물리적 의미**:
- 가중치 ∝ exp(-비용 / temperature)
- temperature → 0: 최저 비용 궤적 하나에 가중치 집중 (greedy)
- temperature → ∞: 모든 궤적 비용 차이 무시, 균일 가중치 (random)

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **낮추면** (예: 0.1) | 극도로 greedy → 최저 비용 궤적 독점 선택; 직선 구간에서 매우 안정적 | 탐색 부족; 지역 최적에 고착될 수 있음 |
| **올리면** (예: 0.7) | 여러 궤적이 골고루 가중치 → 부드러운 blending; 다양한 조건에 유연 | 고속에서 좌/우 오실레이션 궤적도 가중치를 얻어 S자 발생 |

> **오실레이션 관점**: 직선 구간에서 S자 진동이 발생하면 `temperature`를 낮춘다(0.3 → 0.1). 낮아질수록 최저 비용 궤적 하나에 집중해 좌/우 궤적이 가중치를 얻지 못한다. 반대로 좁은 구간에서 MPPI가 경직되어 경로 이탈이 잦아지면 0.5 방향으로 올려 탐색 유연성을 높인다.

---

#### `gamma` (현재: 0.015)

미래 단계 비용에 대한 할인율 (MPPI 논문의 γ). 작을수록 먼 미래 비용을 덜 고려한다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 0.1) | 먼 미래의 목표·장애물을 더 강하게 고려 → 사전 감속·회피 강화 | 너무 크면 먼 미래 불확실성에 과잉 반응 |
| **낮추면** (예: 0.001) | 단기 최적화 우선 → 현재 상태에 집중 | 급커브·장애물 사전 인지 능력 감소 |

---

#### `regenerate_noises` (현재: true)

| 값 | 효과 |
| :--- | :--- |
| `true` | 매 제어 주기마다 새 노이즈 샘플 생성 → 탐색 다양성 최대화; 갑작스러운 장애물 회피에 유리 |
| `false` | 이전 주기 노이즈 재사용 → 계산 절약; 궤적이 더 부드럽게 연속됨 |

---

#### `min_turning_r` (현재: 3.3 m)

Ackermann 운동 모델에서 MPPI 샘플 궤적의 곡률 상한을 결정한다.

**물리적 의미**: 이 값보다 작은 회전반경의 궤적은 물리적으로 불가능하므로 폐기된다. 실제 차량의 최소 회전반경을 CARLA에서 측정 후 입력해야 한다.

| 방향 | 효과 |
| :--- | :--- |
| **올리면** | 완만한 커브만 허용 → 안전하지만 좁은 도로·급커브 통과 불가 |
| **낮추면** | 더 급격한 회전 궤적 허용 → 좁은 코너 통과 가능; 실제 차량 한계보다 낮으면 MPPI가 물리적으로 불가능한 궤적 선택 → 추종 실패 |

> **측정 방법**: CARLA에서 `manual_control.py`로 최대 조향각 고정 후 원 주행, `/odometry/local` pose 기록 후 원의 반경 계산.

---

#### `collision_lookahead_time` (현재: 2.0 s) — TrajectoryValidator

파일: `nav2_carla_params.yaml` → `FollowPath.TrajectoryValidator`

MPPI가 후보 궤적을 최종 선택하기 전에, 이 시간 안에 충돌이 예측되는 궤적을 폐기하는 2차 필터. CostCritic의 비용 기반 평가와 달리 **시간 기준 이진 필터(통과/폐기)**다.

```text
현재: collision_lookahead_time = 2.0 s

vx = 2 m/s → 검사 거리 = 2.0 × 2 =  4 m
vx = 5 m/s → 검사 거리 = 2.0 × 5 = 10 m  ← costmap 반경과 같음
vx = 8 m/s → 검사 거리 = 2.0 × 8 = 16 m  ← costmap 반경(10 m) 초과
```

속도가 높아질수록 검사 거리가 자동으로 늘어난다. 단, costmap 범위(현재 전방 10 m)를 벗어난 검사는 실제로 장애물 정보가 없으므로 의미가 없다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 3.0 s) | 더 먼 미래의 충돌 예측 궤적 조기 폐기 → 고속에서 더 안전 | costmap 범위 내에서만 실효성 있음; costmap 범위 초과 시 효과 없음 |
| **낮추면** (예: 1.0 s) | 가까운 충돌만 필터링 → 더 많은 궤적 생존 → MPPI 탐색 공간 넓어짐 | 고속에서 먼 장애물 대응 능력 저하 |

> `collision_lookahead_time`을 늘리는 것보다 **costmap을 키우는 것**이 우선이다. costmap이 좁으면 이 시간을 아무리 늘려도 장애물 데이터가 없어 효과가 없다.

---

### 13.3 Critic 파라미터

Critics는 각 후보 궤적에 비용을 부여하는 함수다. **`cost_weight`가 높을수록 해당 기준이 최적 궤적 선택에 강하게 반영**된다. 합산 비용이 가장 낮은 궤적이 cmd_vel로 출력된다.

---

#### PathAlignCritic (현재 weight: 14.0) ← **가장 중요한 Critic**

궤적이 전체 경로와 **평행**하게 달릴수록 낮은 비용을 부여한다.

##### `cost_weight` (현재: 14.0)

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 16) | 경로 이탈에 강한 패널티 → 경로 추종 정밀도 향상 | 경로 이탈을 피하기 위해 속도를 낮추는 경향 발생; 장애물 회피 유연성 감소 |
| **낮추면** (예: 8) | 경로 약간 이탈 허용 → 고속 주행 시 자연스러운 라인 선택 | 경로 이탈이 크면 다른 Critic이 보상을 줘도 경로 복귀 못함 |

##### `offset_from_furthest` (현재: 20)

예측 구간 내에서 도달 가능한 가장 먼 waypoint에서 이 숫자만큼 앞 waypoint를 경로 정렬 기준점으로 설정한다.

| 방향 | 효과 |
| :--- | :--- |
| **올리면** (예: 20) | 더 먼 앞 waypoint를 기준 → 커브 사전 감지; 하지만 먼 기준점은 차량 현재 위치와 방향이 달라 기준점 방향 흔들림 유발 → 조향 진동 원인 |
| **낮추면** (예: 5) | 현재 위치 가까운 waypoint 기준 → 방향 안정; 너무 낮으면 이미 지나친 waypoint를 기준삼아 U턴 시도 |

**속도 × waypoint 간격 연동**: `offset_from_furthest`는 waypoint **인덱스** 단위다. 물리적 기준점 거리는 `offset × waypoint_spacing`이므로, 속도가 오르거나 waypoint 간격이 달라지면 같은 `offset_from_furthest` 값이라도 물리적 기준점 위치가 달라진다.

```text
waypoint 간격 d = 0.5 m, vx = 5 m/s 기준:
  4초 동안 40개 waypoint 도달
  furthest = 40번째, offset=20 → 기준점 = 20번째 = 10 m 앞  (적절)

vx = 8 m/s 로 올리면:
  4초 동안 64개 waypoint 도달
  furthest = 64번째, offset=20 → 기준점 = 44번째 = 22 m 앞  (너무 멀어 진동 유발)
  → offset_from_furthest를 30~35로 올려 기준점을 다시 10~15 m 근처로 조정 필요
```

##### `use_path_orientations` (현재: false)

| 값 | 효과 |
| :--- | :--- |
| `true` | 횡거리 이탈 + **heading 이탈** 동시 패널티 → S자 궤적(경로 근처지만 방향이 틀린 궤적)을 직접 억제 |
| `false` | 횡거리 이탈만 패널티 → S자 궤적도 경로 근처에 있으면 낮은 비용 → 오실레이션 발생 가능 |

##### `trajectory_point_step` (현재: 4)

PathAlignCritic이 궤적 비용을 계산할 때 `time_steps` 중 몇 번째 포인트마다 평가할지 결정한다.

현재 설정 기준:

```text
time_steps = 40, trajectory_point_step = 4
→ 4스텝마다 1회 평가 → 총 10개 포인트만 평가
→ 각 평가 간격 = 4 × 0.1 s = 0.4 s
→ 0.4 s 내에서 발생하는 미세 조향 진동은 비용 계산에 포함되지 않음
```

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **낮추면** (예: 2) | 더 세밀한 평가 → 단기 진동 궤적도 높은 비용 부여 → 오실레이션 억제 | 계산량 증가 (step=2이면 평가 횟수 2배) |
| **올리면** (예: 6) | 계산 절약 | 평가 간격 0.6 s → 더 큰 단기 진동 허용 |

---

#### PathFollowCritic (현재 weight: 5.0)

경로상 현재 위치 앞의 waypoint를 향해 이동할수록 낮은 비용을 부여한다. 이 Critic이 속도를 간접적으로 결정한다.

##### `cost_weight` (현재: 5.0)

| 방향 | 효과 |
| :--- | :--- |
| **올리면** (예: 12) | 더 먼 waypoint 도달을 더 강하게 보상 → MPPI가 고속 궤적을 선택하도록 유도; 속도 증가 |
| **낮추면** (예: 4) | 속도 인센티브 감소 → MPPI가 안전·안정 위주로 수렴; 저속화 |

##### `offset_from_furthest` (현재: 5, PathFollowCritic용)

| 방향 | 효과 |
| :--- | :--- |
| **올리면** (예: 8~10) | 더 먼 waypoint를 추적 목표로 설정 → 빠른 진행; 고속 + 긴 직선에 적합 |
| **낮추면** (예: 2~3) | 더 가까운 waypoint 추적 → 느리지만 정밀; 좁은 코너·저속 환경에 적합 |

PathAlignCritic의 `offset_from_furthest`와 마찬가지로, 속도 상향 시 같은 인덱스 값이 더 먼 물리적 거리를 가리키게 된다. 고속(8 m/s 이상)에서는 값을 함께 올려 유효 추적 거리가 너무 멀어지지 않도록 한다.

---

#### PathAngleCritic (현재 weight: 2.0)

경로 방향(waypoint-to-waypoint heading)과 차량 heading을 정렬한다. 급커브 진입 전에 차량을 미리 회전시키는 효과가 있다.

##### `cost_weight` (현재: 2.0)

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 5) | 경로 방향에 강하게 정렬 → 커브 진입 시 heading 오차 최소화 | 직선 구간에서 heading 과민 반응 → 조향 진동 유발 |
| **낮추면** (예: 1) | heading 정렬 약화 → 직선 구간에서 안정적 | 급커브에서 진입 방향 늦게 설정 → 경로 이탈 가능 |

##### `max_angle_to_furthest` (현재: 1.2 rad ≈ 69°)

경로 방향과 현재 heading의 차이가 이 각도를 초과하면 PathAngleCritic 비활성화. 이미 크게 틀어진 상황에서 heading 정렬 페널티가 경로 복귀를 방해하지 않도록 한다.

##### `forward_preference` (현재: true)

PathAngleCritic이 heading을 정렬할 때 전진 방향만 허용할지 여부.

| 값 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| `true` | 경로 heading과 차량 heading의 차이를 전진 방향 기준으로만 계산 → 후진 방향 heading은 높은 비용 | 경로 추종 환경(전진만 필요)에서 올바른 설정 |
| `false` | 전진·후진 heading 모두 허용 → 후진 경로에서 역방향 heading도 낮은 비용 | 전진 전용 경로에서 false로 설정 시, 후진 방향 heading이 우연히 낮은 비용을 얻어 갑작스러운 역방향 조향 명령 → 오실레이션 유발 가능 |

---

#### GoalCritic (현재 weight: 5.0)

예측 수평선 끝(time_steps 번째 pose)이 목표 지점과 가까울수록 낮은 비용.

##### `threshold_to_consider` (현재: 2.0 m)

목표 2.0 m 이내에 들어와야 활성화된다. 멀리 있을 때는 PathFollowCritic이 경로 추종을 담당하므로 GoalCritic이 간섭하지 않도록 이 값은 작게 유지한다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 5.0) | 더 멀리서부터 목표 향해 속도 감소 시작 → 부드럽게 정지 | 목표 지점 앞에서 너무 일찍 감속 |
| **낮추면** (예: 1.0) | 목표 바로 앞에서만 활성화 → 빠른 접근; 마지막 순간 급감속 위험 | |

---

#### GoalAngleCritic (현재 weight: 3.0)

최종 목표 pose의 방향으로 차량 heading을 정렬한다. 주차나 정밀 정지가 필요한 경우 중요하다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** | 목표 heading 정렬 강화 → 정밀 정지 | 경로 추종 중 불필요한 heading 조정 시도 가능 |
| **낮추면** | 도착 방향 무시 → 빠른 접근 우선 | 정밀 주차 불가 |

---

#### ConstraintCritic (현재 weight: 4.0)

속도·조향 범위(`vx_min~vx_max`, `wz_max`)를 벗어나는 궤적에 패널티.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** | 물리 제약 준수 강화; 한계 근처 궤적 완전 억제 | |
| **낮추면** | 한계 근처 궤적도 선택 허용; 고속·급커브에서 유리 | 실제 차량이 따라가지 못할 수 있음 |

---

#### CostCritic (현재 weight: 3.81)

각 후보 궤적이 지나는 costmap 셀 비용을 합산해 장애물 근처 궤적에 패널티.

##### `collision_cost` (현재: 1,000,000)

충돌(LETHAL_OBSTACLE) 셀을 지나는 궤적에 부여하는 페널티. 매우 높게 유지해 충돌 궤적이 절대 선택되지 않도록 한다. 낮추면 MPPI가 충돌 궤적을 선택할 수 있으니 낮추지 말 것.

##### `critical_cost` (현재: 300)

차량 중심이 장애물 외곽(INSCRIBED_INFLATED_OBSTACLE)에 위치하는 궤적의 패널티.

| 방향 | 효과 |
| :--- | :--- |
| **올리면** | 장애물 가장자리에도 접근 거부 → 매우 안전하지만 좁은 통로 통과 불가 |
| **낮추면** | 가장자리 통과 허용 → 좁은 통로 통과 가능; 충돌 여유 감소 |

##### `near_goal_distance` (현재: 1.0 m)

목표 지점 이 거리 이내에서는 CostCritic 비활성화. 목표 근처에 정적 장애물이 있어도 도달 가능하게 한다.

---

#### PreferForwardCritic (현재 weight: 5.0)

후진보다 전진 궤적 선호. 경로 추종 환경에서 후진이 완전히 불필요하면 가중치를 높여 후진 완전 억제 가능.

---

### 13.4 local_costmap 파라미터

---

#### `update_frequency` / `publish_frequency` (현재: 10.0 Hz)

LiDAR 발행 주파수(20 Hz)의 절반으로 설정. `controller_frequency`와 일치시키는 것이 권장.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** | 장애물 감지 주기 단축 → 빠르게 이동하는 장애물에 빠른 반응 | CPU 부하 증가; 20 Hz 이상은 LiDAR 입력 주파수를 초과해 무의미 |
| **낮추면** | CPU 절약 | 장애물 감지 지연 → 고속 이동 장애물 대응 늦어짐 |

---

#### `width` × `height` / `resolution` (현재: 20 m × 20 m / 0.1 m)

- 0.1 m/cell: lidar_2d 포인트 간격(≈0.05 m)의 2배로 충분한 해상도.

**`vx_max`와 연동 — CostCritic 장애물 인식 범위**

CostCritic이 평가할 수 있는 장애물의 최대 전방 거리는 `width / 2`(rolling window 기준 차량 전방)다. `vx_max`를 올릴수록 MPPI의 예측 거리가 늘어나지만 costmap이 그만큼 커지지 않으면 전방 구간에 장애물이 있어도 인식하지 못한다.

```text
현재: width=20 m → CostCritic 전방 최대 인식 = 10 m
예측 거리(vx=5) = 20 m → 10~20 m 구간 장애물은 CostCritic에 미인식

vx_max를 8 m/s로 올리면:
예측 거리 = 32 m, costmap 전방 10 m → 10~32 m 구간 전체 미인식
→ costmap을 30 m × 30 m 이상으로 확대해야 함
```

| `vx_max` | 예측 거리 | 권장 costmap 크기 | 격자 수 (0.1 m/cell) |
| :--- | :--- | :--- | :--- |
| 5 m/s | 20 m | 20 m × 20 m | 200 × 200 = 40,000 |
| 8 m/s | 32 m | 30 m × 30 m | 300 × 300 = 90,000 (+125%) |
| 10 m/s | 40 m | 40 m × 40 m | 400 × 400 = 160,000 (+300%) |

costmap 크기를 2배로 키우면 격자 수는 4배로 늘어난다. OpenMP 빌드 없이 40×40 costmap을 10 Hz로 갱신하면 CPU 부하가 임계치를 넘을 수 있으므로 [Section 10.4.7](#1047-nav2_mppi_controller-openmp-빌드-연산-과다-근본-해결)을 먼저 적용한다.

함께 조정이 필요한 파라미터:

| 파라미터 | 현재값 | 설명 |
| :--- | :--- | :--- |
| `raytrace_max_range` | 18.0 m | LiDAR 레이캐스팅 최대 거리. costmap 절반 크기와 맞춤 |
| `obstacle_max_range` | 15.0 m | 장애물 마킹 최대 거리. `raytrace_max_range`보다 작게 유지 |

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| 크기 **키우면** | 고속 예측 거리만큼 장애물 인식 확보 | 격자 수 제곱 증가 → CPU 부하 급증 |
| 해상도 **높이면** (예: 0.05 m) | 장애물 위치 정밀도 향상 | 격자 수 4배 증가 → 부하 매우 큼 |

---

#### `inflation_radius` (현재: 1.5 m) / `cost_scaling_factor` (현재: 3.0)

**`inflation_radius`**: 장애물 격자에서 이 반경 이내까지 비용을 팽창시켜 MPPI에 안전 여유 공간을 부여한다.

- 계산 근거: 차량 폭(1.475 m) / 2 + 안전 여유(0.76 m) ≈ 1.5 m

| 방향 | 효과 |
| :--- | :--- |
| **올리면** | MPPI가 장애물을 더 넓게 회피 → 안전하지만 좁은 통로 통과 불가 |
| **낮추면** | 좁은 통로 통과 가능; 충돌 여유 감소 |

**`cost_scaling_factor`**: 거리에 따른 비용 감소 비율 (지수 감쇠).

| 방향 | 효과 |
| :--- | :--- |
| **올리면** (예: 5.0) | 장애물 바로 가까이서만 높은 비용 → 좁은 통로에서 중심선 통과 가능 |
| **낮추면** (예: 1.5) | 멀리서부터 완만하게 비용 증가 → MPPI가 장애물로부터 더 넓은 여유 확보 |

---

#### `raytrace_max_range` / `obstacle_max_range` (현재: 18.0 m / 15.0 m)

| 파라미터 | 의미 |
| :--- | :--- |
| `raytrace_max_range` | 이 거리 이상의 격자는 레이캐스팅으로 "비어있음"으로 처리 (lidar range보다 2~5 m 작게 유지) |
| `obstacle_max_range` | 이 거리 이상의 포인트는 장애물로 마킹하지 않음 |

> **lidar range와 연동**: `lidar_2d.range`를 변경하면 두 값도 함께 조정. 권장: `raytrace_max_range ≈ lidar_range - 2`, `obstacle_max_range ≈ lidar_range - 5`.

---

#### ObstacleLayer 갱신 방식 — 실시간 반영 vs. 잔류

파일: `mppi_ws/src/dual_filter/config/nav2_carla_params.yaml`
경로: `local_costmap.local_costmap.ros__parameters.obstacle_layer.lidar_2d` (global_costmap 동일 구조)

관련 파라미터:

| 파라미터 | 현재값 | 역할 |
| :--- | :---: | :--- |
| `marking` | `true` | LiDAR 포인트 위치를 장애물로 기록 |
| `clearing` | `true` | 센서→포인트 광선 경로를 빈 공간으로 클리어 |
| `raytrace_max_range` | 18.0 m | 레이캐스팅이 적용되는 최대 거리 |
| `obstacle_max_range` | 15.0 m | 장애물로 마킹하는 최대 거리 |

ObstacleLayer는 매 LiDAR 스캔마다 두 작업을 동시에 수행한다.

**① marking** — LiDAR 포인트가 찍힌 셀을 `LETHAL_OBSTACLE`로 마킹한다.

**② clearing (레이캐스팅)** — 센서 원점에서 각 포인트까지 광선을 쏘아, 광선이 지나간 경로의 셀을 `FREE_SPACE`로 지운다.

| 상황 | 동작 |
| :--- | :--- |
| 현재 LiDAR FOV 안의 장애물 | 매 스캔마다 실시간 갱신 (marking + clearing 동시 적용) |
| LiDAR FOV 밖으로 사라진 장애물 | **costmap에 그대로 남음** — 레이캐스팅이 닿지 않으므로 이전 마킹이 유지됨 |
| `rolling_window` 범위를 벗어난 셀 | 창이 이동하면서 자동 소멸 |

현재 설정(`raytrace_max_range: 18.0 m`)에서 차량 전방 18 m 이내는 레이캐스팅으로 적극적으로 클리어된다. 그 너머이거나 차량 뒤쪽은 이전 스캔의 장애물 마킹이 잔류한다.

이 방식이 의도적인 이유는, 장애물이 순간 FOV를 벗어났다고 해서 costmap에서 즉시 사라지면 MPPI가 실제로 존재하는 물체 쪽으로 경로를 계획하는 문제가 생기기 때문이다. rolling_window가 이동하면서 잔류 마킹이 점차 자연스럽게 소멸된다.

---

#### `rolling_window` — 격자 원점을 차량에 고정할지 월드에 고정할지

`rolling_window`는 costmap 격자의 **원점을 차량을 따라 이동시킬지(`true`), 월드의 한 점에 고정할지(`false`)** 를 결정하는 플래그다. costmap 장애물의 "기억 지속성"을 좌우하는 1차 변수다.

경로: `local_costmap.local_costmap.ros__parameters.rolling_window` (global_costmap 동일 위치)

**`rolling_window: true` (현재 local_costmap, global_costmap 모두):**

격자가 차량을 중심으로 따라다니는 "창문"처럼 동작한다.

```text
                    width(20m)
        ┌───────────────────────┐
        │                       │
 height │          🚗           │   ← 차량은 항상 창 중앙 근처
 (20m)  │       (항상 중앙)      │
        │                       │
        └───────────────────────┘
   창 전체가 차량과 함께 미끄러져 이동(rolling)
```

* 격자는 항상 차량 주변 `width × height` 만 표현한다. 차량이 1 m 이동하면 창도 1 m 따라간다.
* **창 경계 밖으로 나간 영역의 셀 값은 폐기된다.** 차량이 어떤 지점을 지나 `width/2` 이상 멀어지면, 그 지점에 마킹했던 장애물 셀은 창 밖으로 나가며 메모리에서 사라진다.
* 다시 그 지점으로 돌아오면 창에 새로 들어온 셀은 초기값(unknown/free)으로 리셋되어 있다 — **과거 기억이 없다.**
* 장점: 격자 크기가 고정(20×20 m / 0.1 m → 40,000 셀)이라 CPU·메모리가 일정하고 무한 누적이 없다. local costmap의 표준 설정.

**`rolling_window: false`:**

격자 원점이 월드(`global_frame`)의 고정점에 앵커된다.

* 차량이 이동해도 격자는 움직이지 않는다. 한 번 마킹한 장애물은 차량이 멀어져도 그 셀에 그대로 남는다 → **지속성 확보.**
* 격자가 주행 영역 전체를 덮어야 하므로 셀 수 = 면적 / 해상도² 로 커진다. 크기·원점을 명시하지 않으면 Nav2 기본값(5 m × 5 m 고정 격자, 원점 UTM (0,0) 앵커)에 걸려 로봇이 맵 밖으로 나가는 오류가 발생한다(Section 12.5 의 "⑨ `global_costmap` 범위 오류" 항목 참고).

| 항목 | `rolling_window: true` | `rolling_window: false` |
| :--- | :--- | :--- |
| 격자 원점 | 차량 따라 이동 | 월드 고정점 앵커 |
| 창 밖 셀 | 자동 폐기(소멸) | 해당 없음(전체 유지) |
| 셀 수 | 작고 일정 | 주행 영역 전체 = 큼 |
| 장애물 기억 지속성 | **없음** (떠나면 잊음) | **있음** |
| 용도 | 표준 local 제어 | 맵 단위 기억 / 정적 지도 |

---

#### 장애물 셀이 정확히 언제 지워지는가 (clearing 조건)

마킹된 장애물 셀을 지우는 메커니즘은 **두 가지뿐**이다. 둘 중 하나라도 발동하면 셀이 사라진다.

| 지우개 | 발동 조건 | 창 내부 셀도 지우는가 |
| :--- | :--- | :--- |
| **① rolling_window 이동** | 셀이 창 **경계 밖**으로 벗어남 | ❌ (창 안이면 해당 없음) |
| **② clearing (레이캐스팅)** | 센서→측정점 광선이 그 셀을 **통과** | ✅ **창 내부라도 지운다** |

따라서 "**rolling_window 창 안에 있으면 한 번 감지된 장애물이 영구히 기억되는가?**"의 답은 **조건부 '아니오'** 다. 창 안에 머무는 동안에도 ② clearing 광선이 그 셀을 통과하면 즉시 `FREE_SPACE`로 지워진다. 마킹이 유지되는 것은 **clearing 광선이 닿지 않는 동안에만** 성립한다.

```text
센서(LiDAR) ●━━━━━━━━━━━━▶ ◇측정점
              이 광선이 지나간 셀들 = FREE_SPACE 로 클리어
```

마킹된 셀의 운명은 그 셀로 광선이 지나가느냐에 달렸다.

| 셀 상황 | clearing 광선 | 결과 |
| :--- | :--- | :--- |
| 장애물이 그대로 있고 가림(occlusion)/사각으로 광선이 못 닿음 | 닿지 않음 | **마킹 유지(잔류)** ✅ |
| 장애물이 FOV 밖 또는 `raytrace_max_range`(18 m) 초과 | 닿지 않음 | **마킹 유지(잔류)** ✅ |
| 장애물이 치워졌고 그 자리로 광선이 통과 | 통과 | **FREE 로 클리어** ❌ |
| 장애물은 있으나 각도 변화로 광선이 셀을 비껴 통과 | 통과 | **FREE 로 클리어** ❌ (오클리어) |

**2D LiDAR의 한계:** `lidar_2d`는 단일 평면만 스캔하므로, 차량 자세·각도가 바뀌면 실제로 존재하는 장애물을 비껴가는 광선이 생기기 쉽다. 이 경우 ②에 의해 실재 장애물이 클리어될 수 있어, 창 내부라도 지속성이 불안정하다.

---

#### 지속성이 필요한 경우 (예: 주차 진입↔탈출)

장애물을 **진입 시 감지했지만 탈출 시에는 못 보는** 상황(2D LiDAR 사각/가림/평면 이탈)에서도 안전 주행을 위해 costmap에 장애물을 유지해야 할 때가 있다. 현재 기본 구성(`rolling_window: true` + `clearing: true`)은 위 ①·② 두 메커니즘 때문에 이 지속성을 보장하지 못한다.

지속성을 확보하는 선택지:

| 방법 | 효과 | 주의 |
| :--- | :--- | :--- |
| `clearing: false` 로 변경 | 창 내부에서 절대 클리어 안 됨 | ① 창 밖 소멸은 여전; 동적 장애물·센서 노이즈가 영구 잔류하는 **유령(ghost) 장애물** 위험 |
| 주차 구역을 덮는 **non-rolling 보조 costmap** | ① 소멸까지 차단, 구역 내 영구 기억 | 구역 면적만큼 셀 수 증가 |
| 알려진 정적 장애물을 **StaticLayer** 로 분리 | 감지 여부와 무관하게 항상 존재; clearing 대상 아님 | 사전에 장애물 위치(맵 좌표)를 알아야 함 |

정적 장애물(주차 콘 등)은 clearing이 오히려 방해가 되므로, **실시간 LiDAR 레이어(`clearing: true`)와 정적 장애물 레이어(StaticLayer, 클리어 안 함)를 분리**하는 것이 가장 안전하다. 동적 장애물 대응은 유지하면서 정적 장애물 기억을 영구 보장할 수 있다.

---

### 13.5 현재 설정의 종합적 의도

현재 파라미터 설정은 다음 세 가지 목표의 균형을 노린다:

1. **고속 주행 (2~5 m/s)**: `vx_std: 0.7`로 넓은 속도 탐색, `PathFollowCritic weight: 8`로 강한 전진 인센티브
2. **직선 안정성 (오실레이션 없음)**: `wz_std: 0.15`로 조향 노이즈 억제, `temperature: 0.3`으로 greedy 선택, `use_path_orientations: true`로 heading 이탈 직접 패널티
3. **연산 실시간성**: `controller_frequency: 10 Hz`, `batch_size: 500`으로 100 ms 예산 내 수렴

**속도가 다시 느려지면**: `PathFollowCritic weight: 5 → 8` 또는 `vx_std: 0.3 → 0.5`  
**오실레이션이 재발하면**: `wz_std` 낮춤 또는 `temperature: 0.3 → 0.2`  
**커브에서 이탈하면**: `wz_std` 높임 또는 `PathAngleCritic weight: 2.0 → 3.0`

---

**속도 저하 시 권장 튜닝 순서:**

| 단계 | 파라미터 | 현재값 | 권장값 | 목적 |
| :--- | :--- | :---: | :---: | :--- |
| 1 | `vx_std` | 0.3 m/s | 0.5 m/s | 탐색 폭 확대 (저속 고착 탈출) |
| 2 | `PathFollowCritic.cost_weight` | 5.0 | 8.0 | 전진 인센티브 강화 |
| 3 | `PathFollowCritic.offset_from_furthest` | 5 | 7 | 더 먼 waypoint 추적 |
| 4 | `_KP_SPEED` | 0.8 | 1.2 | 스로틀 추종 속도 향상 |

> 각 단계 후 `ros2 topic hz /cmd_vel`과 `/odometry/local` 속도를 반드시 확인하고 안정적일 때만 다음 단계로 진행한다.

---

### 13.6 `_KP_SPEED` — 스로틀 비례 제어 게인

파일: `mppi_ws/src/dual_filter/dual_filter/cmd_vel_to_carla.py` (모듈 상수, 현재값: 0.8)

MPPI가 `target_vx`를 출력하면 현재 CARLA 차량 속도와의 오차에 게인을 곱해 스로틀을 결정한다:

```python
err = target_vx - current_vx
if err > 0.0:
    throttle = min(_KP_SPEED * err, 1.0)
    brake = 0.0
else:
    throttle = 0.0
    brake = min(-_KP_SPEED * err, 1.0)
```

**P 제어기의 구조적 한계 (정상 상태 오차):**

```text
차량 등속 유지에 필요한 스로틀 = T_steady (마찰·공기 저항 보상)

err → 0 이면 throttle → 0  →  T_steady 공급 불가
→ 실제 속도는 target_vx보다 항상 약간 낮게 수렴

예: target_vx = 0.7 m/s, current_vx = 0.5 m/s
    throttle = 0.8 × 0.2 = 0.16  (매우 약한 가속 → 속도 정체)

    target_vx = 2.0 m/s, current_vx = 0.3 m/s
    throttle = 0.8 × 1.7 = 1.36 → 클리핑 → 1.0  (풀 스로틀, 문제 없음)
```

저속 크리핑 구간(err가 작을 때)에서 스로틀이 약해 속도 정체가 발생한다.  
`vx_std`를 올려 MPPI가 높은 속도를 명령하면 err가 커져 이 문제가 자연스럽게 해소된다.

| 방향 | 효과 | 주의사항 |
| :--- | :--- | :--- |
| **올리면** (예: 1.2) | 오차 추종 빨라짐 → 속도 지연 감소 | 너무 크면 오버슈트 → 속도 헌팅(진동) 발생 |
| **낮추면** (예: 0.5) | 부드러운 가속 | 목표 속도 도달 느려짐 |

> CARLA 시뮬레이션 수준에서는 1.2~1.5가 안정적인 상한이다. 정상 상태 오차를 완전히 제거하려면 적분 항을 추가한 PID 제어기가 필요하다.

---

#### `--wheelbase` (현재: 1.47 m) — 조향각 계산 오차

조향각 계산식: `δ = atan2(-wz × L, vx)` (자전거 모델 역변환, L = wheelbase)

L이 실제 차량 축간거리와 다르면 MPPI가 명령한 `wz`와 실제 차량의 조향각이 불일치한다:

```text
실제 L = 1.47 m, 설정 L = 1.20 m (과소 설정):
  wz = 0.5 rad/s, vx = 2.0 m/s
  → δ_설정 = atan2(-0.5 × 1.20, 2.0) = atan2(-0.6, 2.0) = -0.29 rad  (실제보다 작은 조향)
  → 실제 곡률 부족 → 경로 외측 이탈 → PathAlignCritic 비용 증가
  → 다음 주기 더 큰 wz 명령 → 오버슈트 → 오실레이션

실제 L = 1.47 m, 설정 L = 2.00 m (과대 설정):
  → δ_설정 이 실제보다 큰 조향각 → 과도한 선회 → 반대편 이탈 → 오실레이션
```

**측정 방법**: CARLA `physics_control.wheels`의 `position` 필드로 전·후륜 좌표 차이를 직접 계산하거나, 직선 주행 후 `ros2 topic echo /carla/<vehicle>/wheel_info`에서 확인.

| 오차 방향 | 증상 |
| :--- | :--- |
| L **과소** 설정 | 직선 경로에서 지속적으로 외측 이탈 → 복귀 시도 → 蛇行 |
| L **과대** 설정 | 커브에서 과도한 선회 후 반대편 이탈 → 큰 진폭 오실레이션 |

---

#### steer 출력 필터링 부재 — 오실레이션 직접 전달 경로

`cmd_vel_to_carla.py`는 MPPI의 `wz`를 `δ = atan2(-wz × L, vx)`로 변환한 뒤 **필터링 없이 즉시 CARLA `steer`로 적용**한다. MPPI가 매 제어 주기(100 ms)마다 `wz`를 출력할 때 주기 간 진동이 있으면 이 진동이 조향 액추에이터에 그대로 전달된다:

```text
MPPI 출력 wz 시계열:  +0.3 → -0.3 → +0.3 → -0.3  (100 ms 간격)
→ steer 시계열:        +X°  → -X°  → +X°  → -X°   (100 ms 간격)
→ 차량이 좌우로 흔들리는 물리적 오실레이션 발생
```

현재 코드에는 steer 출력에 대한 스무딩이 **없다**. 지수 이동 평균(EMA) 필터를 추가하면 MPPI 수준의 진동을 물리 조향으로 전달되기 전에 감쇠할 수 있다:

```python
# cmd_vel_to_carla.py 에 추가할 수 있는 EMA 필터 (현재 미구현)
_EMA_ALPHA = 0.5   # 0 에 가까울수록 강한 필터; 1이면 필터 없음

# __init__ 에서: self._smooth_steer = 0.0
# _cmd_vel_cb 에서 steer 계산 후:
self._smooth_steer = _EMA_ALPHA * steer + (1 - _EMA_ALPHA) * self._smooth_steer
ctrl.steer = float(self._smooth_steer)
```

| `_EMA_ALPHA` | 효과 | 주의사항 |
| :--- | :--- | :--- |
| 0.3 | 강한 스무딩 → 진동 크게 감쇠 | 조향 응답이 느려져 급커브 진입 지연 |
| 0.5 | 중간 스무딩 → 1 스텝 지연으로 진폭 절반 | 커브 추종 능력과 안정성 절충 |
| 0.7 | 약한 스무딩 | MPPI 진동이 일부 통과 |
| 1.0 | 필터 없음 (현재 상태) | MPPI 진동 100% 전달 |

> `wz_std` 감소·`temperature` 감소로도 MPPI 수준의 진동을 줄일 수 있지만, EMA 필터는 물리 레벨의 마지막 방어선이다. MPPI 파라미터 튜닝 후에도 미세한 물리 진동이 남는다면 `_EMA_ALPHA = 0.5` 적용을 권장한다.

---

### 13.7 `ParkingPath` — 주차 전용 MPPI 플러그인

파일: `mppi_ws/src/dual_filter/config/nav2_carla_params.yaml`

`FollowPath`와 별도로 정의된 MPPI 플러그인으로, **RViz "2D Goal Pose" 주차 시에만** `controller_id='ParkingPath'`로 지정되어 사용된다. `controller_server`는 두 플러그인을 동시에 activate하고, `FollowPath.Goal.controller_id` 필드로 어느 플러그인을 사용할지 동적으로 선택한다.

#### 설계 목적

CSV 추종 중에는 전진이 주이지만, 주차 모드에서는 SmacPlannerHybrid(REEDS_SHEPP)가 전진·후진 혼합 최적 경로를 계획하므로 MPPI도 두 방향 모두 허용해야 한다. 또한 CSV 추종에서 불필요한 후진을 억제하는 `PreferForwardCritic`을 주차에서 유지하면 MPPI가 후진 구간에서 정지 궤적을 선택하는 문제가 생긴다. MPPI 파라미터를 런타임에 동적으로 변경하면 (`SetParameters` 서비스 호출) MPPI가 configure 시점에만 파라미터를 읽어 반영되지 않으므로, 주차 전용으로 별도 플러그인을 정의하는 방식이 가장 안정적이다.

#### `FollowPath` vs `ParkingPath` 파라미터 비교

| 파라미터 | FollowPath | ParkingPath | 변경 이유 |
| :--- | :---: | :---: | :--- |
| `vx_max` | 5.0 m/s | **2.0 m/s** | SmacPlannerHybrid REEDS_SHEPP 경로의 전진·후진 구간 모두 추종 가능 |
| `vx_min` | −2.0 m/s | −2.0 m/s | 최대 후진 속도 동일 |
| `goal_checker_id` | `goal_checker` | **`parking_goal_checker`** | 주차 정밀 판정용 전용 checker 사용 |
| `GoalCritic.cost_weight` | 5.0 | **10.0** | 목표 위치 끌어당기는 힘 강화 |
| `GoalCritic.threshold_to_consider` | 2.0 m | **3.0 m** | 더 멀리서부터 목표 수렴 유도 |
| `GoalAngleCritic.cost_weight` | 3.0 | **8.0** | 화살표 heading 정렬 강도 대폭 강화 — 핵심 파라미터 |
| `GoalAngleCritic.threshold_to_consider` | 1.0 m | **2.5 m** | 1 m 이내는 너무 늦음 — 2.5 m 이전부터 heading 교정 시작 |
| `PathAlignCritic.use_path_orientations` | false | **true** | REEDS_SHEPP 경로 waypoint orientation 도 정렬 평가에 반영 |
| `PathAngleCritic.cost_weight` | 2.0 | **4.0** | 경로 방향 추종 강화 |
| `PreferForwardCritic` | 포함 (weight 5.0) | **제거** | 후진 패널티 없애야 MPPI가 후진 선택 가능 |
| `PathAngleCritic.forward_preference` | true | **false** | 후진 경로 방향 기준 heading 정렬 허용 |

#### `PreferForwardCritic` 제거 이유

`PreferForwardCritic`은 후진 궤적(`vx < 0`)에 `cost_weight × |vx|` 패널티를 부여한다. `FollowPath`에서는 CSV 경로 추종 중 불필요한 후진을 억제하는 역할을 한다. `ParkingPath`에서 이를 그대로 유지하면, SmacPlannerHybrid가 계산한 후진 경로를 MPPI가 따르려 해도 `PreferForwardCritic`이 모든 후진 궤적에 패널티를 부여해 결국 vx ≈ 0 (정지) 궤적이 최적으로 선택되는 문제가 생긴다.

#### 후진 주차 전체 흐름

```text
RViz "2D Goal Pose" 클릭 (G키 단축키)
  ↓ /goal_pose (PoseStamped) 발행
    frame_id = RViz Fixed Frame (utm 또는 odom 권장)
    position = 후륜축이 도달해야 할 좌표
    orientation = 주차 완료 시 차량 전면 방향

mode_manager._goal_pose_cb()
  ↓ 현재 CSV FollowPath 취소
  ↓ _start_parking(goal_pose)
    planner_server.server_is_ready() 확인
    ComputePathToPose {planner_id='GridBased'} 전송
      → SmacPlannerHybrid (REEDS_SHEPP) 경로 계산
      → Reeds-Shepp 곡선: 전진·후진 혼합 또는 순수 후진 경로 반환

  ↓ _on_plan_result(path)
    _send_follow_path(path, controller_id='ParkingPath',
                           goal_checker_id='parking_goal_checker')
      → controller_server: ParkingPath 플러그인 실행
        vx_max=2.0, vx_min=−2.0 → 전진·후진 혼합 궤적 평가 (경로에 따라 자동 선택)
        GoalCritic (w=10, 3 m 이내 활성): 정확한 위치로 강하게 끌어당김
        GoalAngleCritic (w=8, 2.5 m 이내 활성): 화살표 방향으로 heading 교정
        PathAlignCritic (use_path_orientations=true): 경로 yaw 방향 추종
        PathFollowCritic: 경로 앞 waypoint를 후진으로 추적
        CostCritic: 후진 경로상 장애물 회피

  ↓ parking_goal_checker 성공 판정  (CSV 추종은 goal_checker 사용)
    base_link ↔ 목표 거리 ≤ 0.25 m
    yaw 오차 ≤ 0.1 rad (≈6°)  ← 화살표 직선 위 정렬 강제

  ↓ _on_parking_result()
    _resume_csv() → IDLE → CSV_FOLLOWING 자동 복귀
```

#### 주차 실패 시 확인 사항

| 증상 | 원인 | 확인 방법 |
| :--- | :--- | :--- |
| 주차 경로 계산 실패 (빈 경로) | global_costmap 미초기화 또는 목표가 LETHAL 셀 내부 | `ros2 topic echo /global_costmap/costmap` |
| 차량 정지 후 미동 없음 | `cmd_vel_to_carla` 후진 처리 누락 (`ctrl.reverse=False`) | `ros2 topic echo /cmd_vel` 으로 vx < 0 확인 |
| 주차 후 복귀 안 됨 | `_on_parking_result` 미호출 (action 응답 없음) | `ros2 topic echo /mode_status` 로 PARKING 지속 확인 |
| RViz Fixed Frame ≠ utm | goal_pose TF 변환 실패 → planner 즉시 실패 | `/goal_pose` echo의 `frame_id` 확인 |

---

## 14. TF 시간적 허용 오차 (TF Temporal Tolerance)

### 14.1 개념

ROS 2의 TF 시스템(`tf2_ros::Buffer`)은 노드가 발행한 좌표 변환(TF)을 타임스탬프 순으로 **링 버퍼**에 저장한다. 어떤 노드가 `lookupTransform(target_frame, source_frame, t)` 를 호출하면, tf2는 버퍼 안에서 요청 시각 `t` 에 가장 가까운 두 TF를 찾아 선형 보간한다.

**문제가 발생하는 경우:**

```text
현재 시각 t_now = 100.00 s
tf 버퍼 최신 타임스탬프 = 99.95 s  (50 ms 지연)

lookupTransform(odom, utm, t_now) 요청
→ t_now(100.00) > 버퍼 최신(99.95)
→ "미래 시각으로 외삽이 필요함"
→ ExtrapolationException 발생
```

이 오류를 방지하는 파라미터가 **`transform_tolerance`** (또는 노드에 따라 `transform_timeout`)이다.

**동작 원리:**

```text
transform_tolerance = 0.5 s 설정 시

노드가 TF를 요청할 때 내부적으로:
  요청 시각 = t_now - ε  (혹은 허용 오차 범위 안에서 재시도)

tolerance 기간 안에 원하는 TF가 도착하면 → 정상 변환
tolerance를 초과해도 도착하지 않으면 → 에러 또는 경고 후 포기

실질적 의미:
  "최신 TF가 이 시간(0.5 s) 이상 오래되지 않았으면 허용한다"
```

**CARLA 시뮬레이션 환경에서 TF 지연이 발생하는 이유:**

```text
CARLA Simulation Time (sim time)
  ↓ /clock 토픽으로 발행
  ↓ global_ekf가 /clock 구독 → utm→odom TF 발행
  ↓ controller_server가 TF 요청

CARLA가 렉(tick 지연)을 겪으면:
  /clock 업데이트 지연 → global_ekf 갱신 지연 → TF 타임스탬프 지연
  → controller_server가 요청한 시각 > 최신 TF 타임스탬프
  → 50 ms 내외의 "미래 외삽" 오류 발생
```

기본값(0.1 s)은 대부분의 로컬 환경에서 충분하지만, CARLA처럼 sim time 기반 + 간헐적 렉이 있는 환경에서는 0.5 s로 올려야 안정적이다.

---

### 14.2 시스템 내 노드별 TF 조회 여부 분석

우리 시스템에서 TF를 실제로 `lookupTransform`으로 조회하는 노드를 **소스 코드 기반**으로 분류한 결과다.

#### 커스텀 노드 (dual_filter 스택) — TF 조회 없음

| 노드 | 파일 | TF 조회 여부 | 근거 |
| :--- | :--- | :---: | :--- |
| `gnss_to_odom` | `dual_filter/gnss_to_odom.py` | ❌ | `tf2_ros` import 없음. UTM→ROS 변환을 수식으로 계산해 `/odometry/gnss` 토픽으로 발행 |
| `follow_path_client` | `dual_filter/follow_path_client.py` | ❌ | `tf2_ros` import 없음. IDLE/CSV_FOLLOWING/PARKING 상태 머신. `/odometry/local` pose로 waypoint를 수학적으로 탐색하고, FollowPath / ComputePathToPose action goal 전송 |
| `cmd_vel_to_carla` | `dual_filter/cmd_vel_to_carla.py` | ❌ | `tf2_ros` import 없음. `/cmd_vel` Twist를 CARLA Python API로 직접 변환 |
| `csv_to_utm` | `gnss_to_utm/src/csv_to_utm.cpp` | ❌ | `tf2_ros` include 없음. `/utm_datum`을 받아 수식으로 utm 프레임 Path 생성 |
| `f9r_to_utm` | `gnss_to_utm/src/f9r_to_utm.cpp` | ❌ | NavSatFix → UTM 수식 변환만 수행 |
| `f9p_to_utm` | `gnss_to_utm/src/f9p_to_utm.cpp` | ❌ | NavSatFix → UTM 수식 변환만 수행 |
| `azimuth_angle_calculator` | `gnss_to_utm/src/azimuth_angle_calculator.cpp` | ❌ | TF 조회 없음. 단, 두 GNSS 메시지 간 타임스탬프 동기화 허용 오차(`max_time_diff_sec: 0.1`)를 자체적으로 가짐 — 이것은 TF tolerance가 아니라 메시지 동기화 기준 |

> **결론: 우리가 직접 작성한 모든 커스텀 노드는 TF 버퍼를 생성하거나 `lookupTransform`을 호출하지 않는다.** TF 트리에 변환을 _발행_하는 것(local_ekf, global_ekf)과 TF를 _조회_하는 것은 다르다.

#### robot_localization (EKF 노드) — 제한적 TF 조회

| 노드 | TF 조회 여부 | 내용 | tolerance 파라미터 | 현재 설정 |
| :--- | :---: | :--- | :--- | :--- |
| `local_ekf` | ⚠️ 조건부 | 센서 `header.frame_id ≠ base_link_frame`이면 TF 조회로 센서 위치 보정. 우리 IMU·wheel odom은 `base_link` 또는 `odom` 프레임으로 발행되므로 실질적 조회 최소화 | `transform_timeout` (default: 0.1 s) | **명시 설정 없음 — 기본값 사용** |
| `global_ekf` | ⚠️ 조건부 | 동일. `/odometry/gnss`(frame: utm), IMU(frame: base_link), wheel odom(frame: odom) — 각 센서 프레임이 이미 설정된 프레임과 일치하면 TF 조회 발생 안 함 | `transform_timeout` (default: 0.1 s) | **명시 설정 없음 — 기본값 사용** |

> robot_localization의 `transform_timeout`은 "센서 프레임→base_link 변환을 기다리는 최대 시간"이다. 우리 센서들이 이미 적절한 프레임으로 발행되고 있어 현재는 문제가 없지만, 센서 프레임이 바뀔 경우 `ekf_params.yaml`에 `transform_timeout: 0.5`를 추가해야 한다.

#### Nav2 노드 — 명시적 TF 조회 (tolerance 필수)

| 노드 | TF 조회 내용 | tolerance 파라미터 | 현재 설정 | 수정 이력 |
| :--- | :--- | :--- | :--- | :--- |
| `controller_server` | `odom → utm` 변환으로 robot_pose를 global plan 프레임으로 변환 | `transform_tolerance` | **0.5 s** ✅ | 기본값 0.1 s → 0.5 s 수정 (주행 중 차량 정지 버그 수정) |
| `local_costmap` | `odom → base_link` 변환으로 로봇 위치를 costmap에서 추적 | `transform_tolerance` | **0.5 s** ✅ | 초기 설정부터 0.5 s |

---

### 14.3 버그 수정 이력

**증상**: 주행 중 차량이 갑자기 멈추며 다음 에러 발생:

```text
[controller_server]: Exception in transformPose: Lookup would require extrapolation
into the future. Requested time 265.118526 but the latest data is at time 265.068526,
when looking up transform from frame [odom] to frame [utm]
[controller_server]: Unable to transform robot pose into global plan's frame
[controller_server]: [follow_path] [ActionServer] Aborting handle.
```

**원인**: `controller_server`의 `transform_tolerance` 기본값(0.1 s)이 CARLA sim time 렉으로 인한 TF 지연(최대 ~50 ms)을 흡수하지 못함. 지연이 간헐적으로 0.1 s를 초과할 때 abort 발생.

**수정**: `nav2_carla_params.yaml`의 `controller_server` 섹션에 `transform_tolerance: 0.5` 추가.

```yaml
controller_server:
  ros__parameters:
    transform_tolerance: 0.5   # controller_server 자체 TF 조회 허용 지연 (s)
                               # local_costmap의 transform_tolerance와 별개 파라미터
```

> **주의**: `local_costmap`의 `transform_tolerance`와 `controller_server`의 `transform_tolerance`는 **별개의 파라미터**다. costmap에만 설정해도 controller_server의 pose 변환 오류는 막을 수 없다.

---

### 14.4 파라미터 상세 (Parameter Reference)

시스템에서 TF 타이밍과 관련된 파라미터 4개를 정리한다.

#### Nav2 — `transform_tolerance`

Nav2의 `transform_tolerance`는 `lookupTransform` 호출 시 **"요청 시각에서 얼마나 오래된 TF까지 허용할 것인가"**를 지정한다.  
내부적으로 `tf2_ros::Buffer::lookupTransform(frame_a, frame_b, t - tolerance)` 형태로 요청 시각을 뒤로 당겨 ExtrapolationException을 회피한다.

| 파라미터 위치 | 파라미터명 | 현재 값 | 적용 대상 |
| :--- | :--- | :---: | :--- |
| `controller_server.ros__parameters` | `transform_tolerance` | **0.5 s** | `odom → utm` 변환 (robot pose를 global plan 프레임으로 변환할 때) |
| `local_costmap.ros__parameters` | `transform_tolerance` | **0.5 s** | `odom → base_link` 변환 (로봇 위치를 costmap 안에서 추적할 때) |

```yaml
# nav2_carla_params.yaml

controller_server:
  ros__parameters:
    transform_tolerance: 0.5      # (s) CARLA sim time 렉으로 인한 최대 50ms TF 지연 흡수

local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 0.5   # (s) 동일 이유; controller_server와 별개 파라미터
```

> `controller_server`와 `local_costmap`은 **같은 프로세스** 안에서 동작하지만, 각각 독립적으로 `transform_tolerance`를 읽는다. 한 곳에만 설정하면 다른 곳은 기본값(0.1 s)이 적용된다.

---

#### robot_localization — `transform_timeout`

`transform_timeout`은 Nav2의 `transform_tolerance`와 이름도 의미도 다르다.  
EKF가 **센서 데이터를 base_link 프레임으로 변환할 때** 해당 TF가 도착하기를 기다리는 **최대 대기 시간**이다. 대기 후에도 도착하지 않으면 해당 센서 업데이트를 건너뛴다.

| 파라미터 위치 | 파라미터명 | 현재 값 | 적용 대상 |
| :--- | :--- | :---: | :--- |
| `local_ekf.ros__parameters` | `transform_timeout` | **0.1 s (기본값)** | 센서 `header.frame_id → base_link` 변환 |
| `global_ekf.ros__parameters` | `transform_timeout` | **0.1 s (기본값)** | 동일 |

```yaml
# ekf_params.yaml — 현재 명시 설정 없음 (기본값 0.1 s 사용)

local_ekf:
  ros__parameters:
    # transform_timeout: 0.1   ← 기본값; 센서 프레임이 바뀌면 추가 필요

global_ekf:
  ros__parameters:
    # transform_timeout: 0.1   ← 기본값; 센서 프레임이 바뀌면 추가 필요
```

**현재 명시 설정이 없어도 문제없는 이유:**

우리 센서 데이터는 이미 EKF가 기대하는 프레임으로 발행되고 있다:

| 센서 토픽 | 발행 프레임 | EKF 기대 프레임 | TF 조회 필요 여부 |
| :--- | :---: | :---: | :---: |
| `/wheel_encoder/data` | `base_link` | `base_link` | ❌ 불필요 |
| `/imu/data` | `base_link` | `base_link` | ❌ 불필요 |
| `/odometry/gnss` | `utm` | `utm` (world_frame) | ❌ 불필요 |

센서 프레임이 이미 일치하므로 EKF가 `lookupTransform`을 호출할 일이 없고, `transform_timeout` 기본값(0.1 s)이 문제를 일으키지 않는다.

**`transform_timeout` 명시 설정이 필요한 경우:**

센서의 `header.frame_id`가 `base_link`가 아닌 다른 프레임(예: `imu_link`, `lidar_link`)으로 바뀌면 EKF가 해당 프레임 → `base_link` TF를 조회하게 된다. 이때 CARLA 렉으로 TF 도착이 늦어지면 센서 업데이트가 누락될 수 있으므로 `ekf_params.yaml`에 다음을 추가한다:

```yaml
local_ekf:
  ros__parameters:
    transform_timeout: 0.5     # s — 센서 프레임이 base_link가 아닐 때만 필요

global_ekf:
  ros__parameters:
    transform_timeout: 0.5     # s — 동일
```

---

#### 파라미터 비교 요약

| 파라미터명 | 소속 패키지 | 의미 | 현재 값 |
| :--- | :--- | :--- | :---: |
| `controller_server` → `transform_tolerance` | Nav2 | TF 지연 허용 시간 (오래된 TF 허용 범위) | 0.5 s ✅ |
| `local_costmap` → `transform_tolerance` | Nav2 | 동일 (costmap 독립 설정) | 0.5 s ✅ |
| `local_ekf` → `transform_timeout` | robot_localization | 센서 프레임 TF 대기 최대 시간 | 0.1 s (기본값) |
| `global_ekf` → `transform_timeout` | robot_localization | 동일 | 0.1 s (기본값) |

> Nav2의 `transform_tolerance`와 robot_localization의 `transform_timeout`은 이름도, 의미도, 작동 방식도 다르다. 혼용하면 파라미터가 무시된다.

---

## 15. 커스텀 맵(RoadRunner) CARLA 적용 매뉴얼 — `Mando1` / `Mando2` / `Mando3` 사례

> **이 섹션의 목적**: MATLAB RoadRunner로 제작한 커스텀 맵(`.fbx` + `.xodr`)을 CARLA에 적용해
> Section 12.1 자율주행/주차 스택에서 사용하는 **전 과정**을 기록한다. 맵을 수정해 다시
> 익스포트했을 때 이 절차만 그대로 반복하면 재적용된다. **에이전트가 이 문서만 읽고 전 과정을
> 정확히 재현할 수 있도록** 모든 명령·경로·예상 출력·함정(gotcha)을 포함한다.
>
> **실제 적용 사례 기준 환경** (2026-06 기준, `Mando1`/`Mando2`/`Mando3` 맵):
> - 입력 맵 소스 (`~/carla/CustomMap/` 하위, basename이 달라 **여러 개를 한 번에 import 가능**):
>   - `/home/hannibal/carla/CustomMap/MandoParking1/` (`Mando1.fbx`, `Mando1.xodr`, `Mando1.rrdata.xml`) → 맵 **`Mando1`**
>   - `/home/hannibal/carla/CustomMap/MandoParking2/` (`Mando2.fbx`, `Mando2.xodr`, `Mando2.rrdata.xml`, `Mando2.geojson`) → 맵 **`Mando2`**. `Mando2.geojson`은 `.rrdata.xml`과 마찬가지로 import에 **사용하지 않음**
>   - `/home/hannibal/carla/CustomMap/MandoParking3/` (`Mando3.fbx`, `Mando3.xodr`, `Mando3.rrdata.xml`, `Mando3.geojson`) → 맵 **`Mando3`**. 레퍼런스 경로 기록을 위해 장애물을 제거한 버전. `.rrdata.xml`·`.geojson`은 import에 **사용하지 않음**
>   - basename(`Mando1`/`Mando2`/`Mando3`)이 곧 CARLA 맵 이름. `--map Mando1` / `--map Mando2` / `--map Mando3`로 선택
> - 적용 결과 맵 이름: **`Mando1`**, **`Mando2`**, **`Mando3`** (셋 다 패키지 **`map_package`** 안에 공존)

---

### 15.1 핵심 원리 — 왜 "소스 빌드"가 반드시 필요한가

CARLA에 맵을 넣는 방법은 두 가지인데, 이 시스템에서는 **하나만 가능**하다.

| 방식 | 필요 환경 | 비주얼(FBX) | 본 시스템 가능 여부 |
| :--- | :--- | :--- | :--- |
| **스탠드얼론 OpenDRIVE** (`config.py --xodr-path`) | 패키지 빌드만 | ❌ 도로만 절차 생성 | 가능하지만 외관·주차장 구조 손실 |
| **풀 FBX 임포트** (`make import` → `make package`) | UE4 + **CARLA 소스 빌드** | ✅ RoadRunner 외관 보존 | **이 방법을 사용** |

`~/carla`(런타임용 **패키지 빌드**, CARLA 0.9.16)는 `CarlaUE4.sh`만 있고 Unreal 소스·`Makefile`이
없어 **FBX를 쿠킹(cook)할 수 없다**. FBX 쿠킹은 Unreal Editor가 필요하므로, 별도로 존재하는
**CARLA 소스 빌드**에서 맵을 쿠킹해 `.tar.gz` 패키지로 만든 뒤 패키지 빌드에 끼워넣는다.

#### 환경 인벤토리 (이 절차가 의존하는 자산)

| 자산 | 경로 | 역할 |
| :--- | :--- | :--- |
| 런타임 패키지 빌드 | `/home/hannibal/carla` | Section 12.1을 실행하는 곳. 맵을 **여기에 임포트**한다 |
| **CARLA 소스 빌드** | `/home/hannibal/carla-0.9.16-source` | `make import` / `make package` 로 FBX를 쿠킹하는 곳 |
| Unreal Engine | `/home/hannibal/UnrealEngine_4.26` | `UE4_ROOT` 환경변수로 지정. 쿠킹 엔진 |
| Python venv | `/home/hannibal/carla/.venv` | 기본 `python3`. carla 모듈·빌드 도구가 여기서 해석됨 |

> **전제**: 소스 빌드(`carla-0.9.16-source`)는 이미 한 번 빌드되어 `Unreal/CarlaUE4/Binaries/Linux/`에
> `libUE4Editor-CarlaUE4.so`가 존재해야 한다. 없으면 최초 `make import`가 에디터를 통째로
> 빌드하느라 매우 오래 걸린다(수 시간).

---

### 15.2 전체 파이프라인 개요

```text
[RoadRunner 익스포트]   (CustomMap/MandoParking1, MandoParking2, MandoParking3)
  Mando1.fbx+xodr / Mando2.fbx+xodr / Mando3.fbx+xodr
        │
        │ Step A: cp 세 쌍 모두 → carla-0.9.16-source/Import/  &&  make import
        ▼
[소스 빌드 에디터 콘텐츠]  Unreal/CarlaUE4/Content/map_package/Maps/{Mando1,Mando2,Mando3}/
  <맵>.umap + OpenDrive/<맵>.xodr + TM/<맵>.bin + RoadRunner 머티리얼(.uasset)
        │
        │ Step B: make package ARGS="--packages=map_package"  (세 맵 함께 쿡)
        ▼
[배포 패키지]  Dist/map_package_<TAG>.tar.gz   (LinuxNoEditor 쿡 결과, Mando1+Mando2+Mando3 포함)
        │
        │ Step C: cp → ~/carla/Import/  &&  ./ImportAssets.sh
        ▼
[런타임 패키지 빌드]  ~/carla/CarlaUE4/Content/map_package/Maps/{Mando1,Mando2,Mando3}/
        │
        │ Step D: 서버 기동 → load_world('Mando1') / 'Mando2' / 'Mando3' 검증
        ▼
[Section 12.1 연동]  Step E: 터미널 2를 --map Mando1|Mando2|Mando3 로 + 스폰 좌표 인자(--spawn-*)
```

소요 시간(참고): Step A ≈ 5–10분, Step B ≈ 8–15분(첫 쿡, 셰이더 컴파일 포함), Step C ≈ 1–2분.

---

### 15.3 Step A — `make import` (FBX → 에디터 콘텐츠 쿠킹)

#### A-1. 입력 파일 복사

`.fbx`와 `.xodr`는 **basename이 같아야** CARLA 임포터가 한 맵으로 인식한다(`Mando1.fbx` + `Mando1.xodr`
→ 맵 이름 `Mando1`). basename이 다르면 **여러 맵을 한 번에** import 할 수 있으므로 `Mando1`/`Mando2`/`Mando3`을
동시에 복사한다. `.rrdata.xml`·`.geojson`은 **사용하지 않으므로 복사 불필요**.

> **basename 주의**: `MandoParking3`의 RoadRunner 익스포트 원본은 basename이 `Mando`였으나, 맵 이름을
> `Mando3`으로 쓰려면 **파일명을 `Mando3.fbx`/`Mando3.xodr`로 맞춰야** 한다(저장소에는 이미 `Mando3.*`로
> 보관됨). basename이 다르면 다른 맵으로 import된다.

```bash
# 세 맵을 한 번에 import (각각 Mando1, Mando2, Mando3 맵이 됨)
DEST=/home/hannibal/carla-0.9.16-source/Import
cp /home/hannibal/carla/CustomMap/MandoParking1/Mando1.fbx \
   /home/hannibal/carla/CustomMap/MandoParking1/Mando1.xodr \
   /home/hannibal/carla/CustomMap/MandoParking2/Mando2.fbx \
   /home/hannibal/carla/CustomMap/MandoParking2/Mando2.xodr \
   /home/hannibal/carla/CustomMap/MandoParking3/Mando3.fbx \
   /home/hannibal/carla/CustomMap/MandoParking3/Mando3.xodr \
   "$DEST/"
# 하나만 적용하려면 해당 맵의 .fbx/.xodr 한 쌍만 복사하면 된다.
```

> **`.json` 메타파일 불필요**: `Util/BuildTools/Import.py`가 `Import/` 안의 `.fbx`+`.xodr` 쌍을
> 스캔해 `map_package.json`을 **자동 생성**한다(`use_carla_materials: true` 기본). 패키지 이름은
> 기본 `map_package`. 바꾸려면 `make import ARGS="--package=MyPkg"`.
>
> **⚠️ 함정 — 기존 `Import/map_package.json` 재사용**: `Import.py`는 `Import/`에 이미 `*.json`이
> 있으면 **새로 생성하지 않고 그 파일을 그대로 쓴다**(`if len(json_list) < 1: generate`). 따라서
> 이전 import에서 남은 `map_package.json`(예: `Mando1`+`Mando2`만 들어 있음)이 있으면 새로 추가한
> `Mando3.fbx`/`.xodr`이 **무시되어 import되지 않는다**. 맵을 추가할 때는 **`make import` 전에**
> `rm -f Import/map_package.json Import/roadpainter_decals.json`으로 지워 재생성시키거나, 기존
> json의 `maps` 배열에 새 맵 항목을 직접 추가한다.

#### A-2. `make import` 실행

```bash
cd /home/hannibal/carla-0.9.16-source
export UE4_ROOT=/home/hannibal/UnrealEngine_4.26
make import
```

내부 동작 순서(로그로 확인 가능):
1. 선행 빌드: `LibCarla` → `osm2odr` → `CarlaUE4Editor` → `PythonAPI` (이미 빌드돼 있으면 "up to date")
2. UE4 커맨드릿 체인: `ImportAssets`(FBX 임포트) → `MoveAssets`(시맨틱 폴더 이동) →
   `PrepareAssetsForCooking` → `LoadAssetMaterials`(RoadRunner 머티리얼 매칭)
3. 각 `<맵>.xodr`를 `Content/map_package/Maps/<맵>/OpenDrive/`로 복사 (`<맵>` = `Mando1`, `Mando2`, `Mando3`)
4. `carla.Map().cook_in_memory_map()`으로 맵별 TM 바이너리(`TM/<맵>.bin`) 생성

#### A-3. ⚠️ 함정 #1 — `No module named build.__main__` (반드시 사전 처리)

`make import`는 선행 타겟 `PythonAPI`를 재빌드하는데, `BuildPythonAPI.sh`가 `python3 -m build --wheel`을
호출한다. venv에 PyPI `build` 패키지가 없으면 다음 에러로 **전체가 중단**된다:

```text
/home/hannibal/carla/.venv/bin/python3: No module named build.__main__; 'build' is a package and cannot be directly executed
make: *** [Util/BuildTools/Linux.mk:89: PythonAPI] Error 1
```

**해결** (최초 1회):
```bash
python3 -m pip install build wheel
```

#### A-4. 결과 검증

```bash
ls ~/carla-0.9.16-source/Unreal/CarlaUE4/Content/map_package/Maps/Mando1/
ls ~/carla-0.9.16-source/Unreal/CarlaUE4/Content/map_package/Maps/Mando2/
ls ~/carla-0.9.16-source/Unreal/CarlaUE4/Content/map_package/Maps/Mando3/
```
기대 산출물 (맵 폴더마다, `<맵>` = `Mando1` / `Mando2` / `Mando3`):

| 파일 | 의미 |
| :--- | :--- |
| `<맵>.umap` | 쿠킹된 맵 본체 |
| `OpenDrive/<맵>.xodr` | 논리 도로망 (GNSS/스폰/내비) |
| `TM/<맵>.bin` | Traffic Manager 바이너리 |
| `Asphalt1_*.uasset`, `Concrete1_*.uasset`, `Grass2_*` 등 | RoadRunner 머티리얼 (외관, 두 맵 공유) |

> **`Nav/<맵>.bin` 미생성은 정상/무해**: 보행자(walker) RecastNav 메시이며, 차량 자율주행·GNSS·EKF·
> MPPI·주차와 무관하다. 보행자 NPC가 필요할 때만 별도 처리.

---

### 15.4 Step B — `make package` (배포용 `.tar.gz` 쿠킹)

`map_package`(= `Mando1`+`Mando2`+`Mando3`)만 쿡해 패키지 빌드에 끼울 `.tar.gz`를 만든다. **인자 없는
`make package`는 모든 Town 맵까지 전부 쿡(1~2시간)** 하므로 반드시 `--packages`로 한정한다.

```bash
cd /home/hannibal/carla-0.9.16-source
export UE4_ROOT=/home/hannibal/UnrealEngine_4.26
make package ARGS="--packages=map_package"
```

내부 동작: `Package.sh`가 UAT cook 커맨드릿을 실행
(`-run=cook -map=+/Game/map_package/Maps/Mando1/Mando1+/Game/map_package/Maps/Mando2/Mando2 -cooksinglepackage -targetplatform=LinuxNoEditor`)
→ 의존 에셋 800~950개 쿡 + 셰이더 컴파일 → `tar`/`gzip` 압축. (패키지 내 모든 맵이 함께 쿡됨)

성공 시 마지막 로그: `Package.sh: Success!`
산출물:
```text
/home/hannibal/carla-0.9.16-source/Dist/map_package_<TAG>.tar.gz   (~260 MB)
```
여기서 `<TAG>`는 소스 빌드의 git 리비전(예: `294096eb1-dirty`). **파일명은 실행 시점마다 다를 수
있으니** 다음으로 확인:
```bash
ls -t ~/carla-0.9.16-source/Dist/map_package_*.tar.gz | head -1
```

#### ⚠️ 함정 #2 — `parse-options: unrecognized option '--packages=map_package'`

선행 빌드 보조 스크립트(BuildUE4Plugins/Setup/BuildLibCarla 등)에도 `$(ARGS)`가 전달되는데
이들은 `--packages`를 모른다. **getopt 경고만 찍고 무시한 뒤 정상 빌드되므로 무해**하다(`--packages`를
실제로 소비하는 건 최종 `Package.sh`뿐). 로그에 이 경고가 여러 번 보여도 정상이다.

---

### 15.5 Step C — 패키지 빌드(`~/carla`)에 임포트

```bash
# 1) 쿡된 .tar.gz 를 패키지 빌드 Import 폴더로
cp "$(ls -t ~/carla-0.9.16-source/Dist/map_package_*.tar.gz | head -1)" ~/carla/Import/

# 2) 패키지 빌드 루트에서 ImportAssets.sh 실행 (tar.gz 압축해제)
cd ~/carla
./ImportAssets.sh

# 3) (권장) Import 폴더 정리 — 다음 임포트 때 중복 추출 방지
rm -f ~/carla/Import/map_package_*.tar.gz
```

`ImportAssets.sh`는 `Import/`의 모든 `*.tar.gz`를 `tar --keep-newer-files -xvf`로 `~/carla`에 푼다.
tar 내부 구조가 `CarlaUE4/Content/...`, `Engine/Content/...`이므로 `~/carla` 루트에서 풀면 정확히
`~/carla/CarlaUE4/Content/map_package/Maps/{Mando1,Mando2,Mando3}/`에 안착한다.

검증:
```bash
ls ~/carla/CarlaUE4/Content/map_package/Maps/Mando1/
ls ~/carla/CarlaUE4/Content/map_package/Maps/Mando2/
ls ~/carla/CarlaUE4/Content/map_package/Maps/Mando3/
#   → 각 폴더에 <맵>.umap, OpenDrive/<맵>.xodr, TM/<맵>.bin 가 보이면 성공
```

---

### 15.6 Step D — 로드 검증

서버를 띄우고 파이썬 클라이언트로 맵 목록·로드·스폰을 확인한다.

```bash
# 터미널 A — 서버 (헤드리스)
cd ~/carla
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ./CarlaUE4.sh -RenderOffScreen -quality-level=Low

# 터미널 B — 검증 클라이언트
cd ~/carla && source .venv/bin/activate
python3 - <<'PY'
import carla, time
c = carla.Client('localhost', 2000); c.set_timeout(60.0)
maps = [m.split('/')[-1] for m in c.get_available_maps()]
for name in ('Mando1', 'Mando2', 'Mando3'):
    assert name in maps, f'{name} 없음: {maps}'
    w = c.load_world(name); time.sleep(2.0)
    m = w.get_map()
    print('로드 OK:', m.name)                  # map_package/Maps/<맵>/<맵>
    wps = [wp for wp in m.generate_waypoints(1.0) if str(wp.lane_type)=='Driving']
    xs=[wp.transform.location.x for wp in wps]; ys=[wp.transform.location.y for wp in wps]
    print('  driving lane 범위 x[%.1f~%.1f] y[%.1f~%.1f]'%(min(xs),max(xs),min(ys),max(ys)))
PY
```

`Mando1`/`Mando2`/`Mando3`의 주행 차선 범위(참고): **x[-113.7 ~ -93.6], y[-59.4 ~ 0]** (≈20×60 m 소규모
주차장형). 세 맵은 같은 만도 주차장 기반이라 범위가 거의 동일하나(`Mando3`은 장애물만 제거), 형상을 수정했다면 맵별로 재확인.

> **버전 경고는 무해**: 클라이언트가 `WARNING: Version mismatch ... Client API = 294096eb1-dirty,
> Simulator API = 0.9.16`를 띄운다. 이는 Step B의 `PythonAPI.wheel` 빌드가 소스 빌드 wheel을 venv에
> `--force-reinstall`했기 때문(버전 문자열만 다르고 ABI 동일). 동작에 영향 없음.

> **geoReference 경고도 무해**: `WARNING: cannot parse georeference: ''. Using default values.`
> RoadRunner xodr에 투영 정보가 없어 GNSS가 기본 원점(0,0) 기준이 된다. dual_filter 스택은
> **datum-상대**(첫 fix 기준)라 측위·azimuth·EKF·주차 전부 정상. 실좌표가 필요하면 15.9 참고.

---

### 15.7 Step E — Section 12.1 연동 (맵 + 스폰 좌표)

기존 12.1 절차에서 **딱 두 가지**만 바꾼다.

#### E-1. 터미널 2 — 맵 로드 + 스폰 좌표 인자 (맵별 예시)

스폰 좌표는 `--spawn-x/y/z/yaw` 인자로 지정한다(파일 편집 불필요, 맵에 맞게 좌표만 교체).

**카를라맵 (Town01_Opt) 버전:**

```bash
cd ~/carla && source .venv/bin/activate
python PythonAPI/util/config.py --map Town01_Opt \
  && python PythonAPI/examples/manual_control.py --rolename car \
     --filter vehicle.micro.microlino --generation 2 --sync \
     --spawn-x 299.4 --spawn-y 133.24 --spawn-z 0.3 --spawn-yaw 0.0
```

**만도맵 (Mando1 / Mando2 / Mando3) 버전:** `--map`에 `Mando1` / `Mando2` / `Mando3` 지정.

```bash
cd ~/carla && source .venv/bin/activate
python PythonAPI/util/config.py --map Mando3 \
  && python PythonAPI/examples/manual_control.py --rolename car \
     --filter vehicle.micro.microlino --generation 2 --sync \
     --spawn-x -93.6 --spawn-y 0.0 --spawn-z 0.3 --spawn-yaw -90.0
```

나머지 터미널 1·3~8은 12.1 그대로(맵 무관). 클릭 주차만 할 거면 **터미널 6(csv_to_utm)은 생략**
(그 CSV는 Town01용이라 새 맵과 무관 → 15.8 참고).

#### E-2. 스폰 좌표 — 커맨드라인 인자로 지정

`manual_control.py`는 추천 spawn point가 아니라 **고정 좌표**에 ego를 스폰한다. 이 좌표는 **커맨드라인
인자 `--spawn-x/--spawn-y/--spawn-z/--spawn-yaw`로 지정**한다(파일을 매번 편집할 필요 없음). 인자를
생략하면 아래 기본값(= Mando1/Mando2/Mando3 차선)이 쓰인다.

| 인자 | 기본값 | 의미 |
| :--- | :--- | :--- |
| `--spawn-x` | `-93.6` | CARLA world X [m] |
| `--spawn-y` | `0.0` | CARLA world Y [m] |
| `--spawn-z` | `0.3` | Z [m] (지면 충돌 방지 오프셋) |
| `--spawn-yaw` | `-90.0` | yaw [deg] (도로 진행 방향) |

> **구현 위치**: `main()`의 argparse에 `--spawn-*` 4개가 정의돼 있고, `World.__init__`이 이를
> `self._spawn_x/y/z/yaw`로 저장한 뒤 `restart()`의 `spawn_point` 생성에 사용한다.
> (이전 Town01_Opt 하드코딩 값: x=299.4, y=133.24, yaw=0.0)

**새 맵에서 유효한 스폰 좌표를 구하는 방법** (원하는 대략 위치 `(x,y)`를 차선에 스냅 → 그 값을
`--spawn-x/y/yaw`로 넘김):
```bash
cd ~/carla && source .venv/bin/activate
python3 - <<'PY'
import carla
c=carla.Client('localhost',2000); c.set_timeout(60.0)
m=c.load_world('Mando1').get_map()               # 또는 'Mando2' / 'Mando3'
loc=carla.Location(x=-93.0, y=3.0, z=0.0)        # ← 원하는 대략 위치
wp=m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
t=wp.transform
print('스냅 위치 x=%.2f y=%.2f z=%.2f | 차선 yaw=%.1f | 요청점과 거리=%.2fm'
      %(t.location.x, t.location.y, t.location.z, t.rotation.yaw,
        ((t.location.x-loc.x)**2+(t.location.y-loc.y)**2)**0.5))
PY
```
- `project_to_road=True`로 가장 가까운 **Driving 차선**에 스냅된 좌표와 **차선 방향 yaw**를 얻는다.
- 스폰 `z`는 지면 충돌 방지를 위해 `+0.3` 권장.
- 패치 후 깨끗한 월드에서 `world.try_spawn_actor(...)`가 `None`이 아니면(충돌 없음) 유효.
  (주의: 같은 세션에서 반복 스폰/삭제 시 직전 액터 잔재로 충돌해 `None`이 날 수 있음 — 맵을
  새로 로드하거나 1회만 스폰해 확인할 것.)

> **CARLA 좌표계**: 좌수 좌표 X=전방, **Y=우측**, Z=상. `Mando1`/`Mando2`/`Mando3` 차선은 y가 음수 방향으로 뻗어 있고,
> 도로 진행 방향 yaw는 -90°(−Y쪽)이다.

---

### 15.8 클릭 주차 모드 동작 구조 (CSV 불필요)

`follow_path_client`는 IDLE / CSV_FOLLOWING / **PARKING** 상태 머신이다.

```text
RViz "2D Goal Pose"(단축키 G) 드래그
   → /goal_pose (geometry_msgs/PoseStamped)
   → follow_path_client._goal_pose_cb → PARKING 전환
   → planner_server.compute_path_to_pose (SmacPlannerHybrid)  ← 경로를 플래너가 생성
   → controller_server FollowPath/ParkingPath (MPPI)
   → /cmd_vel → cmd_vel_to_carla → CARLA VehicleControl
```

- **CSV(`/csv_path`)는 차선 추종(CSV_FOLLOWING)용**이며 주차와 독립이다. 따라서 **클릭 주차는
  레퍼런스 CSV가 없어도 동작**한다 → 터미널 6 생략 가능.
- 주차 목표는 반드시 **맵 도로 영역 안**(Mando1/Mando2/Mando3: x −113.7~−93.6, y −59.4~0)에서 클릭할 것.
- `global_costmap`은 `rolling_window: true` 200×200 m라 로봇을 따라다니며 자동으로 새 맵을 커버한다
  (맵별 costmap 재설정 불필요).

기동 중 건강 체크:
```bash
ros2 run tf2_tools view_frames            # utm → odom → base_link
ros2 lifecycle get /controller_server     # active [3]
ros2 action info /follow_path             # Action servers: 1
ros2 topic echo /cmd_vel                  # 클릭 후 속도 명령 발행 확인
```

---

### 15.9 옵션 — geoReference 주입 (GNSS 실좌표화)

RoadRunner xodr에는 `<geoReference>`가 없어 GNSS가 (0,0) 기준이 된다(dual_filter는 datum-상대라
무해). **실제 UTM 좌표가 필요**하면 `make import` **이전에** xodr `<header>` 안에 투영 문자열을
주입한다.

```xml
<header revMajor="1" revMinor="4" ...>
  <geoReference><![CDATA[+proj=tmerc +lat_0=<위도> +lon_0=<경도> +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>
  ...
</header>
```
주입 후 Step A부터 다시 수행한다. (좌표값은 맵의 실제 설치 지점 위경도로 설정.)

---

### 15.10 CARLA 좌표 ↔ UTM 변환 및 맵 시각화/검증 도구

> **이 절의 목적**: 맵의 차선·장애물 좌표를 **정적 costmap(StaticLayer)** 으로 굽거나 절대 위치를
> 다룰 때, CARLA 월드 좌표를 ROS 스택이 쓰는 **UTM/`utm` 프레임**으로 변환하는 방법과 그 정확성을
> 검증하는 도구를 기록한다. 15.9(geoReference 주입)는 _실좌표를 맵에 박아 넣는_ 방법이고, 이 절은
> _geoReference를 건드리지 않고 CARLA 자신의 georeference로 오프라인 변환을 얻는_ 방법이다.

#### 배경 — 왜 변환이 필요한가

`CustomMap/visualize_map.py`(아래)가 보여주는 좌표는 **CARLA 월드 (x, y) 미터**다. 반면 Nav2/dual_filter
스택은 [Section 2.3](#23-utm--전역-절대-기준점)의 **`utm` 프레임**(datum-상대)에서 동작한다. 따라서
맵 좌표를 costmap에 쓰려면 `CARLA world → UTM → utm 프레임` 변환이 필요하다.

핵심은 **CARLA 내장 GNSS 센서가 lat/lon을 만드는 것과 똑같은 georeference를 오프라인에서도 그대로
재현할 수 있다**는 점이다. `carla.Map.transform_to_geolocation()`이 그 georeference를 직접 노출한다.

#### 변환 사슬

```text
CARLA world (x, y)
   │  ① carla.Map.transform_to_geolocation(Location(x,y,z))   ← CARLA georeference (라이브 GNSS와 동일)
   ▼
위경도 (lat, lon)
   │  ② to_utm(lat, lon)   (visualize_map.py, gnss_to_utm/utm_converter.hpp 와 동일한 표준 WGS84 UTM)
   ▼
절대 UTM (E, N)            ← /f9r_utm 과 동일한 값
   │  ③ (E − datum_E), −(N − datum_N)   ← gnss_to_odom.py datum 적용 + CARLA +Y=right 미러링
   ▼
ROS utm 프레임 (x_ros, y_ros)
```

- **① georeference**: 이 맵들은 xodr에 `<geoReference>`가 없어(`cannot parse georeference: ''`) CARLA
  기본 georeference를 쓴다. 실측 결과 기본 기준점은 **lat≈42.0, lon≈2.0 (바르셀로나/UAB)** 이다.
- **② UTM 공식**: `visualize_map.py`의 `to_utm()`은 `gnss_to_utm/utm_converter.hpp`의 `toUTM()`과
  **동일한 수식**(k0=0.9996, false easting 500000, zone=lon 기반)이라 라이브 `f9r_to_utm`과 비트 단위로 일치한다.
- **③ datum**: [Section 2.3](#23-utm--전역-절대-기준점)대로 datum은 실행마다 달라진다. 정적 지도를 미리
  구워 재사용하려면 datum을 **고정 UTM 값으로 하드코딩**해야 ③ 변환이 실행 불변이 된다.

#### 왜 라이브 파이프라인과 동일한가 (검증 완료)

라이브 GNSS 센서는 **서버가 로드한 맵**의 georeference로 lat/lon을 만들고, 오프라인 변환은
**xodr로 생성한 `carla.Map`**의 georeference를 쓴다. 둘이 같으면 오프라인 UTM = 라이브 UTM이다.

| 단계 | 라이브 (실행 중) | 오프라인 (visualize_map) | 일치 |
| :--- | :--- | :--- | :---: |
| world → lat/lon | 서버 맵 georeference | `carla.Map.transform_to_geolocation` | ✅ |
| lat/lon → UTM | `f9r_to_utm` (`utm_converter.hpp`) | `to_utm()` 동일 수식 | ✅ |

`verify_utm_georef.py`로 두 georeference를 비교한 결과 **최대 UTM 오차 0.0000 m** 로 일치 확인.
즉 오프라인 변환을 그대로 신뢰해 costmap 좌표로 사용할 수 있다.

#### 도구 1 — `CustomMap/visualize_map.py` (맵 항공뷰 + 좌표 표시)

xodr을 오프라인 파싱해 모든 레인(주행/주차칸/인도/갓길)의 중심선·좌우 경계와 `<objects>`(트래픽콘 등
장애물)를 CARLA 월드 좌표로 그린다. 마우스를 올리면 해당 픽셀의 **CARLA (x,y)와 UTM (E,N)** 을 함께 표시한다.

```bash
cd ~/carla/CustomMap
python visualize_map.py MandoParking2                      # 맵 이름으로
python visualize_map.py CustomMap/MandoParking2/Mando2.xodr # .xodr 경로 직접
python visualize_map.py MandoParking2 --step 0.3 --size 1200 # 촘촘하게 + 큰 창
```

| 인자 | 기본값 | 의미 |
| :--- | :---: | :--- |
| `map` | (필수) | 맵 이름(`MandoParking2`) 또는 `.xodr` 경로 |
| `--step` | 0.5 | 레인 샘플링 간격(m). 작을수록 곡선이 매끄럽고 느림 |
| `--size` | 1000 | 창 최대 크기(px) |

- 시작 로그에 **CARLA 범위와 UTM 범위**를 함께 출력한다.
- 화면 좌상단 라벨: `CARLA x=… y=…` / `UTM E=… N=…` 두 줄. `q`/`ESC`로 종료.
- 장애물은 빨간 원 + `width×length` 외형 박스(`hdg` 회전)로 표시된다.
- 의존성: `carla`, `numpy`, `opencv-python`. OpenCV 창이 뜨므로 디스플레이가 필요하다(서버는 불필요 — xodr만으로 동작).

#### 도구 2 — `CustomMap/verify_utm_georef.py` (georeference 일치 검증)

오프라인 xodr georeference가 라이브 서버 맵 georeference와 같은지 확인한다. **CARLA 서버가 떠 있고
해당 맵이 로드된 상태**에서 실행한다(ROS 불필요 — 두 `carla.Map`의 `transform_to_geolocation`만 비교).

```bash
cd ~/carla/CustomMap
python verify_utm_georef.py MandoParking2
python verify_utm_georef.py MandoParking2 --host 127.0.0.1 --port 2000 --tol 0.05
```

| 인자 | 기본값 | 의미 |
| :--- | :---: | :--- |
| `map` | (필수) | 맵 이름 또는 `.xodr` 경로 |
| `--host` / `--port` | 127.0.0.1 / 2000 | CARLA 서버 주소 |
| `--tol` | 0.05 | UTM 허용 오차(m). 이내면 PASS |

- 맵 경계(xodr `<header>`의 north/south/east/west) 안에 5×5 격자 점을 만들어 두 맵의 lat/lon·UTM을 비교한다.
- `최대 UTM E/N 차이`가 `--tol` 이내면 `[✓] PASS`, 종료코드 0. 초과면 `[✗] FAIL`, 종료코드 1.
- 서버 미실행 시 연결 실패 메시지 후 종료코드 2.

> **권장 순서**: ① 서버에 맵 로드 → ② `verify_utm_georef.py`로 PASS 확인 → ③ `visualize_map.py`의
> UTM 값을 신뢰해 정적 costmap 좌표로 사용. geoReference를 주입(15.9)하면 기준점이 실좌표로 바뀌므로,
> 주입 여부를 바꿨다면 검증을 다시 수행한다.

---

### 15.11 맵 수정 후 재적용 절차 (반복 작업 — 핵심 요약)

RoadRunner에서 맵을 수정·재익스포트한 뒤 적용하려면 아래만 반복한다. **맵 이름(basename)을
`Mando1`/`Mando2`/`Mando3`로 유지**하면 `make import`가 기존 에셋을 `bReplaceExisting`으로 덮어쓴다(또 다른
맵으로 만들려면 새 basename 사용 — `Mando4` 등).

```bash
# [0] (최초 1회만) 빌드 도구 설치
python3 -m pip install build wheel

# [1] 새 익스포트물 복사 (basename = 맵 이름 유지). 갱신할 맵의 쌍을 모두 복사
cp ~/carla/CustomMap/MandoParking1/Mando1.fbx ~/carla/CustomMap/MandoParking1/Mando1.xodr \
   ~/carla/CustomMap/MandoParking2/Mando2.fbx ~/carla/CustomMap/MandoParking2/Mando2.xodr \
   ~/carla/CustomMap/MandoParking3/Mando3.fbx ~/carla/CustomMap/MandoParking3/Mando3.xodr \
   ~/carla-0.9.16-source/Import/

# [2] 에디터 콘텐츠로 쿠킹
cd ~/carla-0.9.16-source && export UE4_ROOT=/home/hannibal/UnrealEngine_4.26
make import

# [3] 배포 .tar.gz 쿠킹 (map_package = Mando1+Mando2+Mando3)
make package ARGS="--packages=map_package"

# [4] 패키지 빌드에 임포트
cp "$(ls -t Dist/map_package_*.tar.gz | head -1)" ~/carla/Import/
cd ~/carla && ./ImportAssets.sh && rm -f Import/map_package_*.tar.gz

# [5] 로드 검증 (15.6) 후 12.1 실행. 도로 형상이 바뀌었으면 스폰 좌표 재확인(15.7 E-2)
```

#### 재적용 시 주의 체크리스트
- [ ] `Import/`에 이전 `.tar.gz`가 남아있지 않은지 (남으면 ImportAssets가 중복 추출)
- [ ] 도로 형상이 변경됐다면 **스폰 좌표**를 15.7 E-2로 재확인 후 `--spawn-x/y/z/yaw` 인자로 전달
- [ ] 차선 추종(CSV)을 쓴다면 **레퍼런스 CSV를 새 맵에서 재녹화**(Section 11). 클릭 주차만이면 불필요
- [ ] geoReference가 필요하면 `make import` **전에** 주입(15.9)

---

### 15.12 트러블슈팅 요약표

| 증상 | 원인 | 해결 |
| :--- | :--- | :--- |
| `No module named build.__main__` → `make: *** [PythonAPI] Error 1` | venv에 PyPI `build` 미설치 | `python3 -m pip install build wheel` |
| `parse-options: unrecognized option '--packages=...'` | 선행 빌드 스크립트가 옵션 미인식 | **무해**, 무시 (Package.sh만 소비) |
| `make package`가 1~2시간 걸림 | `--packages` 빠뜨려 전체 Town 맵 쿡 | `ARGS="--packages=map_package"` 사용 |
| 맵 목록에 `Mando1`/`Mando2`/`Mando3` 없음 | ImportAssets 미실행 / Import에서 추출 실패 | Step C 재수행, `~/carla/CarlaUE4/Content/map_package/Maps/<맵>/<맵>.umap` 존재 확인 |
| 새 맵(`Mando3`)을 추가했는데 import 안 됨 (`MoveAssets -Maps=Mando1 Mando2`만 보임) | `Import/`에 남은 옛 `map_package.json`을 `Import.py`가 재사용 | `make import` 전에 `rm -f Import/map_package.json Import/roadpainter_decals.json` (15.3 A-1 함정) |
| `There are no spawn points available` | 추천 spawn point 0개인 맵 | `--spawn-x/y/z/yaw` 인자에 차선 스냅값 전달(15.7 E-2) |
| `try_spawn_actor`가 `None` 반환 | 스폰 지점에 액터 잔재 충돌 / 공중·벽 | 맵 새 로드 후 1회 스폰, z+0.3, 차선 스냅 좌표 사용 |
| `Version mismatch ... -dirty` 경고 | make package가 소스 wheel을 venv에 재설치 | **무해**, ABI 동일 |
| `cannot parse georeference: ''` | xodr에 `<geoReference>` 없음 | datum-상대라 **무해**. 실좌표 필요 시 15.9 |
| 클릭해도 주차 안 함 | controller/planner 미활성 또는 목표가 costmap 밖 | `ros2 lifecycle get /controller_server`=active[3], 도로 영역 안에서 클릭 |
