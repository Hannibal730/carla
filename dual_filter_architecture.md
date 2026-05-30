# ROS 2 Humble: 자율주행 듀얼 필터(Dual Filter) 기반 센서 퓨전 아키텍처 구축 매뉴얼

내가 사용 가능한 ros2 topic은 다음과 같다.

| 센서 | 토픽 | ROS2 메시지 타입 |
| :--- | :--- | :--- |
| CARLA simulation clock | `/clock` | `rosgraph_msgs/msg/Clock` |
| RGB 카메라 | `/carla/car/rgb/image` | `sensor_msgs/msg/Image` |
| LiDAR | `/carla/car/lidar/point_cloud` | `sensor_msgs/msg/PointCloud2` |
| GNSS (후륜축) | `/carla/car/f9r/fix` | `sensor_msgs/msg/NavSatFix` |
| GNSS (전방 1.4m) | `/carla/car/f9p/fix` | `sensor_msgs/msg/NavSatFix` |
| IMU | `/carla/car/imu/data` | `sensor_msgs/msg/Imu` |


## 1. 시스템 개요 (System Overview)
본 매뉴얼은 실외 자율주행 차량의 정밀 측위 및 제어 안정성 확보를 위해 **REP-105 표준**을 준수하는 듀얼 필터(Dual Filter) 아키텍처를 구축하는 지침이다. 단일 필터에 GNSS를 결합할 경우 발생하는 위치 도약(Jump) 현상으로 인한 제어기 발산을 방지하기 위해 로컬 필터와 글로벌 필터를 분리하여 구성한다.

*   **Target OS / Middleware:** Ubuntu 22.04 / ROS 2 Humble
*   **Target Package:** `robot_localization`
*   **Sensor Inputs:** CARLA Vehicle API (→ `/wheel/odom`), 6-DOF IMU, Dual RTK GNSS (f9r / f9p)
*   **Controller:** MPPI

---

## 2. 좌표계(TF Tree) 명세 (REP-105 준수)

시스템은 다음의 계층적 TF 트리를 반드시 유지해야 한다.

```
map ──[global_ekf]──> odom ──[local_ekf]──> base_link
```

본 시스템에는 EKF 기반 TF 트리(dual_filter 스택)와, 독립적인 GNSS 전용 경로 추종 프레임(`csv`)이 별도로 존재한다. 두 스택은 TF 트리를 공유하지 않는다.

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

### 2.3 `map` — 전역 절대 기준점

**원점:** `utm_to_odometry.py`에서 설정한 `datum_easting / datum_northing` (UTM 좌표). 이 점이 ROS의 `(0, 0)` map 원점이 된다.

```
현재 설정 (utm_to_odometry.py):
  datum_easting  = 첫 번째 f9r GNSS 수신 시의 UTM easting
  datum_northing = 첫 번째 f9r GNSS 수신 시의 UTM northing

→ datum이 실행마다 달라지므로 map 원점도 실행마다 달라진다.
  datum을 고정 UTM 값으로 하드코딩하면 map 원점도 항상 일정해진다.
```

`map` 프레임을 이해하는 핵심은 **`map`이 `odom`을 보정하는 프레임**이라는 것이다.

```
[이해하기 어려운 이유]
직관적으로는: map → base_link 하나면 충분하지 않나?

[실제 이유]
odom → base_link 는 연속적(제어기용)
map → odom      는 가끔 점프해서 절대 위치 보정

둘을 분리함으로써, GNSS 보정이 제어기에 직접 전달되는 것을 차단한다.
```

**`map → odom` TF의 물리적 의미:**

```
initial:  map → odom = 항등변환
          (map 원점과 odom 원점이 같은 위치)

500m 주행 후 odom이 2m 동쪽으로 드리프트했다면:
  global_ekf가 GNSS로 실제 위치를 파악
  → map → odom 오프셋을 2m 서쪽으로 조정
  → odom → base_link 는 그대로 (제어기에 영향 없음)
  → map → odom → base_link 합산으로 실제 절대 위치 계산
```

- **발행 주체:** `global_ekf` (`world_frame: map`, `publish_tf: true`)
- **계산 방법:** wheel `vx` + IMU `wz` + GNSS UTM `(x,y)` + azimuth yaw의 EKF 융합
- **장점:** 드리프트가 보정된 절대 위치
- **단점:** GNSS 업데이트 시 `map → odom` 오프셋이 불연속적으로 변할 수 있다(Jump). 단, 이 Jump는 `odom → base_link`에는 전달되지 않으므로 제어기는 영향을 받지 않는다.
- **사용처:** 전역 경로 추종, MPPI의 global plan frame

---

### 2.4 `csv` — GNSS 전용 경로 추종 기준점

**원점:** CSV 경로 파일의 **첫 번째 UTM 좌표**. EKF 초기화 여부와 무관하게 항상 동일하다.

```
경로 파일: route_1.csv
  첫 번째 행: 355123.45, 4162345.67  ← 이 UTM 좌표가 csv 원점
  이후 모든 경유점은 이 점에서의 상대 좌표로 변환되어 /csv_path로 발행됨

차량 위치:
  f9r 안테나의 현재 UTM - csv 원점 = f9r 프레임의 csv 기준 위치
  tf_gnss_csv 노드가 csv → f9r TF를 브로드캐스트
```

- **발행 주체:** `tf_gnss_csv` 노드 (`gnss_to_utm` 패키지)
- **계산 방법:** 원시 f9r/f9p GNSS → UTM 변환 → csv 원점 기준 상대 좌표
- **EKF와의 관계:** 완전히 독립. `map/odom/base_link` TF 트리와 연결되지 않는다.
- **장점:** EKF 수렴 여부와 무관하고, 경로 파일이 원점이므로 매 실행마다 일관성이 유지된다.
- **단점:** IMU/wheel 융합이 없는 순수 GNSS 측위이므로 정밀도가 낮고, GNSS 노이즈가 그대로 위치에 반영된다.
- **사용처:** `dual_filter` 없이 GNSS만으로 경로 추종할 때 (두 스택을 동시에 실행하면 안 됨)

---

### 2.5 프레임 관계 요약

```
[dual_filter 스택 — EKF 기반]

  지구상의 절대 위치 (UTM datum 기준)
       │
       ▼
     map ──────────────────────────────── 전역 고정 좌표계
       │                                  원점: utm_to_odometry.py의 datum
       │ map → odom TF                    발행: global_ekf
       │ (GNSS로 드리프트 보정)
       ▼
     odom ─────────────────────────────── 출발점 고정 좌표계
       │                                  원점: 시스템 시작 시 차량 위치
       │ odom → base_link TF              발행: local_ekf
       │ (wheel + IMU 연속 적분)
       ▼
   base_link ─────────────────────────── 차량 물리 프레임
                                          원점: 후륜축 중심 (차량과 함께 이동)


[gnss_to_utm 스택 — GNSS 전용, 별도 독립]

     csv ──────────────────────────────── 경로 기준 좌표계
       │                                  원점: CSV 파일 첫 번째 UTM 점
       │ csv → f9r TF                     발행: tf_gnss_csv
       │ (raw f9r GNSS 직접 변환)
       ▼
     f9r ───────────────────────────────  차량(f9r 안테나) 현재 위치
```

| 프레임 | 원점 | 이동 여부 | 연속성 | 절대 위치 | 발행 주체 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `base_link` | 후륜축 중심 | 차량과 함께 이동 | 연속 | 없음 | local_ekf |
| `odom` | 시스템 시작 위치 | 고정 | 연속 (드리프트 있음) | 없음 | local_ekf |
| `map` | datum UTM 좌표 | 고정 | 가끔 보정(Jump 가능) | 있음 | global_ekf |
| `csv` | CSV 첫 번째 UTM 점 | 고정 | 연속 (GNSS 노이즈 포함) | 있음(상대적) | tf_gnss_csv |

---

## 3. 센서 토픽 및 메시지 타입 명세
EKF 노드에 입력되는 토픽 목록이다. CARLA 원본 토픽명과 EKF 내부 사용 토픽명이 다를 경우 launch 파일에서 리매핑이 필요하다.

| 출처 | CARLA 원본 토픽 | EKF 입력 토픽 | 메시지 타입 | 필수 데이터 필드 |
| :--- | :--- | :--- | :--- | :--- |
| **CARLA API 노드** (별도 작성) | — | `/wheel/odom` | `nav_msgs/Odometry` | `twist.twist.linear.x`, `twist.twist.linear.y = 0` |
| **IMU** | `/carla/car/imu/data` | `/imu/data` (리매핑) | `sensor_msgs/Imu` | `angular_velocity.z` |
| **GNSS (후륜축)** | `/carla/car/f9r/fix` | `/f9r/fix` (리매핑) | `sensor_msgs/NavSatFix` | `latitude`, `longitude`, `altitude`, `status` |
| **GNSS (전방 1.4m)** | `/carla/car/f9p/fix` | `/f9p/fix` (리매핑) | `sensor_msgs/NavSatFix` | `latitude`, `longitude`, `altitude`, `status` |

### 3.1 EKF가 실제로 사용하는 메시지 필드

아래 표는 전체 파이프라인에서 실제로 필터 계산에 들어가는 필드만 분리한 것이다. 카메라와 LiDAR는 RViz 확인용이며 현재 dual EKF 입력으로는 사용하지 않는다.

| 토픽 | 메시지 타입 | 사용하는 필드 | 사용 노드 | 목적 |
| :--- | :--- | :--- | :--- | :--- |
| `/clock` | `rosgraph_msgs/Clock` | `clock` | 모든 `use_sim_time:=true` 노드 | CARLA simulation time 기준으로 EKF 적분 시간 통일 |
| `/wheel/odom` | `nav_msgs/Odometry` | `header.stamp` | local/global EKF | wheel 측정 시각 |
| `/wheel/odom` | `nav_msgs/Odometry` | `twist.twist.linear.x` | local/global EKF | 차량 전방 속도 `vx` |
| `/wheel/odom` | `nav_msgs/Odometry` | `twist.twist.linear.y = 0` | local/global EKF | 비홀로노믹 제약, 옆미끄럼 없음 `vy=0` |
| `/carla/car/imu/data` | `sensor_msgs/Imu` | `header.stamp` | local/global EKF | IMU 측정 시각 |
| `/carla/car/imu/data` | `sensor_msgs/Imu` | `angular_velocity.z` | local/global EKF | yaw rate `wz` |
| `/carla/car/f9r/fix` | `sensor_msgs/NavSatFix` | `header.stamp`, `latitude`, `longitude`, `altitude` | `f9r_to_utm`, `azimuth_angle_calculator` | 후륜축 GNSS 위치와 heading 기준점 |
| `/carla/car/f9p/fix` | `sensor_msgs/NavSatFix` | `header.stamp`, `latitude`, `longitude`, `altitude` | `f9p_to_utm`, `azimuth_angle_calculator` | 전방 GNSS 위치와 heading 벡터 끝점 |
| `/f9r_utm` | `geometry_msgs/PointStamped` | `header.stamp`, `point.x`, `point.y`, `point.z` | `utm_to_odometry` | f9r UTM 위치 |
| `/azimuth_angle` | `std_msgs/Float64` | `data` | `utm_to_odometry` | f9r→f9p geographic bearing |
| `/odometry/gnss` | `nav_msgs/Odometry` | `pose.pose.position.x`, `pose.pose.position.y`, `pose.pose.orientation` | global EKF | 절대 위치와 절대 yaw 보정 |

`robot_localization`의 `*_config` 배열 순서는 다음과 같다.

```text
[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
```

따라서 `/wheel/odom`에서 `vx`, `vy`만 사용한다는 것은 7번째와 8번째 항목이 `true`라는 뜻이고, `/imu/data`에서 `angular_velocity.z`만 사용한다는 것은 `vyaw` 항목만 `true`라는 뜻이다.

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
| `utm_to_odometry` | `/odometry/gnss` | `nav_msgs/Odometry` | 글로벌 EKF 입력용 — f9r UTM 위치 + azimuth yaw를 단일 Odometry로 합성 |

> **토픽 리매핑:** launch 파일에서 CARLA 토픽명 → gnss_to_utm 내부 토픽명으로 리매핑.
> `/f9r/fix` ← `/carla/car/f9r/fix`, `/f9p/fix` ← `/carla/car/f9p/fix`
>
> **`utm_to_odometry` 구현:** `/f9r_utm` (PointStamped)와 `/azimuth_angle` (Float64, 도°)를 구독.
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
*   **입력:** `/wheel/odom`, `/imu/data`
*   **출력:** `/odometry/local` 토픽, `odom` $\rightarrow$ `base_link` TF 발행
*   **좌표계 역할:** `odom` 프레임 안에서 `base_link`가 얼마나 부드럽게 움직였는지를 표현한다. 전역 절대 위치가 아니라, 출발 이후의 상대 이동량을 누적한 로컬 추정값이다.

#### 로컬 EKF가 실제로 사용하는 입력 성분

| 입력 토픽 | 사용하는 필드 | EKF config 항목 | 역할 |
| :--- | :--- | :--- | :--- |
| `/wheel/odom` | `header.stamp` | — | wheel 속도 측정 시각. 반드시 CARLA simulation time이어야 함 |
| `/wheel/odom` | `twist.twist.linear.x` | `vx` | 차량 전방 속도. 로컬 위치 적분의 주 이동량 |
| `/wheel/odom` | `twist.twist.linear.y = 0` | `vy` | 차량은 옆으로 미끄러지지 않는다는 비홀로노믹 제약 |
| `/carla/car/imu/data` → `/imu/data` | `header.stamp` | — | IMU 측정 시각. wheel odom과 같은 `/clock` 기준이어야 함 |
| `/carla/car/imu/data` → `/imu/data` | `angular_velocity.z` | `vyaw` | 차량의 상대 yaw rate. 회전 적분의 유일한 각속도 입력 |

`/wheel/odom`의 pose, `/wheel/odom.twist.twist.angular.z`, IMU orientation, IMU linear acceleration은 로컬 EKF에서 사용하지 않는다. 로컬 회전량은 오직 `/imu/data.angular_velocity.z`에서 오며, 선속도는 오직 `/wheel/odom.twist.twist.linear.x`와 `linear.y=0` 제약에서 온다.

#### EKF 파라미터의 의미

로컬 EKF는 `ekf_params.yaml`에서 다음 구조로 설정된다.

```yaml
local_ekf:
  ros__parameters:
    two_d_mode: true
    world_frame: odom
    publish_tf: true

    odom0: /wheel/odom
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
                  false, false, false]
```

`odom0_config`에서 `vx`, `vy`만 `true`이므로 `/wheel/odom`은 위치가 아니라 속도 측정으로만 쓰인다. `imu0_config`에서 `vyaw`만 `true`이므로 IMU는 yaw rate 측정으로만 쓰인다. `world_frame: odom`과 `publish_tf: true` 때문에 로컬 EKF는 `/odometry/local`과 함께 `odom → base_link` TF를 발행한다.

#### 로컬 EKF가 계산하는 움직임

개념적으로 로컬 EKF는 매 시간 간격 `dt`마다 다음 정보를 누적한다.

```text
전방 이동량  ≈ vx × dt
회전 변화량  ≈ wz × dt
측면 속도    = 0으로 제약
```

즉 차량은 현재 바라보는 방향으로 `vx`만큼 전진하고, IMU의 `wz`만큼 회전한다고 가정한다. 이 추정은 짧은 시간에는 매우 부드럽고 제어에 적합하지만, GNSS 보정이 없으므로 장시간 운행하면 위치와 yaw가 조금씩 드리프트한다.

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

따라서 로컬 EKF는 GNSS를 쓰지 않고 wheel+IMU만 적분한다. GNSS로 누적 드리프트를 보정하는 일은 global EKF가 `map → odom` TF를 조정하는 방식으로 담당한다.

#### 왜 `odom → base_link` TF를 발행하는가

`odom` 프레임은 "시작 위치"를 원점으로 하는 로컬 좌표계이다. 로컬 EKF는 _"출발점에서 지금 어디까지 이동했는가?"_ 를 적분으로 계산하여, 그 결과를 `odom → base_link` TF로 표현한다.

```text
시작(odom 원점) ---[wheel vx + IMU wz 적분]--> 현재 차량 위치(base_link)
```

*   이 TF는 GNSS 보정이 없으므로 **연속적이고 부드러움** → 제어기가 안정적으로 동작.
*   단점: GNSS 없이 적분만 하므로 장시간 운행 시 오차 누적(드리프트) → **글로벌 필터가 `map → odom`으로 보정.**

#### 로컬 EKF에서 특히 조심해야 하는 오류

| 오류 | 증상 | 원인 |
| :--- | :--- | :--- |
| `/clock` 미사용 또는 stamp 불일치 | 90도 회전이 유턴처럼 과적분됨 | 속도는 simulation second 기준인데 EKF 적분 `dt`가 wall time으로 계산됨 |
| CARLA/ROS yaw 부호 불일치 | 좌회전/우회전 방향이 뒤집힘 | CARLA `+Y=right`, ROS `+Y=left` 미러링 누락 |
| `/wheel/odom.angular.z`와 IMU `angular_velocity.z` 동시 사용 | 회전량이 과하게 들어감 | yaw rate를 두 센서에서 중복 융합 |
| `vy=0` 제약 미사용 | 코너에서 옆으로 미끄러지는 궤적 | 차량 비홀로노믹 특성이 EKF에 반영되지 않음 |

### Node 3: 글로벌 필터 (`ekf_node` - Global)

*   **목적:** GNSS 절대 위치로 로컬 필터의 장기 드리프트를 보정하고, 맵 상의 절대 위치를 파악.
*   **입력:** `/wheel/odom`, `/imu/data`, `/odometry/gnss` (Node 1 출력 — UTM 위치 + azimuth yaw)
*   **출력:** `/odometry/global` 토픽, `map` $\rightarrow$ `odom` TF 발행

#### 글로벌 EKF — 각 입력이 필요한 이유 (예측/보정 단계)

EKF는 **예측(Prediction)** + **보정(Correction)** 2단계로 동작한다.

| 입력 | 단계 | 역할 | 없으면? |
| :--- | :--- | :--- | :--- |
| `/wheel/odom(vx, vy=0)` + `/imu/data(wz)` | 예측 | GNSS 업데이트(30Hz) 사이 구간에서 차량 이동을 물리 모델로 추정 | GNSS가 없는 구간(1/30초)마다 위치를 전혀 모름 |
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

→ utm_to_odometry
  yaw_enu = π/2 − bearing
  yaw_ros = −yaw_enu
  quaternion(qz, qw)

→ /odometry/gnss.pose.pose.orientation
  robot_localization이 quaternion에서 yaw 추출
```

global EKF에서 yaw를 쓰는 이유는 절대 heading을 보정하기 위해서다. `/wheel/odom(vx, vy=0)`와 `/imu/data(wz)`만 있으면 global EKF도 local EKF처럼 상대 적분만 수행한다. 위치 x, y를 GNSS로 보정하더라도 yaw가 틀어져 있으면 다음 예측 단계에서 진행 방향이 잘못되어 위치 보정과 예측이 서로 싸우게 된다. 특히 코너 구간에서는 GNSS 위치 업데이트마다 경로가 흔들리거나, `map→odom` 보정이 불안정해질 수 있다. dual GNSS yaw를 함께 넣으면 위치와 방향이 같은 절대 좌표계에서 동시에 보정된다.

#### `/azimuth_angle`을 `pose.pose.orientation` yaw 대신 직접 쓸 수 있는가

`/azimuth_angle`을 global EKF에 직접 넣을 수는 없다. 이유는 `robot_localization`이 `std_msgs/Float64` heading 토픽을 직접 입력으로 받지 않기 때문이다. `robot_localization`이 yaw pose 측정으로 받아들일 수 있는 형태는 대표적으로 다음과 같다.

| 입력 형태 | yaw 전달 방식 | 현재 구조에서의 적합성 |
| :--- | :--- | :--- |
| `nav_msgs/Odometry` | `pose.pose.orientation` quaternion | 현재 사용 중, 가장 적합 |
| `geometry_msgs/PoseWithCovarianceStamped` | `pose.pose.orientation` quaternion | 가능하지만 위치와 yaw를 별도 메시지로 나누게 됨 |
| `sensor_msgs/Imu` orientation | `orientation` quaternion | IMU orientation처럼 보이므로 dual GNSS heading 의미가 흐려질 수 있음 |
| `std_msgs/Float64` | scalar degree/radian | `robot_localization` 입력으로 직접 사용 불가 |

따라서 `/azimuth_angle`을 "대체"하려면 raw Float64를 그대로 넣는 것이 아니라, 별도 브리지 노드에서 quaternion orientation과 covariance를 가진 `Odometry` 또는 `PoseWithCovarianceStamped`로 변환해야 한다. 현재 `utm_to_odometry.py`가 이미 이 역할을 수행한다.

둘 중 무엇이 더 좋은가? 현재 구조에서는 **`/azimuth_angle`을 `/odometry/gnss.pose.pose.orientation` quaternion으로 변환해서 global EKF에 넣는 방식이 더 좋다.** 이유는 다음과 같다.

| 비교 항목 | `/azimuth_angle` 직접 사용 | `/odometry/gnss.pose.pose.orientation` 사용 |
| :--- | :--- | :--- |
| `robot_localization` 호환성 | 직접 입력 불가 | 바로 입력 가능 |
| 좌표계 변환 | EKF 밖에서 따로 처리 필요 | `utm_to_odometry.py`에서 일관 처리 |
| 단위 | degree, N=0, CW+ | quaternion, ROS yaw 기준 |
| covariance | Float64에 없음 | `pose.covariance[35]`로 yaw 신뢰도 지정 가능 |
| position과 heading 동기화 | 별도 관리 필요 | 하나의 `/odometry/gnss` 메시지로 함께 전달 |

즉 `/azimuth_angle`은 좋은 원천 데이터이고, global EKF에는 그것을 ROS 좌표계 quaternion yaw로 변환한 `/odometry/gnss.pose.pose.orientation`을 넣는 것이 정답에 가깝다.

#### 왜 `odom → base_link` 대신 `map → odom` TF를 발행하는가

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

`map → odom` 오프셋을 조정하면:

```text
GNSS 보정 발생
→ map → odom 오프셋만 조용히 변경됨
→ odom → base_link는 전혀 변하지 않음 (로컬 EKF가 계속 부드럽게 발행 중)
→ MPPI는 아무것도 감지하지 못함 → 안정적 제어
→ 전역 경로 추종 노드만 map → base_link를 새로 계산하여 장거리 오차 보정
```

**전체 TF 관계 요약:**

```text
map ──[글로벌 EKF]──> odom ──[로컬 EKF]──> base_link
     (절대 위치 오프셋)       (부드러운 이동)

map → base_link = (map→odom) + (odom→base_link)
                   ^글로벌EKF    ^로컬EKF

전역 경로 추종: map → base_link 사용 (절대 위치 기반)
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
    
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom              # 기준 프레임을 odom으로 설정

    # Wheel Encoder 설정 (X축 선속도 + Y축 비홀로노믹 제약 vy=0)
    odom0: /wheel/odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    # IMU 설정 (Z축 각속도만 사용 / orientation은 dual GNSS azimuth로 대체)
    # launch에서 리매핑: /imu/data <- /carla/car/imu/data
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_queue_size: 10
    imu0_nodelay: true
    imu0_differential: false
    imu0_relative: false

global_ekf:
  ros__parameters:
    use_sim_time: true              # /clock(CARLA simulation time) 사용
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
                   true,  true,  false,
                   false, false, false,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    # launch에서 리매핑: /imu/data <- /carla/car/imu/data
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
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

구현 워크스페이스: `/home/hannibal/carla/mppi/`

```text
mppi/
├── src/
│   ├── gnss_to_utm/             ← mppi 내부 독립 소스 패키지
│   │                                (f9r_to_utm, f9p_to_utm, azimuth_angle_calculator_node)
│   └── dual_filter/
│       ├── package.xml             ament_python 패키지 메타데이터
│       ├── setup.py / setup.cfg
│       ├── dual_filter/
│       │   └── utm_to_odometry.py  ← Node 1d: 브리지 노드 (구현 완료)
│       ├── config/
│       │   └── ekf_params.yaml     ← Section 5 파라미터 (local_ekf + global_ekf)
│       └── launch/
│           └── dual_filter.launch.py ← 전체 시스템 런치 파일
├── build/
├── install/
└── log/
```

### 6.2 `utm_to_odometry` 노드

파일: `dual_filter/dual_filter/utm_to_odometry.py`

* **역할:** Node 1의 최종 브리지 — UTM 위치와 방위각을 하나의 `nav_msgs/Odometry`로 묶어 글로벌 EKF에 전달.
* **구독:**
  * `/f9r_utm` (`geometry_msgs/PointStamped`) — f9r의 UTM easting/northing
  * `/azimuth_angle` (`std_msgs/Float64`) — geographic bearing, **도°**, N=0 CW+
* **발행:** `/odometry/gnss` (`nav_msgs/Odometry`)
  * `header.frame_id = "map"`, `child_frame_id = "base_link"`
  * `pose.pose.position.x` = `easting - datum_easting`
  * `pose.pose.position.y` = `-(northing - datum_northing)` — CARLA `+Y=right`를 ROS `+Y=left`로 미러링
  * `pose.pose.orientation` = azimuth → ENU yaw 변환 → yaw 부호 반전 후 쿼터니언
* **공분산 설정 (robot_localization 가중치 제어):**

| 요소 | 인덱스 (6×6 행렬) | 설정값 | 근거 |
| :--- | :--- | :--- | :--- |
| xx (위치 x) | [0] | 0.01 m² | RTK 정확도 ~10 cm |
| yy (위치 y) | [7] | 0.01 m² | RTK 정확도 ~10 cm |
| yaw | [35] | 0.05 rad² | 듀얼 GNSS 1.4 m 기선의 heading을 쓰되, 회전 중 과신하지 않도록 완화 |
| z, roll, pitch (미사용) | [14],[21],[28] | 1e9 | 높은 값 → EKF가 해당 측정값 무시 |

### 6.3 `ros2_sensor.py` 센서 브리지 노드

파일: `PythonAPI/examples/ros2_sensor/ros2_sensor.py`

이 노드는 CARLA 센서 데이터를 ROS 2 토픽으로 발행하고, EKF 입력에 필요한 `/wheel/odom`과 `/clock`도 함께 만든다.

#### `/clock`

| 항목 | 내용 |
| :--- | :--- |
| 토픽 | `/clock` |
| 타입 | `rosgraph_msgs/Clock` |
| 원천 | `vehicle.get_world().get_snapshot().timestamp.elapsed_seconds` |
| 목적 | ROS 전체를 CARLA simulation time 기준으로 동작시킴 |

CARLA synchronous/passive 환경에서는 simulation time과 wall time이 다를 수 있다. 이때 속도는 simulation second 기준인데 EKF가 wall time으로 적분하면 회전과 이동량이 과적분된다. 따라서 `/clock`을 발행하고 EKF, path publisher, RViz를 `use_sim_time:=true`로 실행한다.

#### `/wheel/odom`

| 필드 | 값 | EKF 사용 여부 |
| :--- | :--- | :--- |
| `header.stamp` | CARLA simulation timestamp | 사용 |
| `header.frame_id` | `odom` | 참조 프레임 |
| `child_frame_id` | `base_link` | twist 프레임 |
| `twist.twist.linear.x` | CARLA world velocity를 차량 전방축으로 투영한 `vx` | 사용 |
| `twist.twist.linear.y` | `0.0` | 사용, 비홀로노믹 제약 |
| `twist.twist.angular.z` | 발행하지 않음 | 사용 안 함 |

`twist.covariance[0] = 0.05`로 `vx` 신뢰도를 지정하고, `twist.covariance[7] = 0.01`로 `vy=0` 제약을 비교적 강하게 준다. yaw-rate는 IMU에서만 사용하므로 `/wheel/odom`의 angular 축 covariance는 크게 둔다.

#### `/carla/car/imu/data`

| 필드 | 값 | EKF 사용 여부 |
| :--- | :--- | :--- |
| `header.stamp` | CARLA IMU timestamp | 사용 |
| `angular_velocity.z` | `-imu.gyroscope.z` | 사용 |
| `orientation` | 채우지 않음, `orientation_covariance[0] = -1` | 사용 안 함 |
| `linear_acceleration` | CARLA acceleration을 ROS Y-left로 변환 | 현재 EKF에서는 사용 안 함 |

CARLA는 `X=front, Y=right, Z=up`이고 ROS `base_link`는 `X=front, Y=left, Z=up`이다. 따라서 yaw-rate와 Y축 성분은 부호를 반전한다.

### 6.4 `dual_filter.launch.py` 토픽 리매핑 요약

파일: `dual_filter/launch/dual_filter.launch.py`

| 노드 | 내부 토픽 (코드 하드코딩) | 실제 CARLA 토픽 | 리매핑 방법 |
| :--- | :--- | :--- | :--- |
| `f9r_to_utm` | `/f9r/fix` | `/carla/car/f9r/fix` | `remappings=` |
| `f9p_to_utm` | `/f9p/fix` | `/carla/car/f9p/fix` | `remappings=` |
| `azimuth_angle_calculator` | `gnss1_topic`, `gnss2_topic` | `/carla/car/f9r/fix`, `/f9p/fix` | `parameters=` (파라미터 오버라이드) |
| `local_ekf` | `/imu/data` | `/carla/car/imu/data` | `remappings=` |
| `global_ekf` | `/imu/data` | `/carla/car/imu/data` | `remappings=` |
| `local_ekf` | `odometry/filtered` | `/odometry/local` | `remappings=` |
| `global_ekf` | `odometry/filtered` | `/odometry/global` | `remappings=` |

> **`/wheel/odom`** 은 `ros2_sensor.py`가 직접 `/wheel/odom`으로 발행하므로 리매핑 불필요.
> 이 토픽은 **전진 선속도 + 비홀로노믹 제약 입력**으로 사용한다. `twist.twist.angular.z`는 `/imu/data.angular_velocity.z`와 중복되므로 EKF에서 사용하지 않으며, `ros2_sensor.py`에서도 yaw-rate covariance를 크게 설정해 회전 입력으로 선택되지 않게 한다.
> `header.stamp`는 ROS wall time이 아니라 CARLA simulation timestamp를 사용한다. `/imu/data`, `/odometry/gnss`도 같은 시간 기준을 사용해야 local/global EKF가 속도를 올바른 시간 간격으로 적분한다.
> 따라서 `ros2_sensor.py`는 `/clock`을 발행하고, dual filter launch의 모든 노드는 `use_sim_time:=true`로 실행한다.

### 6.5 Path 출력

`odom_path_publisher`는 Odometry 메시지를 누적하여 RViz용 `nav_msgs/Path`를 발행한다.

| 출력 Path | 입력 Odometry | Path frame | 의미 |
| :--- | :--- | :--- | :--- |
| `/path/odom` | `/odometry/local` | `odom` | GNSS 없이 wheel+IMU만 적분한 부드러운 odom-frame dead-reckoning 궤적 |
| `/path/gnss` | `/odometry/gnss` | `map` | EKF를 거치지 않은 GNSS 위치와 dual GNSS yaw 기반 절대 궤적 |
| `/path/global_ekf` | `/odometry/global` | `map` | wheel+IMU+GNSS를 융합한 global EKF 추정 궤적 |

세 Path의 이름은 의미를 분리하기 위해 명확하게 둔다.

| RViz 표시 이름 | 토픽 | 해석 |
| :--- | :--- | :--- |
| Odom Path | `/path/odom` | local EKF dead-reckoning 결과 |
| GNSS Path | `/path/gnss` | dual GNSS 기반 절대 궤적 |
| Global EKF Path | `/path/global_ekf` | global EKF 융합 결과 |

`/path/odom`은 제어 안정성 확인용이고, `/path/gnss`는 GNSS 변환 결과가 CARLA 주행 궤적과 맞는지 확인하는 전역 기준 궤적이다. `/path/global_ekf`는 global EKF가 GNSS 원천 궤적을 얼마나 부드럽게 따라가며 `map→odom` 보정을 만드는지 확인하는 용도이다. RViz는 `use_sim_time:=true`로 실행해야 Path와 TF가 같은 시간축에서 표시된다.

---

## 7. 실행 방법

### 7.1 사전 준비

```bash
# robot_localization 패키지 설치 (최초 1회)
sudo apt install ros-humble-robot-localization

# mppi 워크스페이스 빌드 (최초 1회 또는 소스 수정 후)
cd ~/carla/mppi
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 7.2 실행 순서

#### 터미널 1 — CARLA 시뮬레이터 서버

```bash
cd ~/carla
source /opt/ros/humble/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ./CarlaUE4.sh -RenderOffScreen -quality-level=Low --ros2
```

#### 터미널 2 — 수동 조종 차량 (맵 로드 및 차량 스폰)

```bash
cd ~/carla
source .venv/bin/activate

python PythonAPI/util/config.py --no-rendering
python PythonAPI/util/config.py --map Town01_Opt

python PythonAPI/examples/manual_control.py \
  --rolename car \
  --filter vehicle.micro.microlino \
  --generation 2 \
  --sync
```

#### 터미널 3 — ROS2 센서 브리지 (`/wheel/odom` 포함)

```bash
cd ~/carla
source /opt/ros/humble/setup.bash
source .venv/bin/activate

python PythonAPI/examples/ros2_sensor/ros2_sensor.py \
  -f PythonAPI/examples/ros2_sensor/stack.json \
  --attach-existing \
  --passive \
  --python-ros2 \
  --base-frame base_link \
  --wait-for-vehicle 30
```




> **주의:** `--python-ros2` 플래그가 없으면 `static TF(base_link → 각 센서)`가 발행되지 않아 RViz에서 렌더링이 되지 않는다.
> 또한 이 노드가 CARLA simulation timestamp와 `/clock`을 발행하므로, dual filter와 RViz보다 먼저 실행하는 것이 가장 안전하다.

원하는 센서만 선택할 경우 `--sensors` 플래그에 아래 ID를 공백으로 나열한다.

| 센서 ID | 타입 | 발행 토픽 | 비고 |
| :--- | :--- | :--- | :--- |
| `rgb` | `sensor.camera.rgb` | `/carla/car/rgb/image` | RGB 카메라 |
| `lidar` | `sensor.lidar.ray_cast` | `/carla/car/lidar/point_cloud` | 3D LiDAR (64채널) |
| `lidar_2d` | `sensor.lidar.ray_cast` | `/carla/car/lidar_2d/point_cloud` | 2D LiDAR (1채널) |
| `f9r` | `sensor.other.gnss` | `/carla/car/f9r/fix` | GNSS 후륜축 — azimuth 기준점 |
| `f9p` | `sensor.other.gnss` | `/carla/car/f9p/fix` | GNSS 전방 1.4m — azimuth 벡터 끝점 |
| `imu` | `sensor.other.imu` | `/carla/car/imu/data` | 6-DOF IMU |

```bash
# 예시: 듀얼 필터에 필요한 센서만 활성화

cd ~/carla
source /opt/ros/humble/setup.bash
source .venv/bin/activate

python PythonAPI/examples/ros2_sensor/ros2_sensor.py \
  -f PythonAPI/examples/ros2_sensor/stack.json \
  --attach-existing \
  --passive \
  --python-ros2 \
  --base-frame base_link \
  --wait-for-vehicle 30 \
  --sensors f9r f9p imu
```

#### 터미널 4 — 듀얼 필터 전체 실행

```bash
source /opt/ros/humble/setup.bash
source ~/carla/mppi/install/setup.bash
ros2 launch dual_filter dual_filter.launch.py
```

#### 터미널 5 — RViz (선택)

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/carla/PythonAPI/examples/ros2_sensor/rviz/ros2_sensor.rviz --ros-args -p use_sim_time:=true
```

### 7.3 종료

```bash
pkill -TERM -f 'ros2_sensor.py'
pkill -TERM -f 'manual_control.py'
pkill -TERM -f 'rviz2'
pkill -TERM -f 'CarlaUE4-Linux-Shipping'
```

### 7.4 동작 확인

```bash
# sim time 확인
ros2 topic echo --once /clock
ros2 param get /local_ekf use_sim_time
ros2 param get /global_ekf use_sim_time

# 주요 입력 stamp가 /clock과 같은 CARLA simulation time인지 확인
ros2 topic echo --once /wheel/odom --field header.stamp
ros2 topic echo --once /carla/car/imu/data --field header.stamp
ros2 topic echo --once /odometry/local --field header.stamp
ros2 topic echo --once /odometry/global --field header.stamp

# TF 트리 확인 (map → odom → base_link 구조인지 확인)
ros2 run tf2_tools view_frames

# wheel/IMU 입력 성분 확인
ros2 topic echo --once /wheel/odom --field twist.twist.linear
ros2 topic echo --once /carla/car/imu/data --field angular_velocity

# 로컬 EKF 출력 확인 (MPPI 제어 입력용 — GNSS jump 없이 부드러움)
ros2 topic echo /odometry/local

# 글로벌 EKF 출력 확인 (map → odom 보정용)
ros2 topic echo /odometry/global

# GNSS 브리지 출력 확인 (/path/gnss의 원천: UTM 좌표 + azimuth yaw)
ros2 topic echo /odometry/gnss
```

### 7.5 전체 데이터 흐름

```text
CARLA Simulator
  ├─ sim time ─────────────→ ros2_sensor.py ──→ /clock ──→ use_sim_time nodes
  │
  ├─ /carla/car/imu/data  ──────────────────────────────────────────────┐
  │                                                                     │
  ├─ /carla/car/f9r/fix ──→ f9r_to_utm ──→ /f9r_utm ──┐                 │
  │                     └──→ azimuth_calc ──→ /azimuth_angle ──┐        │
  │                                                             │       │
  ├─ /carla/car/f9p/fix ──→ f9p_to_utm ──→ /f9p_utm            │        │
  │                                                             ▼       │
  └─ ros2_sensor.py ──→ /wheel/odom ──────────── utm_to_odometry        │
                              │                       │                 │
                              │                       ▼                 │
                              │                 /odometry/gnss           │
                              │                       │                 │
                              ├───────────────────────┼─────────────────┤
                              │                       │                 │
                              ▼                       ▼                 ▼
                         local_ekf ◄──── /wheel/odom(vx, vy=0) + /imu/data(wz)
                         global_ekf ◄─── /wheel/odom(vx, vy=0) + /imu/data(wz) + /odometry/gnss
                              │                       │
                              ▼                       ▼
                    /odometry/local           /odometry/global
                    odom → base_link TF       map → odom TF
                    (MPPI 제어용)              (전역 보정용)

                         /odometry/local  ─→ /path/odom
                         /odometry/gnss    ─→ /path/gnss
                         /odometry/global ─→ /path/global_ekf
```

---

## 8. Nav2 MPPI Controller에 필요한 전체 입력과 현재 충족 여부

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
| 현재 차량 pose | `geometry_msgs/PoseStamped robot_pose` | `controller_server`가 TF/costmap으로 계산 | 충족 가능. `map→odom→base_link` TF가 있음 | Nav2 frame을 `global_frame: map`, `robot_base_frame: base_link` 기준으로 맞춤 |
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
| `/wheel/odom` | 부분 가능 | 전방 속도 `linear.x`가 직접적이고 지연이 작음 | yaw rate를 쓰지 않도록 만든 topic이라 `angular.z`가 부정확하거나 0이 될 수 있음 | 비권장 |
| `/odometry/global` | 가능은 함 | global EKF 융합 결과 | GNSS 보정 영향이 섞이며 제어용 현재 속도에는 불필요 | 비권장 |
| `/odometry/gnss` | 부적합 | 절대 위치와 dual GNSS yaw가 있음 | `utm_to_odometry`는 pose 브리지이며 twist를 제공하지 않음 | 사용 금지 |

권장 설정:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    odom_topic: /odometry/local
    odom_duration: 0.3
```

`/odometry/local`은 `/wheel/odom.twist.twist.linear.x`, `linear.y=0`, `/imu/data.angular_velocity.z`를 EKF로 융합하므로 MPPI의 `robot_speed` 입력으로 가장 안정적이다. 또한 `/odometry/local`은 GNSS 보정을 받지 않으므로 제어 루프에 GNSS jump를 전달하지 않는다.

### 8.4 Pose와 TF 요구사항

MPPI의 `robot_pose`는 odometry topic의 pose가 아니라 controller server/costmap/TF 경로에서 들어온다. 따라서 다음 TF가 반드시 살아 있어야 한다.

```text
map ──> odom ──> base_link
```

| TF | 발행 주체 | MPPI 관점에서 필요한 이유 | 현재 충족 여부 |
| :--- | :--- | :--- | :--- |
| `odom → base_link` | `local_ekf` | local frame에서 차량 pose를 연속적으로 제공 | 충족 |
| `map → odom` | `global_ekf` | global path/map frame과 local odom frame 연결 | 충족 |
| `map → base_link` | TF 합성 결과 | global plan을 local control frame으로 변환할 때 필요 | 위 두 TF가 있으면 충족 |

Nav2 costmap frame 설정은 보통 다음 구성이 자연스럽다.

| Nav2 frame parameter | 권장값 | 이유 |
| :--- | :--- | :--- |
| `global_frame` | `map` 또는 local costmap에서는 `odom` | global planner/path와 local controller 구성에 따라 선택 |
| `robot_base_frame` | `base_link` | CARLA 차량 기준 프레임 |
| `transform_tolerance` | `0.1` 이상에서 시작 | sim time/TF 지연을 흡수 |

중요한 점은 `robot_pose`와 `transformed_global_plan`이 같은 local control 기준에서 일관되게 계산되어야 한다는 것이다.

### 8.5 Path와 Goal 요구사항

MPPI는 path를 직접 만들지 않는다. `controller_server`가 FollowPath action으로 받은 `nav_msgs/Path`를 path handler로 자르고 변환한 뒤 MPPI에 넘긴다.

| 요구사항 | 필요 메시지/객체 | 현재 dual filter가 제공? | 추가 필요 |
| :--- | :--- | :--- | :--- |
| 추종할 global path | `nav_msgs/Path` | 아니오. `/path/gnss`, `/path/global_ekf`는 시각화용 주행 궤적임 | planner 또는 별도 path publisher |
| path frame | 보통 `map` | 가능 | path header frame과 TF tree 일치 필요 |
| 최종 goal | path 마지막 pose 또는 action goal | 아니오 | FollowPath action goal 제공 |
| transformed local plan | controller server 내부 생성 | Nav2가 생성 | TF와 path가 정상이어야 함 |

주의: `/path/odom`, `/path/gnss`, `/path/global_ekf`는 RViz에서 실제 주행 궤적을 확인하기 위한 출력이다. 이들은 “따라가야 할 계획 경로”가 아니라 “이미 지나온 경로 기록”이므로 MPPI의 global plan으로 넣으면 안 된다.

### 8.6 Costmap과 장애물 정보

MPPI는 critics를 통해 trajectory cost를 계산하고, obstacle 관련 critic은 local costmap을 사용한다. 따라서 장애물 회피까지 하려면 local costmap이 필요하다.

| 요구사항 | 현재 파이프라인 충족 여부 | 추가 필요 |
| :--- | :--- | :--- |
| local costmap | 별도 설정 필요 | Nav2 local_costmap 구성 |
| obstacle layer 입력 | 가능성 있음 | `/carla/car/lidar/point_cloud` 또는 2D LiDAR topic을 costmap observation source로 연결 |
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
  motion_model: "ackermann"
  ackermann:
    plugin: "mppi::AckermannMotionModel"
    min_turning_r: 3.0
```

### 8.8 MPPI 주요 파라미터

| 파라미터 | 의미 | 현재 시스템에서의 주의점 |
| :--- | :--- | :--- |
| `controller_frequency` | controller server loop 주파수 | `/odometry/local` publish rate보다 낮거나 같게 시작. 예: 30Hz |
| `model_dt` | rollout time step | 코드상 `controller_period <= model_dt`이어야 함. 같게 두는 것을 권장 |
| `time_steps` | rollout 길이 | `time_steps × model_dt`가 예측 horizon |
| `batch_size` | 샘플 trajectory 수 | 클수록 품질↑, CPU 비용↑ |
| `vx_max`, `vx_min` | 전후진 속도 제한 | CARLA 차량 속도와 안전 한계에 맞춤 |
| `wz_max` | yaw rate 제한 | 실제 조향 한계와 맞춤 |
| `ax_max`, `ax_min`, `az_max` | 가속/감속 및 yaw 가속 제한 | 너무 크면 명령이 거칠어짐 |
| `vx_std`, `wz_std` | 샘플링 분산 | 조향 chatter가 있으면 `wz_std`를 줄여봄 |
| `open_loop` | 현재 odometry 대신 이전 command 기반 예측 | 기본은 `false` 권장. odom latency가 심할 때만 검토 |
| `critics` | trajectory 평가 항목 | path/goal/obstacle critic을 목적에 맞게 선택 |

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

| 항목 | 필요 여부 | 현재 제공 topic/구성 | 상태 | 다음 작업 |
| :--- | :--- | :--- | :--- | :--- |
| 현재 속도 odometry | 필수 | `/odometry/local` | 충족 | `controller_server.odom_topic`에 지정 |
| 연속 TF | 필수 | `odom→base_link` | 충족 | local EKF 유지 |
| 전역 보정 TF | 필수에 가까움 | `map→odom` | 충족 | global EKF 유지 |
| 현재 pose | 필수 | TF 합성 `map/odom→base_link` | 충족 가능 | Nav2 frame 설정 필요 |
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
2. Nav2 TF frame: map/odom/base_link 일치
3. FollowPath에 넣을 계획 경로 생성
4. local costmap 구성
5. motion_model = ackermann 및 제약 튜닝
6. cmd_vel을 CARLA VehicleControl로 변환
```

---

## 9. 실제 하드웨어 휠 오도메트리 구현

CARLA 시뮬레이션에서는 `ros2_sensor.py`가 `/wheel/odom`을 직접 발행한다(Section 6.3). 실제 하드웨어에서는 이 역할을 `serial_bridge` 노드가 대신한다. 아두이노가 전륜 엔코더와 POT 조향각 센서 데이터를 시리얼로 전송하고, `serial_bridge`가 이를 파싱하여 자전거 모델 보정을 적용한 뒤 `/wheel/odom`으로 발행한다.

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

파일: `mppi/src/serial_bridge/serial_bridge/serial_bridge.py`

패키지: `mppi/src/serial_bridge/`

#### 역할

| 방향 | 동작 |
| :--- | :--- |
| `/auto_throttle` → 아두이노 | Float32 수신 → `TH <val>\n` 시리얼 전송 |
| `/auto_steer_angle` → 아두이노 | Float32 수신 → `SA <val>\n` 시리얼 전송 |
| 아두이노 → `/wheel/odom` | 시리얼 수신 → VX/PS 파싱 → 보정 → Odometry 발행 |

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
  /wheel/odom.twist.twist.linear.x = 0.5068
  /wheel/odom.twist.twist.linear.y = 0.0   (비홀로노믹 제약)
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

### 9.4 CARLA 시뮬레이션과 실제 하드웨어의 `/wheel/odom` 비교

| 항목 | CARLA 시뮬레이션 (ros2_sensor.py) | 실제 하드웨어 (serial_bridge) |
| :--- | :--- | :--- |
| vx 원천 | CARLA world velocity → base_link 투영 | 전륜 엔코더 펄스 → 선속도 변환 |
| 조향 보정 | 불필요 (CARLA가 직접 후륜축 기준 속도 제공) | 필요: v_rear = v_encoder × cos(δ) |
| 조향각 원천 | 없음 | POT 측정 실측값 (raw_deg, PS) |
| 타임스탬프 | CARLA simulation time (/clock) | ROS2 wall time (get_clock().now()) |
| 발행 노드 | `ros2_sensor.py` | `serial_bridge` |
| 패키지 위치 | `PythonAPI/examples/ros2_sensor/` | `mppi/src/serial_bridge/` |

> **주의:** 실제 하드웨어 실행 시 `use_sim_time: false`로 설정해야 한다. CARLA 시뮬레이션에서만 `use_sim_time: true`를 사용한다. 두 환경을 혼합하면 EKF 적분 시간 오류가 발생한다.

### 9.5 패키지 의존성

`mppi/src/serial_bridge/package.xml`:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>nav_msgs</depend>   <!-- Odometry 메시지 타입 -->
```

빌드:

```bash
cd ~/carla/mppi
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
