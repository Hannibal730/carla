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
`map` $\rightarrow$ `odom` $\rightarrow$ `base_link`

1.  **`base_link`:** 차량의 뒷바퀴 축 중심(또는 무게중심)을 나타내는 로컬 물리 프레임.
2.  **`odom`:** 시작 위치를 원점으로 하는 로컬 기준 프레임. 연속적이고 부드러운 궤적을 보장하나 장기 드리프트(Drift)가 존재함. (단기 제어 및 장애물 회피용)
3.  **`map`:** GNSS 기반의 전역 기준 프레임. 위치의 도약(Jump)이 발생할 수 있으나 지구상의 절대 위치를 보장함. (전역 경로 추종용)

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
