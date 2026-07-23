# 자동주차 기능 실차 구동 저장소 이식 가이드

> 기준 저장소: `/home/dolbat/carla`  
> 기준 환경: ROS 2 Humble, Navigation2 1.1.20, Ackermann 차량  
> 기준 커밋: `2c91f23` + 작성 시점 working tree의 주차 파라미터 변경  
> 작성 목적: 현재 CARLA 저장소의 자동주차 기능을 실제 차량 구동 저장소로 옮길 때 필요한 소스, 설정, 인터페이스, 현장 좌표, 안전 변경 사항을 한 문서로 고정한다.

이 문서는 현재 소스 트리를 정적으로 분석해 작성했다. 이 문서를 작성하는 과정에서는 패키지 빌드와 런타임 검증을 수행하지 않았다.

---

## 1. 이식 결과의 목표

실차 저장소에서 다음 흐름이 끊기지 않게 만드는 것이 이식의 완료 조건이다.

```text
후륜축 GNSS UTM 위치 ───────────────────────────────────────────────┐
                                                                  │
2D LiDAR + 측정 시각의 map TF                                     │
  │                                                               │
  ▼                                                               ▼
zone_scan ── 주차구역 ROI 누적/ICP ──> point_parking ──> parking
  │                                      │                │
  │ /parkingScan, /parkingMode           │ 빈 공간 goal   │ UTM→map
  │ global costmap용 장애물              │                ▼
  └────────────────────────────────────────────> /point_parking/nav_goal
                                                        │
                                                        ▼
                                              follow_path_client
                                              ├─ Smac 경로 계획
                                              ├─ 후진 경로 반전
                                              ├─ ParkingPath MPPI
                                              ├─ Zone별 추가 후진
                                              └─ 전진 출차
                                                        │
                                                        ▼
                                                   /cmd_vel
                                                        │
                                                        ▼
                                              실차 종·횡방향 구동기
```

현재 구현은 두 구역을 서로 다르게 처리한다.

| 구역 | 주차 진입 | 주차 후 추가 동작 | 출차 |
|---|---|---|---|
| `ParkingZone1` | Smac DUBIN 결과를 뒤집어 전 구간 후진 | 고정 UTM Easting까지 직선 후진 | 1차 goal까지 직선 전진한 후 Gate A까지 DUBIN 전진 |
| `ParkingZone2` | Smac DUBIN 결과를 뒤집어 전 구간 후진 | 추가 후진 생략 | Gate B 지정점까지 DUBIN 전진 |

---

## 2. 이식 전에 반드시 결정할 항목

아래 항목은 단순 복사로 결정할 수 없다. 실차 저장소의 실제 인터페이스에 맞게 먼저 확정해야 한다.

| 항목 | 현재 구현의 계약 | 실차에서 결정할 내용 |
|---|---|---|
| ROS 시간 | launch 기본값 또는 강제값이 `use_sim_time: true` | 실차는 일반적으로 `false`. 외부 시뮬레이션 clock을 쓰는 경우만 `true` |
| 전역 좌표 | 절대 UTM + 최초 GNSS fix datum | 동일 UTM zone 사용 여부, datum 공급 노드 |
| TF | `map → odom → base_link → lidar_2d` | 실차 localization이 같은 트리를 발행하도록 구성 |
| 기준점 | `base_link`가 후륜축 중심 | 실차 기준점이 다르면 footprint, GNSS lever arm, 경로 목표 모두 보정 |
| GNSS 위치 | `/f9p_utm`, `PointStamped`, 절대 UTM | 실차 GNSS 드라이버 출력 또는 변환 노드 토픽 |
| LiDAR | Reliable `PointCloud2`, 정확한 timestamp와 frame | 실차 토픽, QoS, frame, 시간 동기화 |
| 차량 명령 | `/cmd_vel.linear.x`의 음수는 후진 | 실차 DBW가 음수 속도를 지원하는지, 기어 명령이 별도인지 |
| 안전 정지 | CARLA 브리지에는 실차용 watchdog/E-stop이 없음 | 명령 timeout, gear interlock, E-stop, brake fallback을 별도 구현 |
| 기존 경로 복귀 | 주차/출차 후 `/csv_path` 추종으로 자동 복귀 | 실차 mission manager로 복귀할지, CSV 기능을 유지할지 |

이 가운데 하나라도 미정이면 actuator에 연결하지 말고 경로와 `/cmd_vel`까지만 shadow mode로 확인해야 한다.

---

## 3. 소스 파일 이식 목록

### 3.1 반드시 통째로 옮길 패키지

다음 디렉터리는 패키지 단위로 그대로 옮긴다.

```text
mppi_ws/src/auto_parking/
├── auto_parking/
│   ├── __init__.py
│   ├── zone_scan.py
│   ├── point_parking.py
│   ├── parking.py
│   └── parkingZone_visualizer.py
├── config/
│   ├── zone_scan.yaml
│   ├── parking_mode_gates.yaml
│   ├── point_parking.yaml
│   └── parking.yaml
├── docs/
├── launch/
│   ├── auto_parking.launch.py
│   ├── zone_scan.launch.py
│   ├── point_parking.launch.py
│   ├── parking.launch.py
│   └── parking_zone_visualizer.launch.py
├── resource/
│   ├── auto_parking
│   └── parking_Zone
├── rviz/zone_scan.rviz
├── package.xml
├── setup.cfg
└── setup.py
```

다음 생성물은 복사하지 않는다.

```text
__pycache__/
*.pyc
.pytest_cache/
build/
install/
log/
```

`resource/parking_Zone`은 일반 문서가 아니라 실행 시 파싱되는 현장 설정 파일이다. `setup.py`의 `data_files`에도 등록되어 있으므로 반드시 함께 옮겨야 한다.

### 3.2 `dual_filter`에서 옮기거나 실차 패키지로 흡수할 파일

| 원본 | 역할 | 이식 방법 |
|---|---|---|
| `mppi_ws/src/dual_filter/dual_filter/follow_path_client.py` | CSV/주차/최종 후진/출차 상태 머신 | 그대로 옮긴 후 실차 mission 흐름에 맞게 이름과 복귀 동작 수정 |
| `mppi_ws/src/dual_filter/config/nav2_carla_params.yaml` | MPPI, goal checker, Smac, local/global costmap 설정 | 파일명을 실차용으로 바꾸고 차량·센서·시간 파라미터 교체 |
| `mppi_ws/src/dual_filter/launch/controller.launch.py` | controller/planner/lifecycle/mode manager 기동 | `use_sim_time`을 launch 인자로 만들고 실차 bringup에 include |
| `mppi_ws/src/dual_filter/package.xml` | 런타임 의존성 예시 | 대상 패키지 manifest에 필요한 depend 병합 |
| `mppi_ws/src/dual_filter/setup.py` | `follow_path_client` entry point | 대상 Python 패키지의 entry point에 병합 |
| `mppi_ws/src/dual_filter/dual_filter/gnss_to_odom.py` | `/utm_datum`, map 상대 pose 생성 | 실차 localization이 같은 출력을 주지 않을 때만 이식 |

현재 `cmd_vel_to_carla.py`는 실차에 옮기지 않는다. 이것은 CARLA `VehicleControl` 전용 브리지다.

### 3.3 선택 이식 파일

| 파일 | 필요한 경우 |
|---|---|
| `mppi_ws/src/gnss_to_utm/` | 실차 GNSS가 위경도만 발행하고 절대 UTM `/f9p_utm`을 만들 노드가 없을 때 |
| `ros2_sensor/rviz/ros2_sensor.rviz` | 기존 통합 RViz 화면까지 그대로 사용할 때 |
| `mppi_ws/src/auto_parking/rviz/zone_scan.rviz` | 주차 기능만 별도 시각화할 때 |
| `nav2_ws/src/navigation2/` | 대상 시스템의 Nav2/MPPI 버전이 현재 파라미터와 호환되지 않을 때 |

### 3.4 복사 명령 예시

아래 명령은 경로 예시이며 이 문서 작성 중 실행하지 않았다.

```bash
SOURCE_REPO=/path/to/carla
TARGET_WS=/path/to/real_vehicle_ws

cp -a "$SOURCE_REPO/mppi_ws/src/auto_parking" "$TARGET_WS/src/"
```

`dual_filter` 전체를 쓰지 않는 실차 저장소라면 `follow_path_client.py`를 기존 mission/navigation 패키지에 넣고 `setup.py`, `package.xml`, launch만 병합한다. 동일 이름의 파일을 무조건 덮어쓰면 기존 bringup을 잃을 수 있으므로 이 부분은 파일 단위 병합이 원칙이다.

---

## 4. 패키지 의존성

### 4.1 `auto_parking` 런타임 의존성

현재 `package.xml`이 요구하는 패키지는 다음과 같다.

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>rcl_interfaces</exec_depend>
<exec_depend>ament_index_python</exec_depend>
<exec_depend>python3-numpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>sensor_msgs_py</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>visualization_msgs</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>rviz2</exec_depend>
```

RViz를 차량 PC에서 실행하지 않으면 `rviz2`는 운영 이미지가 아니라 개발/모니터링 이미지에만 둘 수 있다. 단, 현재 manifest에는 exec dependency로 들어 있다.

### 4.2 mode manager와 Nav2 의존성

대상 패키지에는 최소한 다음 의존성이 필요하다.

```xml
<depend>rclpy</depend>
<depend>action_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>nav2_controller</exec_depend>
<exec_depend>nav2_planner</exec_depend>
<exec_depend>nav2_smac_planner</exec_depend>
<exec_depend>nav2_lifecycle_manager</exec_depend>
<exec_depend>nav2_mppi_controller</exec_depend>
```

현재 `dual_filter/package.xml`에는 `nav2_mppi_controller`가 직접 적혀 있지 않다. 실차용 manifest에는 명시적으로 추가하는 편이 안전하다.

### 4.3 Nav2 버전 기준

현재 소스 트리의 기준 버전은 `1.1.20`이다.

주차 설정이 실제로 사용하는 기능은 다음과 같다.

- 하나의 controller server에 `FollowPath`, `ParkingPath` 두 MPPI 인스턴스 로드
- `nav2_mppi_controller::MPPIController`
- Ackermann motion model
- 음수 `vx_min`을 이용한 후진 trajectory
- `GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`
- `VelocityDeadbandCritic`
- `PathAlignCritic.use_path_orientations`
- `PathAngleCritic.forward_preference`
- `GoalAngleCritic.symmetric_yaw_tolerance`
- `nav2_smac_planner/SmacPlannerHybrid`의 `DUBIN`
- `ClearEntireCostmap` 서비스
- costmap inflation parameter의 런타임 변경

대상 Nav2가 이 파라미터를 지원하지 않으면 현재 `nav2_ws/src/navigation2`를 호환 기준으로 가져가야 한다. 가장 재현성이 높은 방법은 Navigation2 소스 트리를 같은 버전으로 고정하는 것이다.

현재 `nav2_carla_params.yaml`의 아래 두 설정은 현재 저장소의 `nav2_controller` 소스에서 실제 로딩 코드가 확인되지 않는 잔여 설정이다.

```yaml
path_handler_plugins: ["PathHandler"]
PathHandler:
  plugin: "nav2_controller::FeasiblePathHandler"
```

`FeasiblePathHandler` 구현도 현재 트리에 없다. 주차 이식의 필수 구성으로 취급하지 말고, 대상 Nav2에서 별도 플러그인을 실제 제공하는 경우에만 유지한다. MPPI 내부 `prune_distance`, `transform_tolerance`, `max_robot_pose_search_dist`는 이 설정과 별개다.

---

## 5. 좌표계와 시간 계약

### 5.1 필수 TF 트리

```text
map ──> odom ──> base_link ──> lidar_2d
```

- `map`: UTM datum 상대 전역 좌표
- `odom`: 연속적인 로컬 제어 좌표
- `base_link`: 후륜축 중심
- `lidar_2d`: 실제 LiDAR 측정 원점

controller는 `/odometry/local`을 사용하지만 주차 목표와 Smac global costmap은 `map`을 사용한다. 따라서 `map → odom` TF가 항상 존재해야 한다.

### 5.2 절대 UTM과 map 변환

현재 모든 주차구역, GPS gate, 최종 출차 지점은 절대 UTM으로 저장된다. ROS `map` 좌표 변환은 다음과 같다.

```text
map_x   = UTM_E - datum_E
map_y   = -(UTM_N - datum_N)
map_yaw = -UTM_yaw
```

반대 변환은 다음과 같다.

```text
UTM_E = map_x + datum_E
UTM_N = datum_N - map_y
```

Y와 yaw 부호 반전은 현재 CARLA/ROS 정렬 방식에서 비롯된다. 실차 localization이 표준 ENU(`x=east`, `y=north`, yaw CCW+)를 그대로 쓰면 이 반전을 유지하면 안 된다. 실차 map 정의를 먼저 확인하고 다음 세 파일에서 같은 변환을 일관되게 바꿔야 한다.

- `auto_parking/zone_scan.py`
- `auto_parking/point_parking.py`
- `auto_parking/parking.py`
- `follow_path_client.py`의 출차 UTM→map 변환

한 파일만 바꾸면 ROI, 목표점, 차량 pose가 서로 거울상으로 갈라진다.

### 5.3 datum 계약

`/utm_datum`의 계약은 다음과 같다.

| 항목 | 값 |
|---|---|
| 타입 | `geometry_msgs/msg/PointStamped` |
| `header.frame_id` | `utm` |
| `point.x` | datum Easting |
| `point.y` | datum Northing |
| QoS | Reliable + Transient Local + Keep Last 1 |

현재 `gnss_to_odom.py`는 첫 `/f9p_utm`을 datum으로 래치한다. 실차 저장소가 고정 datum을 이미 쓰면 그 datum을 같은 토픽으로 한 번 latched publish하면 된다.

### 5.4 시간 동기화

`zone_scan`은 LiDAR 메시지의 `header.stamp`에 정확히 대응하는 `map → lidar` TF만 사용한다.

- TF buffer: 30초
- pending LiDAR queue: 최대 50개
- 처리 timer: 0.02초
- 기본 `tf_wait_timeout`: 0.5초
- 정확한 timestamp의 TF가 timeout 안에 없으면 scan 폐기

실차에서 GNSS/localization clock과 LiDAR clock이 다르면 누적점이 사라지거나 잘못 정합된다. NTP/PTP 또는 하드웨어 time synchronization을 먼저 해결해야 하며, timeout을 늘리는 것은 clock offset의 해결책이 아니다.

실차 launch에서는 일반적으로 다음을 사용한다.

```yaml
use_sim_time: false
```

현재 `controller.launch.py`는 `True`를 직접 넣고 있으므로 실차 이식 시 launch argument로 바꿔야 한다.

---

## 6. 외부 인터페이스 계약

### 6.1 필수 입력

| 토픽/TF | 타입 | 소비자 | QoS/조건 | 의미 |
|---|---|---|---|---|
| `/f9p_utm` | `PointStamped` | `zone_scan`, localization | Reliable/Volatile | 후륜축 절대 UTM 위치 |
| `/utm_datum` | `PointStamped` | 세 auto_parking 노드, mode manager | Reliable/Transient Local | map 원점의 절대 UTM |
| LiDAR 토픽 | `PointCloud2` | `zone_scan` | Reliable/Volatile, depth 10 | 2D 장애물 scan |
| `map → lidar frame` | TF | `zone_scan` | LiDAR 측정 timestamp에 존재 | scan의 map 변환 |
| `/odometry/local` | `Odometry` | controller, mode manager | 기본 Reliable | 연속적인 제어용 pose/twist |
| `/odometry/global` | `Odometry` | mode manager | 기본 Reliable | `map` 기준 현재 pose |
| `/csv_path` | `Path` | mode manager | Reliable/Transient Local | 주차 후 복귀할 기존 주행 경로, 선택 사항 |

실차 LiDAR 드라이버가 SensorDataQoS/Best Effort를 쓰면 현재 Reliable subscriber와 연결되지 않을 수 있다. 이 경우 드라이버 QoS 또는 `zone_scan.py`의 `lidar_qos`를 동일하게 맞춘다. global costmap source QoS까지 같은 정책으로 맞춰야 한다.

### 6.2 auto_parking 내부 및 출력 토픽

| 토픽 | 타입 | 발행자 | 주요 소비자 | QoS |
|---|---|---|---|---|
| `/parkingScan` | `Bool` | `zone_scan` | 내부 상태/모니터링 | Transient Local |
| `/parkingMode` | `Bool` | `zone_scan` | `parking` | Transient Local |
| `/parking_zones` | `MarkerArray` | `zone_scan` | RViz | Transient Local |
| `/zone_scan/occupied_points` | `PointCloud2` | `zone_scan` | `point_parking` | Transient Local |
| `/zone_scan/cost_lidar_points` | `PointCloud2` | `zone_scan` | global costmap | Reliable/Volatile |
| `/zone_scan/roi_boundary_points` | `PointCloud2` | `zone_scan` | global costmap | Transient Local |
| `/parking_exit/goal_utm` | `PoseStamped` | `zone_scan` | mode manager | Transient Local |
| `/parking_exit/zone` | `String` | `zone_scan` | mode manager | Transient Local |
| `/point_parking/goal_utm_yaw` | `Float64MultiArray` | `point_parking` | 모니터링 | Transient Local |
| `/point_parking/goal_pose` | `PoseStamped` | `point_parking` | `parking` | Transient Local |
| `/point_parking/goal_candidates` | `PoseArray` | `point_parking` | RViz/모니터링 | Transient Local |
| `/point_parking/goal_valid` | `Bool` | `point_parking` | `parking` | Transient Local |
| `/point_parking/markers` | `MarkerArray` | `point_parking` | RViz | Transient Local |
| `/point_parking/nav_goal` | `PoseStamped` | `parking` | mode manager | Reliable/Volatile |
| `/mode_status` | `String` | mode manager | mission/모니터링 | 기본 QoS |
| `/parking_path/smac_reverse` | `Path` | mode manager | RViz | Transient Local |
| `/parking_path/final_reverse` | `Path` | mode manager | RViz | Transient Local |
| `/parking_path/exit_straight` | `Path` | mode manager | RViz | Transient Local |
| `/parking_path/exit_forward` | `Path` | mode manager | RViz | Transient Local |
| `/cmd_vel` | `Twist` | Nav2 controller server | 실차 구동기 | 기본 QoS, 10 Hz 설정 |

### 6.3 서비스와 액션

| 이름 | 타입 | 호출자 | 서버 |
|---|---|---|---|
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | `zone_scan` | global costmap |
| `/global_costmap/global_costmap/set_parameters` | `rcl_interfaces/srv/SetParameters` | `zone_scan` | global costmap node |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | mode manager | planner server |
| `follow_path` | `nav2_msgs/action/FollowPath` | mode manager | controller server |

namespace를 추가하면 이 네 이름과 YAML의 service/topic 이름을 함께 바꿔야 한다.

---

## 7. 노드별 동작과 이식 포인트

### 7.1 `zone_scan`

역할은 네 가지다.

1. 절대 UTM 주차구역을 map ROI로 변환한다.
2. GNSS gate 통과로 `/parkingScan`, `/parkingMode` 상태를 만든다.
3. scan 상태에서만 LiDAR를 map에 누적하고 point-to-line ICP로 작은 pose 오차를 보정한다.
4. global costmap을 초기화하고 구역별 inflation radius를 동적으로 변경한다.

#### 상태 전환

```text
시작: Scan=false, Mode=false

Zone1 Gate A 통과: Scan=true,  Mode=false
Zone1 Gate B 통과: Scan=false, Mode=true
Zone1 Gate A 재통과: Scan=false, Mode=false

Zone2 Gate A 통과: Scan=true,  Mode=false
Zone2 Gate B 통과: Scan=false, Mode=true
Zone2 Gate B 재통과: Scan=false, Mode=false
```

gate 판정은 이전 GNSS 위치와 현재 GNSS 위치가 만드는 선분이 gate 선분과 교차하거나 `gate_tolerance` 이내로 지나가는지 검사한다. `gate_rearm_distance`만큼 멀어져야 같은 gate가 다시 armed된다.

#### 누적과 ICP

- 기본 누적 범위: 두 parking ROI 내부 및 `roi_margin`
- voxel 해상도: `history_resolution`
- 기존 관측 병합: `spatial_merge_radius`
- point-to-line ICP는 EKF/TF를 수정하지 않고 현재 scan 좌표에만 보정 적용
- ICP 실패 또는 보정량 초과 시 원래 TF 결과 사용
- scan 시작 시 `clear_map_on_start: true`면 이전 voxel 삭제

실차에서 localization이 충분히 안정적이지 않으면 ICP를 유지하되, 움직이는 물체가 많은 환경에서는 Huber/대응 거리/RMSE 조건을 보수적으로 조정해야 한다.

#### global costmap 관리

- scan 시작: 이전 global costmap clear
- scan 종료 및 주차 중: 누적된 장애물 cost 유지
- parking mode 종료: global costmap clear
- clear 직후 ROI의 열린 입구를 제외한 세 면을 다시 publish
- Zone2 진입: `zone2_global_inflation_radius` 요청
- Zone2 출차: `zone1_global_inflation_radius`로 복원

현재 working tree의 실제 값은 두 반경이 모두 `1.5` m다.

```yaml
zone1_global_inflation_radius: 1.5
zone2_global_inflation_radius: 1.5
```

주석에는 Zone2가 `2.0` m라고 남아 있어 현재 값과 불일치한다. 실차 이식 전에 의도한 값을 하나로 확정하고 주석도 같이 고친다.

### 7.2 `point_parking`

`/zone_scan/occupied_points`를 주차구역의 열린 변에 투영하고 벽 구간 사이 gap을 구한다.

```text
map 누적점 → 절대 UTM 복원
→ 열린 변의 1차원 축으로 투영
→ 가까운 점들을 벽 구간으로 병합
→ 벽-벽, ROI끝-벽 gap 계산
→ min_gap_width 이상 후보 생성
→ 가장 넓은 후보 선택
```

Zone별 목표 규칙:

- Zone1: gap 중앙에서 UTM 남쪽으로 `zone1_first_parking_south_offset`
- Zone2: gap을 `zone2_goal_ratio_n : zone2_goal_ratio_m`으로 내분한 Easting 사용
- Zone2 Northing: ROI 네 꼭짓점 평균으로 고정
- Zone2 yaw: UTM 동쪽 `0 rad`

현재 설정은 다음과 같다.

```yaml
min_gap_width: 1.5
wall_merge_distance: 0.35
wall_padding: 0.1
wall_search_depth: 4.0
wall_behind_margin: 1.0
roi_endpoint_margin: 0.0
min_zone_wall_points: 3
zone1_first_parking_south_offset: 0.5
zone2_goal_ratio_n: 3.0
zone2_goal_ratio_m: 1.0
```

두 구역 후보를 한 목록에 넣고 폭이 가장 큰 후보를 선택한다. `clear_map_on_start: true`로 현재 구역의 점만 남기는 전제가 중요하다.

### 7.3 `parking`

`/parkingMode`가 `false → true`가 될 때 최신 유효 UTM goal을 map goal로 한 번만 변환해 `/point_parking/nav_goal`로 보낸다.

다음 네 조건이 모두 충족되어야 publish한다.

- parking mode가 true
- goal valid가 true
- 최신 goal pose가 존재
- UTM goal이면 datum이 존재

mode가 true인 동안 후보가 갱신되어도 기존 주차 action을 반복 취소하지 않는다. `/parkingMode=false`가 되어야 다음 goal 전송을 재무장한다.

### 7.4 `follow_path_client` / mode manager

상태는 다음과 같다.

```text
IDLE
  └─ CSV_FOLLOWING
       └─ PARKING
            ├─ Zone1: FINAL_REVERSE → EXIT_STRAIGHT → EXIT_FORWARD
            └─ Zone2: EXIT_FORWARD
                 └─ CSV_FOLLOWING 또는 IDLE
```

#### 1차 주차 경로

Smac의 DUBIN은 전진 경로만 만든다. mode manager는 다음 순서로 후진 경로를 얻는다.

```text
planner start = 주차 목표
planner goal  = 현재 차량 pose
전진 DUBIN 경로 계산
path.poses 순서 반전
현재 pose → 주차 목표의 전 구간 후진 경로로 사용
ParkingPath + first_parking_goal_checker 실행
```

#### Zone1 후속 시퀀스

1. 현재 map Y와 yaw를 고정한다.
2. `final_reverse_easting`을 map X로 변환한다.
3. `final_reverse_path_step` 간격의 직선 후진 Path를 직접 만든다.
4. `ParkingPath + parking_goal_checker`로 실행한다.
5. 1차 goal까지 `exit_straight_path_step` 간격으로 직선 전진한다.
6. 저장된 Gate A goal까지 Smac DUBIN 전진 경로를 만든다.
7. `FollowPath + goal_checker`로 출차한다.

직접 만든 최종 직선 후진 경로는 Smac을 다시 거치지 않는다. 따라서 이 구간의 장애물 안전은 local costmap과 MPPI `CostCritic`에 의존한다.

#### Zone2 후속 시퀀스

1차 주차가 성공하면 추가 후진 없이 저장된 Gate B 출차 goal까지 Smac DUBIN 경로를 만들고 `FollowPath`로 전진한다.

#### 새 goal과 실패 처리

- 새 주차 goal마다 sequence 번호 증가
- 이전 action 취소 후 새 goal 실행
- 늦게 도착한 이전 callback은 sequence 불일치로 폐기
- 실패 또는 완료 시 `_resume_csv()` 호출

실차 mission manager가 CSV를 쓰지 않으면 `_resume_csv()`를 mission-complete event publish 또는 IDLE 전환으로 바꿔야 한다.

---

## 8. 현장 전용 절대좌표 교체

현재 좌표는 특정 시험장의 값이므로 다른 장소에서 그대로 사용하면 안 된다.

### 8.1 `resource/parking_Zone`

파서는 각 구역에서 다음 한국어 키와 `E`, `N` 숫자를 찾는다.

```text
ParkingZone1
좌전방 (...), (E..., N...)
좌후방 (...), (E..., N...)
우전방 (...), (E..., N...)
우후방 (...), (E..., N...)
```

앞쪽 괄호의 CARLA 좌표는 파싱에 사용하지 않고, `E...`, `N...`만 사용한다. 실차 이식 시 혼동을 막기 위해 CARLA 좌표 표기를 제거하거나 현장 설명으로 바꾸는 것을 권장한다.

코드에 고정된 열린 변은 다음과 같다.

| 구역 | 열린 변 |
|---|---|
| Zone1 | `좌전방 → 우전방` |
| Zone2 | `좌전방 → 좌후방` |

새 구역의 입구 방향이 다르면 좌표 이름만 억지로 맞추지 말고 `point_parking.py`와 `zone_scan.py`의 open-edge 규칙을 함께 수정한다.

### 8.2 `config/parking_mode_gates.yaml`

각 gate는 두 절대 UTM 점으로 정의한다.

```yaml
zone1_gate_a_utm: [E1, N1, E2, N2]
zone1_gate_b_utm: [E1, N1, E2, N2]
zone2_gate_a_utm: [E1, N1, E2, N2]
zone2_gate_b_utm: [E1, N1, E2, N2]
```

gate를 너무 짧게 만들면 GNSS 오차로 통과를 놓치고, 너무 길게 만들면 인접 차선에서 잘못 전환될 수 있다. gate는 주행 방향을 가로지르도록 배치한다.

### 8.3 그 밖의 하드코딩 좌표

| 파라미터 | 현재 값 | 의미 |
|---|---:|---|
| `zone_scan.zone2_exit_gate_northing` | `4650268.0` | Zone2 Gate B 위 출차 목표 N |
| `follow_path_client.final_reverse_easting` | `417069.41` | Zone1 2차 직선 후진 종료 E |
| `zone1_first_parking_south_offset` | `0.5` | Zone1 gap 중앙에서 남쪽 목표 offset |
| `zone2_goal_ratio_n:m` | `3:1` | Zone2 gap 내부 목표 위치 |

특히 `final_reverse_easting`은 현재 차량 heading 기준 뒤쪽에 있어야 한다. 코드가 `delta_x * cos(yaw) < -0.05`를 검사하므로, 다른 방향으로 배치된 주차장에서는 Easting 고정 방식 자체를 거리 기반 또는 local heading 기반으로 일반화해야 한다.

---

## 9. Nav2 설정 이식

### 9.1 controller server의 필수 plugin 목록

```yaml
controller_server:
  ros__parameters:
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins:
      ["goal_checker", "first_parking_goal_checker", "parking_goal_checker"]
    controller_plugins: ["FollowPath", "ParkingPath"]
```

### 9.2 goal checker

```yaml
goal_checker:
  stateful: true
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.5
  yaw_goal_tolerance: 0.3

first_parking_goal_checker:
  stateful: true
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.35
  yaw_goal_tolerance: 0.35

parking_goal_checker:
  stateful: true
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.20
  yaw_goal_tolerance: 0.20
```

- `first_parking_goal_checker`: 1차 goal과 Zone1 출차 직선 구간을 비교적 일찍 완료
- `parking_goal_checker`: Zone1 최종 직선 후진의 엄격한 완료 판정
- 값이 크면 경로 추종 자체가 느슨해지는 것이 아니라 마지막 정렬 전에 action이 성공할 가능성이 커진다.

### 9.3 `ParkingPath`

현재 핵심 설정은 다음과 같다.

```yaml
ParkingPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Ackermann"
  vx_max: 0.0
  vx_min: -4.0
  wz_max: 1.5
  ackermann:
    plugin: "mppi::AckermannMotionModel"
    min_turning_r: 3.3
```

`vx_max: 0.0`으로 전진 trajectory를 금지하고 `vx_min`의 음수 크기로 최대 후진 명령을 정한다.

현재 working tree의 `-4.0 m/s`는 `14.4 km/h`다. 정확히 `14 km/h`를 원하면 다음 값이다.

```yaml
vx_min: -3.8889
```

14 km/h는 주차 속도로 매우 빠르다. 실차 최초 폐쇄구간 시험은 훨씬 낮은 속도에서 시작해야 하며, MPPI 상한만으로 실제 속도 overshoot를 막을 수 없으므로 DBW 계층에도 독립 상한이 필요하다.

주차 전용 critic 목록과 세부 weight는 `nav2_carla_params.yaml`의 `ParkingPath` 블록 전체를 복사한다. 특히 다음 설정을 잃으면 후진 경로의 방향 추종이 달라진다.

```yaml
PathAlignCritic:
  use_path_orientations: true

PathAngleCritic:
  forward_preference: false

GoalAngleCritic:
  symmetric_yaw_tolerance: false
```

### 9.4 `FollowPath`와 출차 속도

`EXIT_STRAIGHT`와 `EXIT_FORWARD`는 별도 출차 controller가 아니라 일반 `FollowPath`를 사용한다. 따라서 `FollowPath.vx_max`를 바꾸면 출차뿐 아니라 CSV 일반 주행 속도도 같이 바뀐다.

현재 working tree 값은 다음과 같다.

```yaml
FollowPath:
  vx_max: 4.0
  vx_min: -4.0
```

정확한 14 km/h 상한은 `3.8889 m/s`다.

출차에만 별도 속도를 적용하려면 다음 중 하나를 구현한다.

1. `ExitPath`라는 세 번째 MPPI plugin instance를 추가하고 출차 action의 `controller_id`를 바꾼다.
2. Nav2 `speed_limit` 토픽을 mode manager가 상태 전환 때 publish하고 복귀 시 원래 값으로 되돌린다.
3. 실차 DBW adapter가 `/mode_status`를 보고 mode별 속도 제한을 적용한다.

안전성과 책임 분리를 고려하면 실차에서는 1번 또는 3번이 명확하다.

### 9.5 Smac planner

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      motion_model_for_search: "DUBIN"
      minimum_turning_radius: 3.3
      angle_quantization_bins: 72
      tolerance: 0.25
      use_final_approach_orientation: true
      max_planning_time: 5.0
```

`minimum_turning_radius`는 MPPI와 실제 차량의 최소 회전반경과 일치해야 한다. wheelbase만으로 임의 산정하지 말고 최대 조향각을 포함해 측정한다.

### 9.6 global costmap

global costmap은 주차 경로를 계획할 때 다음 두 source를 사용한다.

```yaml
observation_sources: lidar_2d zone_scan_roi_boundary
```

- `lidar_2d`: `/zone_scan/cost_lidar_points`, marking + clearing
- `zone_scan_roi_boundary`: `/zone_scan/roi_boundary_points`, marking only
- ROI 내부를 채우지 않고 열린 입구를 제외한 세 면만 lethal obstacle로 만든다.

실차에 옮겨야 할 중요한 값:

```yaml
global_frame: map
robot_base_frame: base_link
rolling_window: true
width: 200
height: 200
resolution: 0.2
footprint: "[[2.1, 0.74], [2.1, -0.74], [-0.4, -0.74], [-0.4, 0.74]]"
```

footprint는 현재 Microlino/CARLA 값이므로 실차 외곽으로 반드시 교체한다.

### 9.7 local costmap의 중요한 현재 상태

현재 checkout의 local obstacle layer는 다음처럼 비활성화되어 있다.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        enabled: false
```

즉 현재 설정 그대로면 Smac의 계획 시점 global 장애물 회피는 가능하지만, 경로 실행 중 새로 나타난 장애물에 대한 MPPI local cost 회피는 꺼져 있다.

실차에서 동적 장애물 회피가 필요하면 다음을 수행한다.

- `enabled: true`
- LiDAR 토픽을 실차 토픽으로 변경
- sensor QoS 일치
- local costmap의 `global_frame`, footprint, 높이 범위, range 조정
- 정지 장애물/보행자 시험은 actuator 연결 전 shadow mode로 수행

단순히 `enabled: true`만 바꾸고 검증 없이 실차에 투입하면 안 된다.

---

## 10. 실차 구동기 어댑터

### 10.1 입력 계약

Nav2가 발행하는 명령은 `geometry_msgs/msg/Twist`다.

```text
linear.x  > 0: 전진 목표 속도 [m/s]
linear.x  = 0: 정지
linear.x  < 0: 후진 목표 속도 [m/s]
angular.z    : ROS 기준 yaw rate [rad/s], CCW+
```

### 10.2 CARLA 브리지에서 가져오지 말아야 할 부분

`cmd_vel_to_carla.py`는 다음 CARLA API에 직접 의존한다.

- `carla.Client`
- `carla.VehicleControl`
- CARLA physics wheel steering angle
- `reverse=True`

따라서 파일 자체를 실차에 복사하지 말고 `/cmd_vel` 이후를 실차 DBW 인터페이스로 새로 작성한다.

### 10.3 실차 adapter의 필수 안전 기능

- 명령 수신 timeout 시 throttle 0 + service brake
- 전진↔후진 전환 전에 차량 완전 정지 확인
- 별도 gear command가 필요한 경우 `D/R` state machine
- steering angle/rate 제한
- 종가속도/감속도/jerk 제한
- mode별 속도 제한과 절대 하드 리밋
- E-stop 및 manual override 우선권
- localization invalid, TF timeout, controller inactive 시 정지
- `/cmd_vel` sign과 실차 속도 feedback sign 일치
- 명령 source arbitration

MPPI의 `vx_min/vx_max`는 후보 trajectory와 출력 명령의 범위를 정하지만, 실제 차량의 독립 안전 제한을 대체하지 않는다.

### 10.4 Ackermann 변환

실차 구동기가 속도와 조향각을 받는다면 기본 관계는 다음과 같다.

```text
steering_angle = atan(angular_z * wheelbase / linear_x)
```

후진, 0속도 부근, steering sign, steering ratio는 차량 DBW 정의에 맞춰 별도 처리한다. 현재 CARLA 코드의 부호 반전은 CARLA 좌표계 전용이므로 그대로 가져오지 않는다.

---

## 11. launch 이식

### 11.1 권장 기동 순서

```text
1. 실차 센서 드라이버 및 hardware clock sync
2. localization / dual filter
3. map→odom→base_link 및 base_link→lidar TF
4. /f9p_utm 및 /utm_datum 공급
5. controller_server + planner_server + lifecycle manager + mode manager
6. auto_parking의 zone_scan + point_parking + parking
7. 실차 cmd_vel adapter는 safety ready 이후 활성화
```

`zone_scan`은 global costmap service가 준비되지 않으면 0.5초마다 재시도하므로 Nav2보다 조금 먼저 떠도 회복 가능하다. 하지만 초기 로그를 단순하게 유지하려면 Nav2 lifecycle active 이후 시작하는 편이 낫다.

### 11.2 실차 시간 인자

`auto_parking.launch.py`를 include할 때:

```python
launch_arguments={
    'use_sim_time': 'false',
    'start_rviz': 'false',
}.items()
```

`controller.launch.py`도 다음처럼 launch argument를 받아 모든 node에 전달하도록 바꾼다.

```python
use_sim_time = LaunchConfiguration('use_sim_time')
parameters=[params_file, {'use_sim_time': use_sim_time}]
```

현재 파일처럼 `True`를 하드코딩한 상태로 실차에 옮기면 `/clock`이 없을 때 timer, TF, action이 정상 진행하지 않는다.

### 11.3 namespace 사용 시

namespace를 적용하려면 상대 토픽과 절대 토픽을 구분해야 한다. 현재 많은 토픽이 `/`로 시작하는 절대 이름이므로 node namespace만 추가해도 이름이 바뀌지 않는다.

다차량 또는 통합 namespace가 필요하면 YAML과 코드의 topic 기본값을 상대 이름으로 바꾸거나 remapping을 명시한다. costmap service와 action 이름도 같이 바꾼다.

---

## 12. 이식 작업 순서

### 단계 A — 소스와 manifest

- [ ] `auto_parking` 패키지를 생성물 없이 복사
- [ ] `follow_path_client.py`를 대상 navigation/mission 패키지에 추가
- [ ] Python entry point 추가
- [ ] `package.xml` 의존성 병합
- [ ] config/resource/launch가 `setup.py data_files`에 포함됐는지 확인
- [ ] target Nav2 버전 고정

### 단계 B — 프레임과 데이터 계약

- [ ] `base_link` 원점을 실차 후륜축 또는 선택 기준점으로 확정
- [ ] `map → odom → base_link → lidar` 제공
- [ ] `/odometry/local`과 `/odometry/global` 의미 확정
- [ ] `/f9p_utm` 절대 UTM 공급
- [ ] `/utm_datum` latched 공급
- [ ] UTM zone과 hemisphere 고정
- [ ] map Y/yaw 부호 반전 유지 여부 결정
- [ ] LiDAR timestamp와 TF clock 동기화

### 단계 C — 현장 데이터

- [ ] `resource/parking_Zone` 네 꼭짓점 재측량
- [ ] Zone1/Zone2 열린 변 방향 확인
- [ ] Gate A/B 두 끝점 재측량
- [ ] `zone2_exit_gate_northing` 교체
- [ ] `final_reverse_easting` 교체 또는 거리 기반 로직으로 일반화
- [ ] Zone1 offset, Zone2 ratio 결정

### 단계 D — 차량/센서 파라미터

- [ ] local/global footprint 교체
- [ ] MPPI/Smac 최소 회전반경 교체
- [ ] wheelbase와 최대 조향각 확인
- [ ] LiDAR topic/frame/QoS/range/height 교체
- [ ] 속도, yaw rate, 가속도, DBW hard limit 설정
- [ ] `use_sim_time: false`

### 단계 E — mission 결합

- [ ] 기존 CSV 복귀를 유지할지 결정
- [ ] 유지하지 않으면 `_resume_csv()`를 mission event로 교체
- [ ] `/mode_status`를 상위 상태 머신과 연결
- [ ] 중복 goal source(`/goal_pose`, 자동 goal) 정책 결정
- [ ] action cancel과 새 goal retarget 정책 유지 여부 결정

### 단계 F — actuator 전 안전 검토

- [ ] local obstacle layer 활성화 여부를 의도적으로 결정
- [ ] 최종 직선 후진이 Smac 재계획을 거치지 않는다는 점 검토
- [ ] cmd timeout, E-stop, gear interlock 구현
- [ ] 14 km/h 설정을 그대로 쓸지 안전 책임자 승인
- [ ] controller failure 시 0속도 명령 및 brake fallback 확인

---

## 13. 파라미터 튜닝 지도

### 13.1 빈 공간 검출

| 파라미터 | 키우면 | 줄이면 |
|---|---|---|
| `min_gap_width` | 넓은 공간만 후보 | 좁은 틈도 후보 |
| `wall_merge_distance` | 떨어진 점도 한 벽으로 병합 | 벽이 잘게 분리될 수 있음 |
| `wall_padding` | 계산되는 gap 감소, 벽 여유 증가 | gap 증가, 안전 여유 감소 |
| `wall_search_depth` | ROI 안쪽 먼 점까지 사용 | 입구 근처 점만 사용 |
| `min_zone_wall_points` | 노이즈 오검출 감소 | 적은 관측으로도 후보 생성 |
| `zone1_first_parking_south_offset` | Zone1 1차 goal이 더 남쪽 | gap 중앙에 가까움 |
| `zone2_goal_ratio_n:m` | Zone2 gap 내부 목표 위치 이동 | 비율에 따라 반대 방향 이동 |

### 13.2 누적/ICP

| 파라미터 | 역할 |
|---|---|
| `history_resolution` | 누적 voxel 크기 |
| `roi_margin` | zone 경계 밖 허용 누적 거리 |
| `spatial_merge_radius` | 과거 동일 벽 재관측 병합 반경 |
| `icp_max_correspondence_distance` | scan-submap 대응 허용 거리 |
| `icp_max_translation` | ICP 위치 보정 상한 |
| `icp_max_rotation_deg` | ICP yaw 보정 상한 |
| `icp_max_point_to_line_rmse` | 정합 채택 RMSE 상한 |
| `tf_wait_timeout` | 정확한 timestamp TF 대기 시간 |

### 13.3 경로/완료

| 파라미터 | 역할 |
|---|---|
| `Smac.minimum_turning_radius` | 계획 가능한 최소 회전반경 |
| `Smac.angle_quantization_bins` | yaw 탐색 해상도 |
| `Smac.tolerance` | planner 위치 허용 오차 |
| `first_parking_goal_checker.yaw_goal_tolerance` | 1차 주차/출차 직선 완료 yaw 오차 |
| `parking_goal_checker.yaw_goal_tolerance` | Zone1 최종 후진 완료 yaw 오차 |
| `ParkingPath.GoalAngleCritic.cost_weight` | 목표 yaw 정렬 비용 강도 |
| `ParkingPath.PathAlignCritic.cost_weight` | 계획 경로 정렬 강도 |

### 13.4 장애물 여유

| 파라미터 | 역할 |
|---|---|
| `zone1_global_inflation_radius` | 일반/Zone1 계획 장애물 팽창 반경 |
| `zone2_global_inflation_radius` | Zone2 진입 중 계획 장애물 팽창 반경 |
| `global_costmap.inflation_layer.cost_scaling_factor` | 장애물 거리별 비용 감쇠 |
| `CostCritic.collision_cost` | MPPI 충돌 trajectory 패널티 |
| footprint | 차량 실제 외곽 충돌 검사 |

---

## 14. 알려진 제약과 이식 시 개선 권장사항

1. 구역은 `ParkingZone1`, `ParkingZone2` 두 개만 지원한다.
2. 구역별 열린 변과 출차 규칙이 코드에 하드코딩되어 있다.
3. Zone1 최종 후진은 Easting 고정 직선이어서 다른 방향의 주차장에 일반화되지 않는다.
4. Zone1 최종 후진 path는 Smac 재계획을 거치지 않는다.
5. Zone2는 최종 직선 후진을 무조건 생략한다.
6. 출차는 일반 `FollowPath`를 공유하므로 출차 전용 속도 파라미터가 없다.
7. 주차/출차 후 CSV 경로로 자동 복귀한다.
8. local obstacle layer가 현재 비활성화되어 있다.
9. `use_sim_time`이 launch 여러 곳에 기본값 또는 강제값으로 들어 있다.
10. `/cmd_vel` 이후 실차 watchdog, gear interlock, E-stop은 구현되어 있지 않다.
11. 현재 working tree의 Zone2 inflation 값과 주석이 불일치한다.
12. 현재 working tree 속도 `4.0 m/s`는 정확한 14 km/h가 아니라 14.4 km/h다.
13. `path_handler_plugins/FeasiblePathHandler` 설정은 현재 Nav2 소스에 대응 구현이 없다.
14. LiDAR subscriber가 Reliable 고정이라 Best Effort 실차 드라이버와 QoS가 맞지 않을 수 있다.
15. map Y/yaw 반전은 CARLA 파이프라인 전제이므로 표준 ENU 실차 map에서는 재검토가 필요하다.

실차 저장소에서는 좌표와 Zone 규칙을 Python 코드에서 분리해 하나의 site YAML로 옮기는 것을 권장한다. 특히 `open_edge`, `entry_gate`, `exit_gate`, `final_reverse_target`, `exit_sequence`를 구역별 데이터로 만들면 세 번째 구역을 코드 수정 없이 추가할 수 있다.

---

## 15. 장애 증상별 확인 지점

| 증상 | 우선 확인 |
|---|---|
| zone marker가 안 보임 | `/utm_datum` QoS/frame, Fixed Frame=`map`, `resource/parking_Zone` 설치 여부 |
| LiDAR 누적점이 없음 | `/parkingScan`, LiDAR QoS, `header.frame_id`, 정확한 timestamp TF, ROI 좌표 |
| 점이 이중 벽으로 누적됨 | time sync, localization jump, `spatial_merge_radius`, ICP 조건 |
| gate가 동작하지 않음 | `/f9p_utm`, UTM zone, gate 선분 위치, `gate_tolerance`, rearm 거리 |
| goal_valid가 false | 최소 벽점, gap 폭, open edge, wall search depth |
| goal이 반대편에 생성됨 | UTM→map Y 부호, yaw 부호, corner 순서 |
| planner가 빈 path 반환 | global costmap, footprint, inflation, turning radius, goal tolerance |
| 후진 대신 전진하려 함 | `ParkingPath.vx_max=0`, `vx_min<0`, controller_id, path 반전 |
| 목표 근처에서 너무 일찍 완료 | goal checker의 xy/yaw tolerance 감소 |
| 목표 근처에서 끝나지 않음 | tolerance가 localization 오차보다 작은지, GoalAngleCritic, odometry frame |
| 출차 속도만 바뀌지 않음 | 현재 출차가 일반 `FollowPath`를 공유한다는 점 확인 |
| costmap clear 실패 | service 이름/namespace, lifecycle active, `nav2_msgs` 버전 |
| inflation 변경 실패 | set_parameters service 이름과 dynamic parameter 지원 여부 |
| 실차가 gear를 반복 전환 | DBW adapter의 정지 확인/gear interlock 구현 |
| 명령이 끊긴 뒤 차가 계속 움직임 | actuator watchdog 및 brake fallback 구현 |

---

## 16. 단계별 검토 절차

이 절은 이식 후 수행할 검토 순서를 정의한다. 이 문서 작성 과정에서는 아래 검토를 실행하지 않았다.

### 16.1 정적 검토

- source package에 `auto_parking`, mode manager, config, resource가 모두 존재
- `setup.py`가 세 executable과 resource/config/launch를 설치
- manifest 의존성 누락 없음
- 절대 CARLA 토픽이 실차 토픽으로 교체됨
- `use_sim_time`이 실차 정책과 일치

### 16.2 actuator 없는 ROS graph 검토

- datum 수신 후 zone marker가 map에 고정
- gate 통과 재생 데이터로 Scan/Mode 상태가 예상 순서로 변함
- scan 중에만 cost LiDAR가 나옴
- point parking goal이 UTM과 map에서 같은 물리 위치
- global costmap에 ROI 세 면과 LiDAR 장애물이 표시
- Smac reverse path와 exit path가 발행
- `/cmd_vel`은 기록하되 차량에는 전달하지 않음

### 16.3 폐쇄구간 저속 검토

- DBW hard limit을 MPPI limit보다 낮게 설정
- 전진/후진 gear interlock 확인
- E-stop과 timeout 정지 확인
- 빈 구역에서 저속 주차/출차
- 정적 장애물 추가 후 planner failure/회피 확인
- localization 순간 오차에서 actuator가 안전하게 정지하는지 확인

### 16.4 운용 전 승인

- 현장 좌표 측량자와 좌표값 상호 검토
- 차량 footprint/turning radius 실측값 승인
- 속도와 제동거리 승인
- local obstacle layer 정책 승인
- fallback과 수동 전환 절차 승인

---

## 17. 최종 이식 체크리스트

```text
[ ] auto_parking 패키지 전체 복사
[ ] follow_path_client 및 entry point 복사/병합
[ ] Nav2 1.1.20 호환 기능 확보
[ ] use_sim_time=false 적용
[ ] map→odom→base_link→lidar TF 확보
[ ] /f9p_utm, /utm_datum, local/global odometry 확보
[ ] UTM/map Y·yaw 변환 정책 확정
[ ] parking_Zone 현장 좌표 교체
[ ] gate A/B 현장 좌표 교체
[ ] Zone2 exit Northing 교체
[ ] Zone1 final reverse 목표 일반화 또는 교체
[ ] 실차 footprint, wheelbase, turning radius 반영
[ ] LiDAR topic/frame/QoS/range 반영
[ ] goal checker와 MPPI critic 반영
[ ] global costmap 두 observation source 반영
[ ] local obstacle layer 활성화 정책 결정
[ ] 실차 cmd_vel adapter 구현
[ ] gear interlock/watchdog/E-stop 구현
[ ] 출차 전용 속도 분리 여부 결정
[ ] CSV 복귀를 실차 mission 흐름으로 교체/유지
[ ] actuator 없는 graph 검토
[ ] 폐쇄구간 저속 검토
[ ] 안전 승인 후 속도 상향
```

이 체크리스트가 모두 끝나기 전에는 현재 CARLA용 `cmd_vel_to_carla.py`를 대신해 만든 실차 adapter를 actuator에 연결하지 않는 것이 원칙이다.
