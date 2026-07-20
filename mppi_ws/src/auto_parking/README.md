# auto_parking

하나의 ROS 2 Python 패키지 안에 자동주차 파이프라인용 노드 세 개를 둔다.

| ROS 노드 | 실행 파일 | 현재 역할 |
| --- | --- | --- |
| `/zone_scan` | `zone_scan` | GPS 게이트, LiDAR ROI 누적, point-to-line ICP, 열린 입구를 제외한 ROI 테두리 cost cloud 및 RViz Marker 발행 |
| `/point_parking` | `point_parking` | 누적 벽 사이 1.5m 이상 gap 탐색, UTM goal/yaw 및 RViz Marker 발행 |
| `/parking` | `parking` | parkingMode 상승 시 Point Parking UTM goal을 map으로 바꿔 Nav2/MPPI 흐름에 1회 전달 |

## 소스 구조

```text
auto_parking/
├── auto_parking/
│   ├── zone_scan.py
│   ├── point_parking.py
│   ├── parking.py
│   └── parkingZone_visualizer.py  # 이전 import 호환 wrapper
├── config/
│   ├── zone_scan.yaml
│   ├── parking_mode_gates.yaml
│   ├── point_parking.yaml
│   └── parking.yaml
├── docs/
│   ├── zone_scan.md
│   ├── point_parking.md
│   └── parking.md
├── launch/
│   ├── zone_scan.launch.py
│   ├── point_parking.launch.py
│   ├── parking.launch.py
│   ├── auto_parking.launch.py
│   └── parking_zone_visualizer.launch.py  # 이전 명령 호환
├── resource/
│   ├── auto_parking
│   └── parking_Zone
└── rviz/
    └── zone_scan.rviz
```

## 실행

개별 실행:

```bash
ros2 launch auto_parking zone_scan.launch.py
ros2 launch auto_parking point_parking.launch.py
ros2 launch auto_parking parking.launch.py
```

세 노드 함께 실행:

```bash
ros2 launch auto_parking auto_parking.launch.py
```

Zone Scan RViz까지 함께 실행:

```bash
ros2 launch auto_parking auto_parking.launch.py start_rviz:=true
```

이전 실행 명령 `parking_zone_visualizer.launch.py`와 실행 파일
`parking_zone_visualizer`는 `/zone_scan`을 실행하는 호환 별칭으로 유지한다.

## Point Parking 출력

| 토픽 | 타입 | 내용 |
| --- | --- | --- |
| `/point_parking/goal_utm_yaw` | `std_msgs/Float64MultiArray` | 가장 넓은 gap의 `[UTM_E, UTM_N, yaw_rad]` |
| `/point_parking/goal_pose` | `geometry_msgs/PoseStamped` | 대표 goal의 UTM pose |
| `/point_parking/goal_candidates` | `geometry_msgs/PoseArray` | 1.5m 이상인 모든 후보 |
| `/point_parking/goal_valid` | `std_msgs/Bool` | 현재 유효한 대표 goal 존재 여부 |
| `/point_parking/markers` | `visualization_msgs/MarkerArray` | gap, 중심점, yaw 화살표, 폭 label |

## Parking 자동 MPPI 연결

`/parking` 노드는 `/parkingMode`가 `false -> true`로 전환될 때 최신 유효
`/point_parking/goal_pose`를 `/point_parking/nav_goal`로 한 번만 전달한다.
Point Parking의 절대 UTM pose를 `/utm_datum`으로 `map` pose로 변환해
발행한다.

```text
/parkingMode=true
  -> /parking: UTM goal을 map goal로 변환
  -> /point_parking/nav_goal
  -> dual_filter mode_manager: ComputePathToPose
  -> FollowPath(controller_id=ParkingPath)
  -> 현재 N 고정, E=417069.41 직선 후진
  -> Nav2 MPPI
```

`/parkingMode=false`가 되면 다음 주차구간을 위해 다시 무장한다. mode가 계속
`true`인 동안 Point Parking 후보가 갱신되어도 진행 중 경로를 반복해서 취소하거나
재계획하지 않는다.
