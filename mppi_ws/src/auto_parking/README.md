# auto_parking

하나의 ROS 2 Python 패키지 안에 자동주차 파이프라인용 노드 세 개를 둔다.

| ROS 노드 | 실행 파일 | 현재 역할 |
| --- | --- | --- |
| `/zone_scan` | `zone_scan` | GPS 게이트, ParkingScan 상태, LiDAR ROI 누적, point-to-line ICP, RViz Marker |
| `/point_parking` | `point_parking` | 점군 기반 주차공간 판단을 위한 빈 골격 |
| `/parking` | `parking` | 최종 자동주차 제어를 위한 빈 골격 |

## 소스 구조

```text
auto_parking/
├── auto_parking/
│   ├── zone_scan.py
│   ├── point_parking.py
│   ├── parking.py
│   └── parkingZone_visualizer.py  # 이전 import 호환 wrapper
├── config/
│   └── zone_scan.yaml
├── docs/
│   └── zone_scan.md
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
