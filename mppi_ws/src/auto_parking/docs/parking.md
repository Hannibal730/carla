# Parking goal bridge

`/parking`은 Zone Scan의 주차모드와 Point Parking의 대표 goal을 기존 Nav2 주차
흐름에 연결한다. MPPI 컨트롤러는 pose 하나를 직접 입력받지 않고 path를 입력받기
때문에 다음 순서로 전달한다.

```text
/parkingMode (Bool) false -> true
          + 최신 /point_parking/goal_valid=true
          + 최신 /point_parking/goal_pose (UTM PoseStamped)
          + /utm_datum
                         |
                         v
              /parking UTM -> map 변환
                         |
                         v
          /point_parking/nav_goal (map)
                         |
                         v
       dual_filter mode_manager / ComputePathToPose
                         |
                         v
       FollowPath(controller_id=ParkingPath) / MPPI
                         |
                  1차 주차 성공
                    /          \
          ParkingZone1       ParkingZone2
                 |                |
                 v                v
  N 고정, E=417069.41 후진   2차 후진 생략
                 |                |
                 +------ 전진 출차 ------+
```

## 좌표 변환

Point Parking의 pose는 절대 UTM 좌표이고 Nav2의 global frame은 `map`이다.

```text
map_x   = goal_utm_easting - datum_easting
map_y   = -(goal_utm_northing - datum_northing)
map_yaw = -utm_yaw
```

CARLA/ROS map 축과 UTM northing 축의 방향 차이 때문에 Y와 yaw 부호를 반전한다.

## 전송 조건

- `/parkingMode=true`여야 한다.
- `/point_parking/goal_valid=true`여야 한다.
- goal pose와 datum이 모두 도착해야 한다.
- 한 번 전송한 뒤 같은 `true` 구간에서는 다시 전송하지 않는다.
- `/parkingMode=false` 수신 시 다음 전송을 재무장한다.
- 유효 goal이 사라지면 이전 latched pose를 폐기해 재사용하지 않는다.

## 설정

`config/parking.yaml`에서 입출력 토픽과 frame 이름을 변경할 수 있다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `parking_mode_topic` | `/parkingMode` | 자동 goal 전송 트리거 |
| `point_goal_topic` | `/point_parking/goal_pose` | 절대 UTM 대표 goal |
| `point_goal_valid_topic` | `/point_parking/goal_valid` | 대표 goal 유효 여부 |
| `datum_topic` | `/utm_datum` | UTM 기준점 |
| `output_goal_topic` | `/point_parking/nav_goal` | mode_manager의 Point Parking 전용 입력 |
| `utm_frame_id` | `utm` | 입력 goal frame |
| `map_frame_id` | `map` | Nav2 goal frame |

ParkingZone1의 Point Parking goal이 성공하면 mode manager는
`/odometry/global`의 현재 위치에서 N을 고정하고, UTM
`E=417069.41`까지 0.2 m 간격의 직선 path를 만들어 `ParkingPath`로
2차 후진한다. ParkingZone2는 이 단계를 생략하고 Gate B 출차 경로로 바로
전환한다. RViz의 일반 `/goal_pose`는 자동 출차 시퀀스를 실행하지 않는다.
