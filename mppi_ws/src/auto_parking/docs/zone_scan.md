# Zone Scan 노드 구조

## 좌표 변환

`resource/parking_Zone`의 `ParkingZone1`, `ParkingZone2` 섹션에 있는 각 네
모서리 UTM 좌표를 원본 데이터로 사용한다.
노드는 듀얼 필터의 `gnss_to_odom`과 같은 transient-local QoS로
`/utm_datum`을 구독하고, 동일한 식으로 좌표를 변환한다.

```text
map_x = corner_easting - datum_easting
map_y = -(corner_northing - datum_northing)
```

Marker의 `frame_id`는 `map`이며 `map -> odom -> base_link` TF 구조와
일치한다. datum을 받기 전에는 Marker를 발행하지 않는다.

## 2D LiDAR 누적 지도

`/carla/car/lidar_2d/point_cloud`의 `PointCloud2`를 Reliable QoS로 구독한다.
각 스캔은 메시지의 측정 시각에 해당하는 TF를 사용해 센서 프레임에서 `map`
프레임으로 변환한다. 따라서 차량이 이동한 뒤에도 과거 점은 측정 당시의 전역
위치에 남는다.

CARLA LiDAR 메시지가 TF보다 약 0.05초 먼저 도착하므로 메시지를 큐에 보관하고
해당 측정 timestamp의 정확한 `map -> lidar_2d` TF가 들어온 뒤 변환한다. 과거
또는 최신 TF로 대체하지 않는다. `tf_wait_timeout` 안에 정확한 TF가 들어오지 않은
스캔은 잘못된 전역 위치에 기록하는 대신 폐기한다.

CARLA `stack.json`의 2D LiDAR는 `points_per_second: 20000`,
`rotation_frequency: 20`으로 설정한다. 회전당 약 1000점을 생성하므로 0.15m
voxel 지도에서 벽 표면이 불연속 점이 아니라 연속된 바운더리로 형성된다.

고정 주차 경계는 구역별로 다음의 열린 순서로 생성하며 마지막 점에서 시작점으로
돌아가는 면은 닫지 않는다.

```text
ParkingZone1: 좌전방 -> 좌후방 -> 우후방 -> 우전방
ParkingZone2: 좌전방 -> 우전방 -> 우후방 -> 좌후방
```

기본 `accumulation_scope: parking_zones`에서는 두 ParkingZone의 실제 4점
폴리곤 내부와 폴리곤 경계에서 `roi_margin` 이내인 LiDAR 점만 기록한다. 기본
여유 거리는 1m다. TF나 측위값이 급격히 튀어 주차존에서 멀리 변환된 점은 voxel
지도에 추가하기 전에 폐기한다.

허용된 PointCloud는 측정 시점의 `map -> lidar_2d` TF로 변환해 전역 `map`
좌표의 점유 voxel 집합에 즉시 추가한다. 차량이 이동해도 기존 voxel 좌표는 다시
차량 프레임으로 변환하지 않으므로 관측한 전역 위치에 고정된다. 스캔 순서나 같은
스캔 안의 점 간격에는 의존하지 않는다.

이전 스캔에서 기록한 전역 voxel의 `spatial_merge_radius` 안에 같은 물체가 다시
관측되면 새 voxel로 추가하지 않는다. 최초 기록 위치를 유지하여 측위 노이즈로
동일한 벽의 평행 복제선이 반복 생성되는 현상을 억제한다.

## Scan-to-submap point-to-line ICP

`icp_enabled: true`이면 측정 시각의 TF로 변환한 현재 스캔을 바로 누적하지 않고,
기존 LiDAR voxel submap에 2D point-to-line ICP로 먼저 정합한다. EKF/TF pose는
초기값이자 전역 UTM 기준으로 계속 사용하며, ICP가 계산한 작은 SE(2) 보정은 현재
스캔을 지도에 넣을 때만 적용한다. EKF나 `map -> odom` TF는 변경하지 않는다.

적용 스위치는 다음과 같다. YAML 파라미터는 노드 시작 시 읽으므로 값을 변경한
뒤에는 노드를 다시 실행한다. 시작 로그의 `point_to_line_icp=enabled/disabled`로
실제 적용 상태를 확인할 수 있다.

```yaml
# 정합 사용: TF 초기 위치를 submap에 맞춰 소폭 보정한 뒤 누적
icp_enabled: true

# 정합 미사용: 기존처럼 측정 시각 TF 결과를 바로 ROI/voxel 처리
icp_enabled: false
```

### 1. Submap과 normal 생성

Submap은 과거 스캔에서 확정된 `_occupied_voxels`의 중심점으로 만든다. 각 voxel
주변 `icp_normal_radius` 안의 이웃점을 모아 2x2 공분산 행렬을 계산한다.

```text
C = (1/N) * sum((q_j - mean(q)) (q_j - mean(q))^T)
```

공분산의 큰 고유벡터는 벽을 따라가는 방향이고, 작은 고유벡터는 벽에 수직인
normal `n`이다. 두 고유값의 비율이 `icp_max_normal_ratio`보다 큰 원형·모서리형
이웃은 안정적인 직선으로 보지 않고 정합 대상에서 제외한다.

### 2. 최근접 대응점 생성

측정 시각 TF로 얻은 현재 점 `p_i`마다 가장 가까운 submap 점 `q_i`를 찾는다.
두 점의 유클리드 거리가 `icp_max_correspondence_distance`보다 가까운 쌍만
대응점으로 사용한다. 대응점 수가 `icp_min_correspondences`보다 적으면 현재
스캔에는 ICP를 적용하지 않는다.

### 3. Point-to-line 오차 계산

Point-to-point ICP가 `p_i`와 `q_i` 사이의 모든 방향 거리를 줄이는 것과 달리,
point-to-line ICP는 submap 벽의 normal 방향 오차만 줄인다.

```text
r_i = n_i^T * (p_i - q_i)
```

따라서 벽을 따라 점 배열이 조금 달라도 같은 벽 표면에 맞출 수 있다. 현재
스캔에 적용할 보정 변수는 2D 강체변환 `delta = [dx, dy, dtheta]`다. 스캔 중심을
회전축으로 사용한 선형화 Jacobian은 다음 형태다.

```text
J_i = [n_x, n_y, -n_x * p_y + n_y * p_x]
```

여기서 `p_x`, `p_y`는 스캔 중심에 대한 상대 좌표다.

### 4. 강건 최소제곱과 EKF prior

큰 residual은 움직이는 물체나 잘못된 최근접 대응일 가능성이 있으므로
`icp_huber_delta`를 넘으면 Huber weight를 낮춘다. 각 반복에서 다음 선형식을
풀어 `delta`를 구한다.

```text
(J^T W J + lambda I) delta = -J^T W r
```

`W`는 Huber weight, `lambda`는 `icp_prior_weight`다. prior는 평행한 벽 하나만
보여 벽 방향 이동을 관측할 수 없을 때 보정 pose가 EKF 초기값에서 임의로
미끄러지는 것을 억제한다. 계산된 보정을 스캔에 적용하고 최근접점을 다시 찾는
과정을 `icp_max_iterations`까지 반복한다.

### 5. 채택과 누적

최종 보정이 위치 `icp_max_translation`, 각도 `icp_max_rotation_deg` 안에 있고,
RMSE 및 개선량 조건을 통과할 때만 보정된 점을 사용한다. 실패하면 예외적인
좌표를 억지로 정합하지 않고 원래 TF 점으로 되돌아간다. 마지막으로 정확한
ParkingZone ROI를 다시 검사하고 0.15m voxel에 합친다.

누적 voxel 주변을 `icp_normal_radius` 반경으로 모아 PCA를 수행하고, 선형성이 충분한
이웃의 최소 고유벡터를 벽의 normal로 사용한다. 현재 스캔의 최근접 submap 점과
normal 사이의 수직 거리(point-to-line residual)를 Huber 가중 최소제곱으로 줄인다.
평행 벽처럼 관측되지 않는 방향은 `icp_prior_weight`가 EKF 초기값에서 과도하게
움직이지 않도록 제한한다.

다음 경우에는 ICP 보정을 폐기하고 원래 측정 시각 TF 결과를 사용한다.

- submap 또는 대응점 수가 최소값보다 적음
- 대응점 거리가 `icp_max_correspondence_distance`를 넘음
- 전체 보정이 `icp_max_translation` 또는 `icp_max_rotation_deg`를 넘음
- 최종 point-to-line RMSE가 `icp_max_point_to_line_rmse`보다 큼
- RMSE 개선량이 `icp_min_improvement`보다 작음

초기 submap은 ICP 없이 TF 결과로 생성한다. Zone1 submap과 멀리 떨어진 Zone2의
첫 스캔도 대응점이 없으므로 원래 TF 결과로 새 영역을 시작하고, 그 다음 스캔부터
Zone2에 새로 생긴 voxel과 정합한다. ICP 전에는 ROI에 최대 보정 거리만큼 여유를
주지만 최종 누적 직전에는 다시 정확한 `roi_margin`을 검사하므로, 주차존에서 멀리
튄 점은 기록되지 않는다.

이 방식은 온라인으로 새 스캔만 보정한다. 이미 voxel로 합쳐진 과거 스캔을 나중에
다시 움직이는 pose-graph 최적화는 수행하지 않는다.

전체 주행 공간을 의도적으로 누적해야 하는 경우에만 `accumulation_scope: global`
을 사용한다. 이 모드에서는 주차존 ROI 필터가 적용되지 않는다.

`/parkingScan`이 `false -> true`로 다시 시작되면 기본 설정에서
`/global_costmap/clear_entirely_global_costmap`을 호출해 이전 global cost를
초기화한다. 서비스 완료 직후 열린 입구를 제외한 ParkingZone 3면 테두리 cloud를
다시 발행하므로 ROI 경계 cost는 유지되고, 나머지 장애물 cost는 새 LiDAR 스캔으로
다시 생성된다. `/parkingScan`이 `true -> false`로 끝나면 새 LiDAR 입력만 중단하고
Scan 중 생성된 cost는 주차가 진행되는 `/parkingMode=true` 동안 유지한다.
`/parkingMode`가 `true -> false`로 바뀌는 재정비 전환 때 서비스를 다시 호출해
LiDAR cost를 제거하고 ROI 경계 cost만 복원한다.

시각화할 때는 점유 voxel의 상하좌우 이웃을 검사하여, 이웃이 없는 바깥쪽 모서리만
`LINE_LIST`로 생성한다. 따라서 RViz에는 원본 점이 아니라 지금까지 누적된 점유
영역의 2D 바운더리가 표시된다. 생성된 voxel은 `map` 좌표로 저장되며 노드가
종료될 때까지 유지된다.

RViz에는 3D 벽을 만들지 않고 다음 두 종류의 2D 선만 표시한다.

- 녹색 실선: ParkingZone1 열린 경계
- 하늘색 실선: ParkingZone2 열린 경계
- 주황색 실선: 현재까지 누적된 LiDAR 스캔 지도

별도의 최신 스캔 Marker는 사용하지 않는다. 새 스캔의 점유 voxel을 즉시 누적
지도에 합치고 전체 바운더리를 다시 발행하므로, 차량이 지나간 뒤에도 이미 관측된
영역은 지도에 남는다. 바운더리가 없는 연속 영역을 주차공간 입구 후보로 관찰할
수 있다.

기존 RViz의 `Lidar2D/PointCloud` display는 센서 원본을 보여 주므로 항상 점으로
표시된다. 생성된 벽만 보려면 해당 display를 끄고 `/parking_zones`를 구독하는
`MarkerArray` display를 켠다.

## 실행

기존 RViz를 유지한 채 Marker 발행 노드만 실행한다.

```bash
ros2 launch auto_parking zone_scan.launch.py
```

별도 RViz도 함께 열 때만 다음 인자를 사용한다.

```bash
ros2 launch auto_parking zone_scan.launch.py start_rviz:=true
```

현재 RViz의 Fixed Frame을 `map`으로 설정하고 `MarkerArray` display의
토픽을 `/parking_zones`로 지정한다. QoS는 Reliable, Transient Local,
Keep Last, Depth 1로 설정한다.

Orbit View의 `Target Frame`도 `map`으로 설정한다. 이 값이 `f9r` 또는
`base_link`이면 카메라가 차량을 따라가므로, 실제로 `map`에 고정된 누적 지도도
화면상 로컬 지도처럼 움직여 보인다.

## GPS 게이트 기반 Parking Scan / Mode

`config/parking_mode_gates.yaml`에 ParkingZone1과 ParkingZone2의 A/B 게이트를
각각 두 UTM 점으로 설정한다.

```yaml
gate_control_enabled: true
zone1_gate_a_utm: [ZONE1_A_GPS1_E, ZONE1_A_GPS1_N,
                   ZONE1_A_GPS2_E, ZONE1_A_GPS2_N]
zone1_gate_b_utm: [ZONE1_B_GPS1_E, ZONE1_B_GPS1_N,
                   ZONE1_B_GPS2_E, ZONE1_B_GPS2_N]
zone2_gate_a_utm: [ZONE2_A_GPS1_E, ZONE2_A_GPS1_N,
                   ZONE2_A_GPS2_E, ZONE2_A_GPS2_N]
zone2_gate_b_utm: [ZONE2_B_GPS1_E, ZONE2_B_GPS1_N,
                   ZONE2_B_GPS2_E, ZONE2_B_GPS2_N]
```

노드는 `/f9p_utm`의 이전 위치와 현재 위치를 하나의 이동 선분으로 만든다. 이
이동 선분이 게이트 선분과 교차하거나 `gate_tolerance` 이내로 지나가면 상태를
전환한다. 진행 방향과 관계없이 통과를 판정한다.

```text
노드 시작:                /parkingScan=false, /parkingMode=false

ParkingZone1 A 첫 통과:   /parkingScan=true,  /parkingMode=false
ParkingZone1 B 통과:      /parkingScan=false, /parkingMode=true
ParkingZone1 A 재통과:    /parkingScan=false, /parkingMode=false

ParkingZone2 A 통과:      /parkingScan=true,  /parkingMode=false
ParkingZone2 B 첫 통과:   /parkingScan=false, /parkingMode=true
ParkingZone2 B 재통과:    /parkingScan=false, /parkingMode=false
```

ParkingZone1은 `A -> B -> A`, ParkingZone2는 `A -> B -> B` 상태 흐름을 사용한다.
LiDAR PointCloud를 map에 누적할지는 `/parkingScan` 상태로 결정하며,
`/parkingMode`는 B 게이트 첫 통과 후 켜진다. Zone1은 A 재통과, Zone2는 B
재통과 시 꺼진다.

출차에 사용할 gate 목표도 같은 상태 전환에서 저장한다.

- ParkingZone1: Gate A 두 끝점의 중앙
- ParkingZone2: Gate B 선분 위 `N=4650268.0`인 지점

목표는 `/parking_exit/goal_utm`에 UTM Pose로, 구역 이름은
`/parking_exit/zone`에 발행한다. Zone1 yaw는 Gate A 진입 방향의 반대 방향이고,
Zone2 yaw는 Gate B 진입 당시 진행 방향을 그대로 사용한다. Zone1은 연석 근처에서
바로 조향하지 않도록 1차 주차 goal까지 직선 전진한 뒤 Gate A 중앙을 향한 DUBIN
경로로 전환한다.

global costmap용 LiDAR도 `/parkingScan=true`인 동안에만
`/zone_scan/cost_lidar_points`로 중계된다. Scan 종료 시 중계만 즉시 멈추며,
누적된 global LiDAR cost는 주차 중 유지된다. `/parkingMode=false` 재정비 전환에서
global costmap을 초기화한 뒤 ParkingZone ROI 3면 테두리 cost만 복원한다.

각 게이트는 통과 직후 잠금 상태가 되며, 차량이 `gate_rearm_distance` 이상
벗어난 뒤에만 다음 통과를 받을 수 있다. 따라서 GPS가 게이트 주변에서 흔들려도
ParkingZone1 상태가 연속으로 여러 번 토글되지 않는다.

`clear_map_on_start: false`이면 기록을 정지해도 기존 지도는 고정되고, 다음 구역에서
기록을 시작할 때도 이전 구역의 스캔이 유지된다. 두 주차구역 결과를 하나의 map에
남기려면 이 값을 `false`로 사용한다.

`/parkingScan`과 `/parkingMode`는 모두 `std_msgs/Bool`이며 Reliable +
Transient Local QoS로 발행한다. 게이트 제어가 비활성화되면 두 상태 모두
`false`로 유지하여 의도하지 않은 전체 구간 스캔 누적을 막는다.

RViz에는 각 구역의 A 게이트를 노란색, B 게이트를 보라색 실선으로 표시한다.
게이트 Marker 표시는 `gate_control_enabled`와 분리되어 있어 제어가 꺼져 있어도
좌표가 유효하면 계속 표시된다. 두 끝점이 같은 미설정 게이트만 표시하지 않는다.

## 파라미터

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `zone_file` | 설치된 `resource/parking_Zone` | 두 구역의 UTM 모서리 원본 파일 |
| `frame_id` | `map` | Marker 좌표 프레임 |
| `datum_topic` | `/utm_datum` | UTM datum 입력 |
| `marker_topic` | `/parking_zones` | MarkerArray 출력 |
| `line_width` | `0.25` | 외곽선 두께(m) |
| `zone_height` | `0.05` | 지면 위 표시 높이(m) |
| `lidar_topic` | `/carla/car/lidar_2d/point_cloud` | 2D LiDAR PointCloud2 입력 |
| `history_resolution` | `0.15` | 누적 벽 voxel 해상도(m) |
| `roi_margin` | `1.0` | 주차존 폴리곤 경계 바깥 기록 허용 거리(m) |
| `lidar_line_width` | `0.12` | 누적 LiDAR 지도 실선 두께(m) |
| `tf_wait_timeout` | `0.5` | 정확한 측정 시각 TF를 기다릴 최대 시간(초) |
| `spatial_merge_radius` | `0.35` | 기존 전역 기록과 동일 관측으로 볼 반경(m) |
| `accumulation_scope` | `parking_zones` | `parking_zones` ROI 누적 또는 `global` 전체 누적 |
| `icp_enabled` | `true` | Scan-to-submap point-to-line ICP 활성화 |
| `icp_max_iterations` | `8` | 스캔당 최대 ICP 반복 횟수 |
| `icp_min_submap_points` | `30` | normal submap 정합에 필요한 최소 점 수 |
| `icp_min_correspondences` | `20` | 정합을 받아들이기 위한 최소 대응점 수 |
| `icp_max_correspondence_distance` | `0.5` | 최근접 대응 허용 거리(m) |
| `icp_normal_radius` | `0.6` | submap 벽 normal 계산 반경(m) |
| `icp_max_normal_ratio` | `0.3` | PCA 선형 구조 판정 최대 고유값 비율 |
| `icp_huber_delta` | `0.1` | Huber 강건 손실 전환 거리(m) |
| `icp_max_translation` | `0.5` | EKF 초기 pose에서 허용할 최대 위치 보정(m) |
| `icp_max_rotation_deg` | `3.0` | 허용할 최대 각도 보정(deg) |
| `icp_max_scan_points` | `400` | 정합에 사용할 현재 스캔 최대 점 수 |
| `icp_max_submap_points` | `2500` | 정합 대상 submap 최대 점 수 |
| `icp_prior_weight` | `1.0` | EKF 초기값 유지 정규화 가중치 |
| `icp_min_improvement` | `0.002` | ICP 채택에 필요한 최소 RMSE 개선량(m) |
| `icp_max_point_to_line_rmse` | `0.15` | ICP 채택 최대 수직 RMSE(m) |
| `gate_control_enabled` | `false` | GPS 게이트 기반 기록 제어 활성화 |
| `zone1_gate_a_utm` | `[0, 0, 0, 0]` | Zone1 A 게이트 `[E1, N1, E2, N2]` |
| `zone1_gate_b_utm` | `[0, 0, 0, 0]` | Zone1 B 게이트 `[E1, N1, E2, N2]` |
| `zone2_gate_a_utm` | `[0, 0, 0, 0]` | Zone2 A 게이트 `[E1, N1, E2, N2]` |
| `zone2_gate_b_utm` | `[0, 0, 0, 0]` | Zone2 B 게이트 `[E1, N1, E2, N2]` |
| `gate_tolerance` | `0.5` | 이동 선분과 게이트 통과 허용 거리(m) |
| `gate_rearm_distance` | `1.0` | 다음 통과 감지 전 게이트에서 벗어날 거리(m) |
| `clear_map_on_start` | `false` | 기록 시작 시 이전 누적 지도 삭제 여부 |
| `utm_position_topic` | `/f9p_utm` | 차량의 절대 UTM 위치 입력 |
| `parking_scan_topic` | `/parkingScan` | LiDAR map 기록 활성 상태 Bool 출력 |
| `parking_mode_topic` | `/parkingMode` | 주차구역 진입 상태 Bool 출력 |

증분 또는 symlink 설치 공간에 새 리소스가 없으면 실행 파일의 실제 symlink
위치를 기준으로 소스 패키지의 `resource/parking_Zone`을 자동으로 사용한다.
