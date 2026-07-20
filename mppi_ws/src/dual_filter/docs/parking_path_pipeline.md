# 주차 경로 생성·비용지도·추종 파이프라인

## 1. 전체 흐름

```text
ParkingZone ROI 안에서 LiDAR 누적
  └─ /zone_scan/occupied_points (map 좌표의 주황점)
          ↓
ParkingZone 열린 변에 점 투영
          ↓
벽 구간 병합 → gap 계산
          ↓
가장 넓은 gap 선택
          ↓
Zone별 목표 위치·yaw 생성 ───────────── goal ────────────┐

ParkingZone 기하학 테두리
  └─ 열린 입구를 제외한 3면을 /zone_scan/roi_boundary_points로 발행
          ↓
global_costmap obstacle layer
  ├─ Scan 중에만 중계되는 /zone_scan/cost_lidar_points
  └─ ROI 3면 테두리
          ↓
lethal cost + inflation cost ───────────→ SmacPlannerHybrid ←┘
                                                   ↓
주차 목표 → 현재 자세의 DUBIN 전진 경로
                                                   ↓
pose 순서를 반전해 현재 → 목표 후진 경로 생성
                                                   ↓
ParkingPath MPPI로 전 구간 후진 추종
                                                   ↓
Zone별 주차 완료·출차 분기
  ├─ ParkingZone1
  │    E=417069.41까지 2차 직선 후진
  │      → 1차 주차 goal까지 직선 전진
  │      → Gate A 중앙까지 DUBIN 전진
  └─ ParkingZone2
       2차 후진 생략
         → Gate B의 N=4650268.0 지점까지 진입 yaw 그대로 DUBIN 전진
                                                   ↓
CSV 경로 추종 복귀
```

별도의 경로 fusion optimizer와 등간격 후처리는 사용하지 않는다. 장애물 회피와
경로 비용은 Smac이 global costmap을 통해 한 번에 처리한다.

## 2. 주차 목표 생성

### ParkingZone1

- 열린 변에서 가장 넓은 gap을 선택한다.
- gap 중앙점에서 UTM 남쪽으로 0.5 m 이동한 위치를 1차 목표로 사용한다.
- yaw는 ParkingZone 내부에서 열린 입구를 바라보는 방향이다.

### ParkingZone2

평행주차 규칙을 사용한다.

```text
ratio    = zone2_goal_ratio_n / (zone2_goal_ratio_n + zone2_goal_ratio_m)
goal_E   = gap_start_E + (gap_end_E - gap_start_E) × ratio
goal_N   = ParkingZone2 네 꼭짓점의 평균 Northing
goal_yaw = 0 rad (UTM 동쪽)
```

현재 zone 좌표에서 고정되는 Northing은 `4650276.25`이다. gap 탐색 결과에 따라
Easting만 달라진다. `1:1`은 기존 중앙값이며 `2:1`은 gap 시작에서 끝 방향으로
`2/3` 지점이다. map 변환 후에도 목표 yaw는 `0 rad`가 된다.

코스 규칙상 가장 넓은 gap이 실제 목표이므로 다른 후보의 플래너 성공 여부는
비교하지 않는다.

## 3. ROI 테두리의 global cost

`global_costmap.obstacle_layer`는 두 입력을 동시에 사용한다.

| 입력 | 역할 | clearing |
|---|---|---|
| `/zone_scan/cost_lidar_points` | Scan 중에만 중계되는 실시간 장애물 | marking + clearing |
| `/zone_scan/roi_boundary_points` | ParkingZone의 열린 입구를 제외한 3면 테두리 | marking only |

ROI 폴리곤 전체를 장애물로 채우지 않는다. `parking_Zone` 파일의 네 꼭짓점으로
구성한 테두리 중 차량 진입에 필요한 열린 변을 제외한 3면만 일정 간격의 점으로
샘플링한다. 각 점은 lethal obstacle로 마킹되고 `inflation_layer`가 주변에 비용
그라디언트를 만든다. 따라서 ROI 내부와 입구는 free space로 남는다.

```text
ROI 3면 테두리 → LETHAL_OBSTACLE(254)
테두리 주변    → inflation cost(1~253)
ROI 내부·입구  → FREE_SPACE(0)
```

ROI 테두리 source는 `clearing: false`이므로 기하학적 경계를 유지한다. LiDAR source는
`clearing: true`이며, `zone_scan`이 `/parkingScan=true`인 동안만 원본 LiDAR를
`/zone_scan/cost_lidar_points`로 중계한다.

### Scan 시작·재정비 전환 시 비용 초기화

`/parkingScan`이 `false → true`로 바뀌면 `zone_scan`이 다음 서비스를 호출한다.

```text
/global_costmap/clear_entirely_global_costmap
```

초기화 완료 직후 `/zone_scan/roi_boundary_points`를 다시 발행한다. 결과적으로 이전
주차구역과 이전 주행에서 남은 cost는 삭제되고, 다음 항목만 다시 생성된다.

- 항상 복원되는 ParkingZone 3면 테두리 cost
- 초기화 이후 실시간 LiDAR가 새로 관측한 장애물 cost

`/parkingScan`이 `true → false`로 바뀌면 새 LiDAR 중계만 즉시 멈춘다. Scan 중
생성된 LiDAR cost는 `/parkingMode=true`인 주차 동안 그대로 유지되어 Smac 경로
생성에 사용된다. 주차가 끝나 `/parkingMode`가 `true → false`로 바뀌는 재정비
상태에서 global costmap을 초기화하고 ROI 3면 테두리만 복원한다. clear service가
아직 준비되지 않았으면 시작·재정비 요청 모두 0.5초 간격으로 재시도한다.

## 4. 후진 전용 DUBIN 경로

DUBIN은 전진 곡선만 생성한다. 플래너에는 `주차 목표 → 현재 자세`의 전진 경로를
요청하고, 결과 pose 배열을 뒤집어 `현재 자세 → 주차 목표`로 사용한다. 따라서
전진/후진 전환점 없이 전체 구간을 후진한다.

Smac은 다음 항목을 경로 생성에 사용한다.

- 차량 footprint
- 최소 회전반경 3.3 m
- 실시간 LiDAR global cost
- ROI 열린 입구를 제외한 3면 테두리 global cost
- inflation 비용
- 목표 위치와 yaw

## 5. RViz 시각화

실제 사용하는 `ros2_sensor/rviz/ros2_sensor.rviz`에 다음 Display가 등록되어 있다.

| Display | 토픽 | 의미 |
|---|---|---|
| `Global Costmap` | `/global_costmap/costmap` | lethal 및 inflation 비용지도 |
| `Smac Reverse Path` | `/parking_path/smac_reverse` | MPPI에 보내는 후진 경로 |
| `Final Reverse Path` | `/parking_path/final_reverse` | 고정 E 직선 후진 경로 |
| `Exit Straight Path` | `/parking_path/exit_straight` | Zone1 조향 전 1차 goal 복귀 경로 |
| `Exit Forward Path` | `/parking_path/exit_forward` | 저장된 gate 좌표까지의 전진 출차 경로 |

```bash
rviz2 -d ~/carla/ros2_sensor/rviz/ros2_sensor.rviz \
  --ros-args -p use_sim_time:=true
```

RViz의 Fixed Frame은 `map`이어야 한다. global costmap은 `Draw Behind=true`,
`Alpha=0.65`, `Color Scheme=costmap`으로 설정되어 있다. 겹쳐 보이지 않도록
`Local Costmap` Display는 기본적으로 꺼 두었다.

경로 발행만 끄려면 다음 파라미터를 `false`로 설정한다.

```yaml
follow_path_client:
  ros__parameters:
    parking_path_visualization_enabled: false
```

## 6. 주요 파라미터 영향

### 목표점 탐색 (`point_parking.yaml`)

| 파라미터 | 증가시키면 | 감소시키면 |
|---|---|---|
| `min_gap_width` | 넓은 공간만 후보로 인정 | 좁은 틈도 후보가 됨 |
| `wall_merge_distance` | 떨어진 점도 같은 벽으로 병합 | 벽이 조각날 수 있음 |
| `wall_padding` | 벽 비용 구간이 넓고 gap이 좁게 계산 | gap이 넓게 계산되나 벽 여유 감소 |
| `wall_search_depth` | 열린 변 안쪽의 먼 점까지 사용 | 입구 근처 점만 사용 |
| `wall_behind_margin` | 열린 변 바깥쪽 점도 사용 | zone 안쪽 점 위주로 사용 |
| `roi_endpoint_margin` | 후보선이 zone 양끝보다 확장 | `0.0`이면 실제 열린 변과 일치 |
| `min_zone_wall_points` | 노이즈에 강하지만 점 부족 시 후보 없음 | 적은 점으로 후보 생성, 오검출 가능 |
| `zone1_first_parking_south_offset` | Zone1 목표가 더 남쪽으로 이동 | gap 중앙에 가까워짐 |

### Smac (`planner_server.GridBased`)

| 파라미터 | 증가시키면 | 감소시키면 |
|---|---|---|
| `minimum_turning_radius` | 더 완만한 회전만 허용 | 더 급한 회전 허용 |
| `angle_quantization_bins` | yaw 탐색 정밀, 계산량 증가 | 빠르지만 자세 해상도 감소 |
| `non_straight_penalty` | 직선 경로 선호 | 곡선 사용 부담 감소 |
| `cost_penalty` | global cost 고비용 영역을 강하게 회피 | 경로 길이를 상대적으로 우선 |
| `tolerance` | 목표 주변 넓은 범위를 허용 | 목표에 정밀하지만 탐색 실패 증가 가능 |
| `max_planning_time` | 어려운 경로를 더 오래 탐색 | 빠르게 실패 처리 |
| `smooth_path` | 내장 스무딩 적용 | Hybrid 탐색 원형 유지 |

### Global costmap

| 파라미터 | 증가시키면 | 감소시키면 |
|---|---|---|
| `resolution` | 셀이 커져 가볍지만 형상이 거칠어짐 | 좁은 공간 표현 정밀, 메모리·계산 증가 |
| `inflation_radius` | 장애물에서 더 멀리 경로 생성 | 좁은 공간 통과 가능, 안전여유 감소 |
| `cost_scaling_factor` | 비용이 장애물 근처에서 빠르게 감소 | 비용이 먼 거리까지 완만하게 유지 |
| `cost_penalty` | inflation cost의 경로 영향 증가 | 장애물 비용보다 짧은 경로를 선호 |
| `roi_boundary_resolution` | 값이 커지면 테두리 점이 성기고 가벼움 | 값이 작아지면 테두리가 연속적이나 점 수 증가 |
| `zone_scan_roi_boundary.obstacle_max_range` | map 원점에서 더 먼 ROI 테두리까지 마킹 | 먼 주차구역 테두리가 제외될 수 있음 |
| `clear_global_costmap_on_scan_start` | `true`이면 새 scan 시작마다 과거 global cost 초기화 | `false`이면 이전 cost를 유지 |
| `clear_global_costmap_on_parking_mode_stop` | `true`이면 재정비 전환 때 LiDAR global cost 제거 | `false`이면 완료된 주차의 cost가 다음 상태에도 남음 |
| `zone1_global_inflation_radius` | Zone1 및 일반 상태의 global 회피반경 증가 | 장애물에 더 가까운 경로 허용 |
| `zone2_global_inflation_radius` | Zone2 전용 global 회피반경 증가 | Zone2의 좁은 공간 경로 생성 여유 증가 |

`minimum_turning_radius`와 footprint는 Smac, MPPI, global/local costmap에서 동일한
차량 실측값을 사용해야 한다.

## 7. 주의사항

- `zone_scan`이 실행되지 않으면 ROI 테두리와 gated LiDAR source가 모두 없어 global
  obstacle layer에 새 장애물이 입력되지 않는다.
- 열린 입구도 테두리 cost로 닫으면 Smac이 주차구역에 진입할 수 없으므로 현재는 3면만 발행한다.
- global costmap은 주차 경로 생성에 사용되며, local costmap obstacle layer가
  꺼져 있으면 MPPI 추종 중의 실시간 장애물 회피와는 별개다.
- 최종 E 직선 경로는 Smac을 다시 거치지 않고 직접 생성된다.
