# Point Parking 노드

`/point_parking`은 `/zone_scan/occupied_points`에 누적된 벽점을 이용해 빈 주차공간
후보를 만든다. 한 점짜리 순간 검출이 아니라 Zone Scan에서 ROI와 ICP를 거쳐
고정된 voxel 중심점을 입력으로 사용한다.

## 탐색 기준

ParkingZone1의 열린 변은 `좌전방 -> 우전방`, ParkingZone2의 열린 변은
`좌전방 -> 좌후방`이다. 각 열린 변을 주차공간이 나열된 1차원 탐색선으로 사용한다.

1. map 누적점을 `/utm_datum`으로 절대 UTM 좌표로 복원한다.
2. 열린 변 양끝을 `roi_endpoint_margin`만큼 확장해 ROI 끝점으로 사용한다.
3. 열린 변에서 `wall_search_depth` 안쪽에 있는 점만 탐색선에 투영한다.
4. 투영점 사이가 `wall_merge_distance` 이하면 같은 벽 구간으로 합친다.
5. ROI 끝점과 첫 벽, 인접한 두 벽, 마지막 벽과 ROI 끝점 사이를 검사한다.
6. 빈 구간 폭이 `min_gap_width` 이상이면 그 구간 중심을 후보 goal로 만든다.
7. 후보 중 폭이 가장 큰 공간을 대표 goal로 선택한다.

벽점이 하나도 없으면 ROI끝-ROI끝 공간을 임의의 빈 주차공간으로 판단하지 않는다.
`min_zone_wall_points` 이상의 벽점이 관측된 구역만 후보를 만든다.

## Goal과 yaw

대표 토픽 `/point_parking/goal_utm_yaw`의 배열 순서는 다음과 같다.

```text
[UTM_Easting, UTM_Northing, yaw_rad]
```

yaw는 UTM 평면에서 동쪽이 0 rad이고 반시계 방향이 양수다. ParkingZone 폴리곤
중심에서 열린 변을 바라보는 바깥쪽 방향을 사용한다. RViz Marker는 UTM을 datum
상대 `map` 좌표로 변환하고 CARLA/ROS Y 반전을 적용해 표시한다.

대표 goal은 가장 넓은 gap이며, 모든 후보는 `/point_parking/goal_candidates`에
`PoseArray`로 발행한다. 후보가 사라지면 `/point_parking/goal_valid=false`와 빈
`goal_utm_yaw` 배열을 발행하여 downstream 제어기가 오래된 latched goal을 사용하지
않도록 한다.

## 주요 설정

```yaml
min_gap_width: 1.5
wall_merge_distance: 0.35
wall_padding: 0.1
wall_search_depth: 4.0
wall_behind_margin: 1.0
roi_endpoint_margin: 1.0
min_zone_wall_points: 3
```

- `wall_merge_distance`: 인접 투영점을 같은 벽으로 합칠 최대 간격
- `wall_padding`: 벽점 하나가 탐색선에서 차지하는 좌우 폭
- `wall_search_depth`: 열린 변에서 안쪽으로 벽을 사용할 최대 깊이
- `wall_behind_margin`: 열린 변 바깥쪽 센서점 허용 범위
- `roi_endpoint_margin`: ParkingZone 열린 변 양끝의 ROI 확장 거리
