# CBGL Map Origin 디버깅 메모

이 문서는 `map_scan`이 전부 `20.0` 또는 `0.0`으로 나오는 문제를 추적할 때 참고하는 메모다.

전체 global localization 실패 원인과 최종 수정 결과는
[`CBGL_GLOBAL_LOCALIZATION_DEBUGGING_KO.md`](CBGL_GLOBAL_LOCALIZATION_DEBUGGING_KO.md)에 정리되어 있다.

## 샘플 맵

현재 샘플 맵은 다음 파일이다.

```text
sample_map/v2.yaml
sample_map/v2.pgm
```

`sample_map/v2.yaml`:

```yaml
image: v2.pgm
mode: trinary
resolution: 0.05
origin: [-15, -24.1, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

`v2.pgm`은 599 x 982 pixel map이고, resolution은 0.05 m/pixel이다. 따라서 ROS map 좌표 범위는 대략 다음과 같다.

```text
x: -15.0 ~ 14.95
y: -24.1 ~ 25.0
```

## Origin 해석

ROS `map.yaml`의 `origin`은 이미지 좌하단 pixel의 world pose이다.

정답 pose가 실제 map frame에서 `(2, 2, 0)`이면, 이미지 좌하단 기준 좌표는 다음처럼 해석된다.

```text
image_frame_x = 2 - (-15.0) = 17.0
image_frame_y = 2 - (-24.1) = 26.1
```

YAML origin을 임의로 `[0, 0, 0]`으로 바꾸면 같은 이미지 pixel의 map 좌표가 바뀐다. 이 경우 Gazebo/world 기준 정답 pose와 비교할 때 offset을 함께 고려해야 한다.

## Map Scan 증상

- `map_scan`이 전부 `20.0`이면 raycaster가 장애물을 보지 못하고 있다는 뜻이다.
- `map_scan`이 전부 `0.0`이면 raycaster가 시작점을 occupied cell 안으로 보고 있을 가능성이 높다.
- 두 증상 모두 CAER/ICP 이전에 ROS OccupancyGrid, RangeLib OMap, raycast grid 좌표 변환을 먼저 확인해야 한다.

## 확인 순서

1. `/map/info`에서 width, height, resolution, origin을 확인한다.
2. `/scan.header.frame_id`와 `base_footprint -> lidar_link` TF를 확인한다.
3. `world_scan`과 `map_scan`의 range 분포를 비교한다.
4. `scanMap()`에서 world 좌표가 grid 좌표로 변환되는 방식을 확인한다.
5. `mapCallback()`에서 ROS OccupancyGrid가 RangeLib OMap으로 들어갈 때 y축 방향이 맞는지 확인한다.

## 주의

벽 자체를 feature에서 제거하는 방식은 권장하지 않는다. 벽, 코너, 복도는 lidar localization에서 중요한 정보다. 먼저 `map_scan`이 정상적인 거리 분포를 가지는지 확인하고, 그 다음에 CAER/ICP scoring이나 후보 생성 문제를 분리해서 본다.
