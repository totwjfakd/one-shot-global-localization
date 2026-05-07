# CBGL Source Specific CAER Invalid Policy

## 요청

- `/robot/cbgl_node/global_localization`은 기존 `/scan` 기반 CBGL 동작을 유지한다.
- `/robot/cbgl_node/global_localization_known`은 `/scan_known`에서 unknown 장애물 때문에 제거된 beam만 CAER에서 제외한다.
- 두 서비스를 사용해 동적 장애물 포함 scan과 unknown 제거 scan의 성능을 비교할 수 있게 한다.

## 변경 내용

- `cbgl/src/cbgl_node/cbgl.cpp`
  - CAER range 처리 helper를 source별 정책으로 분리했다.
  - raw `/scan`은 invalid/no-hit/range_max 초과 값을 `range_max`로 정규화해 기존 CBGL 방식으로 비교한다.
  - known `/scan_known`은 robot scan 쪽 invalid beam만 CAER pair에서 제외한다.
  - known 서비스에서도 map raycast 쪽 no-hit/range_max 초과 값은 `range_max`로 정규화해 비교한다.
  - raw scan preprocessing은 `range_max` clamp를 복원했고, known scan preprocessing은 invalid sentinel을 유지한다.
- `cbgl/include/cbgl_node/cbgl.h`
  - source별 CAER range helper 선언을 추가했다.

## 동작 정책

- `global_localization`
  - `/scan` 사용.
  - robot scan과 map raycast 모두 invalid/no-hit을 `range_max`로 맞춘다.
  - CAER는 180개 beam 전체를 사용할 수 있다.
- `global_localization_known`
  - `/scan_known` 사용.
  - robot scan 쪽 invalid beam은 skip한다.
  - map raycast 쪽 no-hit은 `range_max`로 유지한다.
  - 현재 실험 scan에서는 180개 중 154개 beam만 CAER에 사용된다.

## Unknown 과 Lidar Max Range 구분

- 현재 `/scan_known`에서 unknown 장애물 제거 beam과 실제 라이다 탐지거리 초과 beam이 모두 `inf` 또는 `range_max` 초과 sentinel로 들어오면 CBGL은 둘을 구분할 수 없다.
- 더 정확한 실험을 하려면 `/scan_known` 생성 단계에서 둘을 구분하는 편이 좋다.
  - unknown 장애물 때문에 제거된 beam: `-1.0` 같은 `range_min`보다 작은 값으로 표시한다.
  - 실제 라이다 no-hit 또는 탐지거리 초과 beam: `range_max`로 표시한다.
- 이번 CBGL helper는 known scan에서 `<= 0` 값을 invalid로 보기 때문에, 향후 unknown을 `-1.0`으로 보내도 CAER에서 자동 skip된다.

## 검증

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 결과: 성공. `All 1 packages succeeded`.

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
docker logs --tail 220 cbgl
```

- 결과:
  - raw scan preprocessing: `processed ... over_max=0`
  - CAER summary: `scan_valid=180`, `valid_pairs[min,max]=[180,180]`, `coverage=1.000`

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 170 cbgl
```

- 결과:
  - known scan preprocessing: `processed ... over_max=26`
  - CAER summary: `scan_valid=154`, `valid_pairs[min,max]=[154,154]`, `coverage=0.856`

## 남은 이슈

- `/scan_known`에서 unknown 제거 beam과 실제 lidar no-hit beam이 모두 `inf`로 들어오면 known 서비스는 둘 다 skip한다.
- 실제 no-hit을 CAER free-space 정보로 살리고 싶다면 `/scan_known` 생성 쪽에서 unknown은 `-1.0`, 실제 no-hit은 `range_max`로 분리해야 한다.

## 후속 Debug 보강

- `docs/features/2026-05-07_cbgl-negative-scan-debug.md`에서 `-1.0` 같은 negative range marker를 로그에 표시하도록 보강했다.
- scan preprocessing 로그에는 `raw_negative`, `processed_negative`가 출력된다.
- CAER summary/top/current TF 로그에는 `scan_negative`, `map_negative`가 출력된다.
