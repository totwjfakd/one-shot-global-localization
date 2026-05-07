# Known Scan Negative Marker

## 사용자 요청

- `/scan_known` 생성부는 외부 프로젝트에서 처리한다.
- 이 저장소의 CBGL은 `/scan_known`에서 `-1.0`으로 표시된 beam만 지도에 없는 장애물 제거 marker로 보고 CAER에서 제외한다.
- 실제 lidar no-hit/free-space 정보는 `range_max`로 CAER에 남긴다.

## 요구사항

- 기존 `/robot/cbgl_node/global_localization` raw scan 동작은 유지한다.
- `/robot/cbgl_node/global_localization_known`에서만 known scan source 정책을 조정한다.
- `/scan_known`의 `-1.0` marker는 scan preprocessing 중 보존한다.
- known scan의 `nan`, `+inf`, `range_max` 초과 값은 `range_max`로 정규화해 no-hit처럼 비교한다.
- `relocalization_toolbox/`, launch, YAML, Docker 설정은 수정하지 않는다.

## 구현 내용

- `cbgl/src/cbgl_node/cbgl.cpp`
  - `isKnownScanUnknownObstacleMarker()`를 추가해 `-1.0` marker를 명시적으로 식별한다.
  - `normalizeKnownScanRangeForCAER()`를 추가해 known scan에서 `-1.0` marker만 CAER pair에서 제외하고, 그 외 no-hit/over-range 값은 `range_max`로 정규화한다.
  - known scan preprocessing에서 `nan`, `+inf`, `range_max` 초과 값을 `range_max`로 정규화한다.
  - min-range garbage interpolation이 `-1.0` marker를 지우지 않도록 known scan에서는 marker 위치를 보존한다.
- `cbgl/include/cbgl_node/cbgl.h`
  - known scan marker/normalization helper 선언을 추가했다.
- `docs/plans/2026-05-07_2117_known-scan-negative-marker-plan.md`
  - 승인 전 설계안과 승인 상태를 기록했다.

## 동작 정책

- `/robot/cbgl_node/global_localization`
  - `/scan` 사용.
  - 기존 raw scan 정책 유지.
- `/robot/cbgl_node/global_localization_known`
  - `/scan_known` 사용.
  - scan range가 `-1.0`이면 unknown obstacle 제거 marker로 보고 CAER에서 제외한다.
  - scan range가 `range_max`이면 실제 no-hit/free-space 정보로 보고 CAER에 포함한다.
  - scan range가 `nan`, `+inf`, `range_max` 초과이면 `range_max`로 정규화한다.

## 검증 결과

문법/공백 검사:

```bash
git diff --check
```

- 결과: 성공.

Docker 기준 빌드:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 결과: 성공. `All 1 packages succeeded`.

새 바이너리 적용:

```bash
docker restart cbgl
```

- 결과: 성공.

known 서비스 호출:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; timeout 30 rosservice call /robot/cbgl_node/global_localization_known'
```

- 결과: 성공.

CBGL 로그 확인:

```text
Scan debug known: raw finite=180 inf=0 nan=0 over_max=3; raw_negative=64;
processed finite=180 inf=0 nan=0 over_max=0 processed_negative=64 range_max=20.000000

CAER debug summary: hypotheses=1251520 nrays=180 range_max=20.000000
scan_valid=116 scan_negative=64 valid_pairs[min,max]=[116,116]
zero_valid_pairs=0 overflow_scores=0
```

해석:

- `/scan_known`의 `-1.0` marker 64개가 preprocessing 이후에도 보존됐다.
- known scan의 over-range 3개는 `range_max`로 정규화되어 `processed over_max=0`이 됐다.
- CAER는 180개 중 64개 marker를 제외한 116개 beam을 valid pair로 사용했다.
- `scan_inf=0`, `overflow_scores=0`으로 CAER score 포화는 발생하지 않았다.

## 남은 리스크

- 외부 `/scan_known` publisher가 unknown 제거 beam을 `-1.0`이 아닌 값으로 보내면 이번 정책과 맞지 않는다.
- `-1.0` marker가 너무 많으면 CAER coverage가 낮아져 후보 선별 안정성이 떨어질 수 있다.
- `/scan_known`의 frame id, angle metadata, range metadata는 `/scan`과 같은 기준을 유지해야 한다.
