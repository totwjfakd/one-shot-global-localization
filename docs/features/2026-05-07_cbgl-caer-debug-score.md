# CBGL CAER Debug Score

## 요청

- `inf` skip 이후 CAER 후보군이 잘못 잡히는지 확인하기 위해 후보별 score, valid pair 수, coverage, 현재 TF pose 기준 score/rank를 출력한다.

## 변경 내용

- `cbgl/src/cbgl_node/cbgl.cpp`
  - CAER와 동일한 valid-pair 규칙을 사용하는 debug 통계 helper를 추가했다.
  - `siftThroughCAERPanoramic()`에서 CAER top-k 후보별 `score`, `sum`, `mean`, `valid_pairs`, `coverage`, pose를 로그로 출력한다.
  - 현재 `/map -> /base_footprint` TF pose에 대한 CAER score/rank도 함께 출력한다.
  - scan callback 전처리 단계에서 raw/processed scan의 finite, inf, nan, range_max 초과 개수를 출력한다.
- `cbgl/include/cbgl_node/cbgl.h`
  - CAER debug 통계 구조체와 helper 선언을 추가했다.

## 확인 결과

- `/scan`과 `/scan_known`을 Python/rospy로 직접 읽으면 downsample 기준 180개 중 `inf=26`, `over=0`, `range_max=20`으로 보였다.
- CBGL C++ callback 내부 debug 로그에서는 같은 scan이 `raw finite=180`, `inf=0`, `over_max=26`으로 잡혔다.
- 따라서 CAER의 `std::isfinite(sr[i])` 기반 skip이 현재 CBGL translation unit에서는 기대대로 `inf`를 제외하지 못하고 있다.
- 그 결과 `range_max`를 초과한 beam 26개가 CAER valid pair에 포함되고, top-k 후보 및 current TF pose 모두 `score=inf`, `sum=inf`로 포화된다.
- 실패 케이스에서는 CAER top-k 후보가 이미 `(1.594932, -9.536757)` 부근으로 몰린 뒤 ICP로 넘어간다.

## 검증

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 결과: 성공. `All 1 packages succeeded`.

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 170 cbgl
```

- 결과: CAER debug 로그와 scan preprocessing 로그 출력 확인.

## 남은 판단

- 다음 수정은 `std::isfinite` 대신 명시적인 `isinf`/`nan`/`range_max` 기반 판정으로 CAER와 scan preprocessing의 invalid beam 판단을 고치는 방향이 유력하다.
- 이 변경은 CAER scoring 동작 변경이므로 별도 계획과 승인 후 진행한다.
