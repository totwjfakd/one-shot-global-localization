# CBGL CAER Valid Range Guard

> 후속 변경: 이 문서의 valid range guard는 이후 source별 정책으로 세분화됐다. 현재 최종 동작은 `global_localization`은 기존 CBGL처럼 `range_max` clamp를 사용하고, `global_localization_known`만 robot scan invalid beam을 skip한다. 자세한 내용은 `docs/features/2026-05-07_cbgl-source-specific-caer-invalid-policy.md`를 참고한다.

## 요청

- `inf` skip 이후 CAER 후보군이 잘못 잡히고, CAER score가 `inf`로 출력되는 문제를 확인했다.
- CAER에서 실제 valid beam만 점수에 포함되도록 valid beam 판정 조건을 추가한다.

## 변경 내용

- `cbgl/src/cbgl_node/cbgl.cpp`
  - CAER 전용 valid range helper를 추가했다.
  - CAER score 계산에서 `0 < range <= range_max`인 beam pair만 합산하도록 변경했다.
  - CAER debug 통계도 같은 helper를 사용하도록 맞췄다.
  - `range_max` 초과 beam 카운트는 valid 여부와 별개로 계속 기록해, invalid beam이 얼마나 들어오는지 확인할 수 있게 했다.
- `cbgl/include/cbgl_node/cbgl.h`
  - CAER valid range helper 선언을 추가했다.

## 동작 영향

- CAER ranking score는 기존처럼 valid pair들의 절대 range 차이 합계를 사용한다.
- 평균 score, coverage penalty, top-k 선정 방식은 변경하지 않았다.
- 변경된 부분은 "어떤 beam을 CAER 합계에 포함할지"뿐이다.
- 따라서 정상적인 finite range beam에는 기존 CAER 계산과 같은 값이 적용된다.
- `inf`, `nan`, `range_max` 초과 sentinel처럼 지도 feature와 직접 비교하면 안 되는 beam은 CAER 합계에서 제외된다.

## 확인 결과

- 수정 전 CBGL 로그:
  - `scan_valid=180`, `valid_pairs=180`, `score=inf`
  - `/scan_known`에는 unknown obstacle 제거로 인한 invalid beam이 있었지만 CAER에 포함됐다.
- 수정 후 CBGL 로그:
  - `scan_valid=154`
  - `valid_pairs[min,max]=[154,154]`
  - `overflow_scores=0`
  - top-k 후보와 current TF pose score가 모두 finite 값으로 출력됐다.
- `scan_over_max=26`은 계속 관측되지만, 이 26개 beam은 CAER valid pair에서 제외된다.

## 검증

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 결과: 성공. `All 1 packages succeeded`.

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 220 cbgl
```

- 결과: `global_localization_known` 호출 후 CAER debug 로그에서 valid beam guard 적용을 확인했다.
- 최종 pose 출력도 확인했다.

## 남은 이슈

- 이번 수정은 CAER의 `inf` score 포화 문제를 막는 변경이다.
- 후보군 자체가 실제 pose 주변으로 잘 잡히는지는 map origin, scan convention, known scan 생성 방식까지 포함해 별도로 판단해야 한다.
