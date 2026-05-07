# CBGL CAER Valid Range Guard Plan

## 요청 요약

- 사용자 요청: `inf` skip이 CAER에서 기대대로 동작하지 않으므로 valid beam 판정 조건을 추가한다.
- 목표: CAER 계산에서 `inf` 또는 `range_max` 초과 beam이 점수에 포함되지 않도록 `0 < range <= range_max` 조건을 명시적으로 적용한다.
- 제외 범위: CAER를 평균 score로 바꾸거나 top-k ranking 방식을 변경하지 않는다. ICP/CSM 파라미터도 변경하지 않는다.

## 현재 상태 확인

- 확인한 파일/토픽/서비스:
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `cbgl/include/cbgl_node/cbgl.h`
  - `/scan`, `/scan_known`
  - `/robot/cbgl_node/global_localization_known`
- 관측 요약:
  - Python/rospy에서는 `/scan_known` downsample 기준 `inf=26`, `over=0`으로 보인다.
  - CBGL 내부 debug 로그에서는 같은 beam이 `inf=0`, `over_max=26`으로 잡혔다.
  - CAER에서는 `scan_valid=180`, `valid_pairs=180`, `score=inf`가 출력되어 invalid beam이 skip되지 않았다.
- 관련 제약:
  - CBGL 알고리즘 변경은 작은 단위로 제한한다.
  - 이번 수정은 CAER valid 판정만 변경한다.

## 영향 범위

- 패키지: `cbgl/`
- 파일 후보:
  - `cbgl/include/cbgl_node/cbgl.h`
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `docs/features/2026-05-07_cbgl-caer-valid-range-guard.md`
- ROS topic/service/frame:
  - topic/service/frame 추가 없음.
- Docker/launch/parameter 영향:
  - Docker, launch, YAML parameter 변경 없음.

## 구현 설계안

- 변경 방식:
  - CAER 전용 helper를 추가해 `range > 0.0 && range <= latest_world_scan_->range_max`를 valid beam 조건으로 사용한다.
  - 기존 `std::isfinite(range) && range > 0.0` 조건을 helper 호출로 교체한다.
  - debug 통계도 같은 helper를 사용하게 하여 CAER scoring과 로그가 같은 기준을 보게 한다.
- 대안:
  - scan preprocessing에서 invalid beam을 전부 `range_max`로 되돌린다.
  - CAER score를 평균/coverage penalty 방식으로 변경한다.
- 선택 이유:
  - unknown obstacle beam을 skip한다는 현재 방향을 유지하면서, `inf`가 finite처럼 보이는 런타임 문제를 상한 조건으로 차단할 수 있다.
  - CAER score 구조 자체는 유지하므로 변경 범위가 가장 작다.
- 리스크:
  - `range_max` beam을 free-space 정보로 쓰던 원래 의미는 계속 제외된다.
  - skip 이후 CAER의 정보량 부족 문제는 남아 있을 수 있으므로, score 분포를 로그로 다시 확인한다.

## 검증 계획

- 빌드:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 실행/서비스 호출:

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 170 cbgl
```

- 확인 항목:
  - CAER debug `scan_valid`가 180이 아니라 실제 valid 수로 줄어드는지 확인한다.
  - top-k `score`가 `inf`로 포화되지 않는지 확인한다.
  - `valid_pairs`가 range_max 초과 beam을 제외하는지 확인한다.

## 문서화 계획

- 기능 결과 문서: `docs/features/2026-05-07_cbgl-caer-valid-range-guard.md`
- 운영 문서 갱신: 필요 시 후속 정리.

## 승인 상태

- 사용자 승인 문구: "ㅇㅇ valid beam 판정 조건을 추가하자"
- 승인 시각: 2026-05-07
