# CBGL CAER Debug Score Plan

## 요청 요약

- 사용자 요청: `inf` skip 이후 CAER가 후보군을 잘못 뽑는지 확인하기 위해 후보별 score와 유효 beam 통계를 출력한다.
- 목표: CAER 후보 선별 단계에서 실제 점수 분포, valid pair 수, coverage, 현재 TF pose 기준 score/rank를 로그로 확인한다.
- 제외 범위: 이번 작업에서는 CAER scoring 알고리즘 자체를 변경하지 않는다. `inf` 처리 정책, top-k 선택 방식, ICP/CSM 파라미터도 변경하지 않는다.

## 현재 상태 확인

- 확인한 파일/토픽/서비스:
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `cbgl/include/cbgl_node/cbgl.h`
  - `/scan`, `/scan_known`
  - `/robot/cbgl_node/global_localization`
  - `/robot/cbgl_node/global_localization_known`
- 관측 요약:
  - 현재 `/scan`과 `/scan_known`은 한 시점 기준 동일했다.
  - 입력 scan은 720개 중 `inf` 107개, undersample 이후 180개 중 finite valid 154개였다.
  - 실패 로그에서는 CAER top 후보가 이미 잘못된 위치로 모인 뒤 ICP correspondences가 거의 0으로 실패했다.
- 관련 제약:
  - CBGL 알고리즘 변경은 작은 단위로 제한한다.
  - 디버깅 로그 추가와 scoring 변경을 분리한다.

## 영향 범위

- 패키지: `cbgl/`
- 파일 후보:
  - `cbgl/include/cbgl_node/cbgl.h`
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `docs/features/2026-05-07_cbgl-caer-debug-score.md`
- ROS topic/service/frame:
  - 새 topic/service/frame은 추가하지 않는다.
  - 현재 pose 기준 score는 가능하면 `/map -> /base_footprint` TF를 읽어 계산한다.
- Docker/launch/parameter 영향:
  - Docker, launch, YAML parameter 변경 없음.

## 구현 설계안

- 변경 방식:
  - CAER 합계와 valid pair 수를 함께 계산하는 debug helper를 추가한다.
  - `siftThroughCAERPanoramic()`에서 후보별 CAER 계산 시 `caer_sum`, `valid_pairs`, `mean_caer`, `coverage`를 함께 수집한다.
  - top-k 후보의 score 통계를 로그로 출력한다.
  - 현재 TF pose를 읽을 수 있으면 같은 방식으로 현재 pose의 best yaw-adjusted score와 전체 후보 중 rank를 출력한다.
- 대안:
  - ROS service를 새로 추가해 수동으로 score를 조회할 수 있다.
  - 이번에는 빠른 원인 확인을 위해 서비스 추가 없이 기존 global localization 실행 로그에만 출력한다.
- 선택 이유:
  - 알고리즘 동작을 바꾸지 않고 CAER ranking 문제를 먼저 관측할 수 있다.
  - custom srv 추가나 launch 변경 없이 Docker 빌드만으로 확인 가능하다.
- 리스크:
  - 후보 수가 많기 때문에 모든 후보를 출력하면 로그가 과도해질 수 있다. top-k와 요약값 중심으로 제한한다.
  - 현재 TF가 없거나 map frame이 아직 없으면 현재 pose score는 warning만 남기고 건너뛴다.

## 검증 계획

- 빌드:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 실행/서비스 호출:

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
```

- topic/frame 확인:
  - `/scan`, `/scan_known` 입력 finite/inf 개수
  - `/map -> /base_footprint` TF 존재 여부
- 실패 시 확인할 항목:
  - 현재 pose score가 계산되지 않는 경우 TF frame 상태 확인
  - 현재 pose score는 좋은데 top-k rank가 낮은 경우 CAER scoring 정책 검토
  - valid pair coverage가 낮은 후보가 top-k를 차지하는 경우 `inf` skip 보정 방식 검토

## 문서화 계획

- 기능 결과 문서: `docs/features/2026-05-07_cbgl-caer-debug-score.md`
- 운영 문서 갱신: 필요 시 기존 CBGL 디버깅 문서에 로그 해석법을 추가한다.

## 승인 상태

- 사용자 승인 문구: "ㅇㅇ 저것들 좀 찍어보자"
- 승인 시각: 2026-05-07
