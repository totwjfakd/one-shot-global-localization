# 작업 계획서

## 요청 요약

- 사용자 요청: `/scan_known` 생성은 외부 프로젝트에서 처리하므로 이 저장소에서는 찾거나 수정하지 않는다. CBGL은 `/scan_known`에서 `-1.0`으로 표시된 beam만 지도에 없는 장애물 제거 beam으로 보고 CAER에서 제외한다.
- 목표: `/robot/cbgl_node/global_localization_known` 실험에서 실제 lidar no-hit/free-space 정보는 `range_max`로 CAER에 남기고, unknown obstacle 제거 marker인 `-1.0`만 skip한다.
- 제외 범위: `/scan_known` publisher, ROS 2 시뮬레이션, bridge 설정, `relocalization_toolbox/`는 수정하지 않는다. `/robot/cbgl_node/global_localization` raw scan 동작은 변경하지 않는다.

## 현재 상태 확인

- 확인한 파일/토픽/서비스:
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `cbgl/include/cbgl_node/cbgl.h`
  - `cbgl/configuration_files/params_cbgl.yaml`
  - 서비스: `/robot/cbgl_node/global_localization`, `/robot/cbgl_node/global_localization_known`
  - 입력 토픽: `/scan`, `/scan_known`
- 관측 요약:
  - 현재 CBGL known scan preprocessing은 `nan` 또는 `range_max` 초과 값을 `inf` sentinel로 유지한다.
  - 현재 known CAER 정책은 `0 < range <= range_max`인 beam만 valid로 보고, `inf`, `nan`, `range_max` 초과, `<=0`을 skip한다.
  - 이 정책은 unknown 제거 beam과 실제 lidar no-hit beam이 모두 `inf` 계열로 들어오면 둘 다 CAER에서 제외하는 문제가 있다.
- 관련 제약:
  - `/scan_known` 생성부는 외부 프로젝트에 있으므로 이 저장소에서 수정하지 않는다.
  - scan preprocessing 변경은 CAER, ICP/CSM, debug publisher 영향을 분리해서 기록한다.
  - 구현 전 사용자 승인 후 진행한다.

## 영향 범위

- 패키지:
  - `cbgl/`
- 파일 후보:
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `cbgl/include/cbgl_node/cbgl.h`
  - `docs/features/2026-05-07_HHMM_known-scan-negative-marker.md`
- ROS topic/service/frame:
  - `/scan_known`: external publisher가 `-1.0` marker와 `range_max` no-hit을 구분해서 제공한다고 전제한다.
  - `/robot/cbgl_node/global_localization_known`: known scan source 소비 정책만 변경한다.
  - `/robot/cbgl_node/global_localization`: raw scan source 정책은 유지한다.
  - frame, namespace, service 이름은 변경하지 않는다.
- Docker/launch/parameter 영향:
  - launch, YAML, Docker 설정 변경 없음.
  - 새 파라미터는 추가하지 않는다. marker 값은 실험 계약으로 `-1.0`을 사용한다.

## 구현 설계안

- 변경 방식:
  - known scan preprocessing에서 `-1.0` marker는 그대로 보존한다.
  - known scan에서 `nan`, `+inf`, `range_max` 초과 값은 실제 no-hit/free-space 정보로 보고 `range_max`로 정규화한다.
  - known CAER valid-pair 판정은 `-1.0` marker를 명시적으로 skip하고, 그 외 값은 raw 정책과 같은 `range_max` 정규화 후 비교한다.
  - debug 로그의 `scan_negative`, `raw_negative`, `processed_negative`는 유지해 marker 개수를 확인한다.
- 대안:
  - `/scan_known` 생성부에서만 모든 encoding을 완성하고 CBGL은 현 정책을 유지한다.
  - `unknown_removed_range_marker` 파라미터를 추가한다.
- 선택 이유:
  - 생성부가 외부 프로젝트에 있으므로 CBGL 쪽은 소비 계약만 명확히 한다.
  - 새 파라미터를 추가하지 않으면 launch/YAML 변경 없이 실험 조건을 단순하게 유지할 수 있다.
  - 실제 no-hit을 `range_max`로 CAER에 남기면 free-space 정보가 사라지지 않는다.
- 리스크:
  - 외부 publisher가 unknown 제거 beam을 `inf`로 계속 보내면 CBGL은 이를 no-hit으로 보고 `range_max`로 비교하게 된다.
  - 외부 publisher가 `-1.0` 외 다른 음수 marker를 쓰면 이번 정책에서는 실험 의도와 어긋날 수 있다.
  - `-1.0` marker가 너무 많으면 known 서비스 CAER coverage가 낮아져 후보 선별이 불안정해질 수 있다.

## 검증 계획

- 빌드:
  - `docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'`
- 실행/서비스 호출:
  - `docker restart cbgl`
  - `docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'`
  - raw 비교가 필요하면 `/robot/cbgl_node/global_localization`도 호출한다.
- topic/frame 확인:
  - `/scan_known` sample에서 `-1.0` marker 개수와 `range_max` no-hit 개수를 확인한다.
  - CBGL 로그에서 `processed_negative`, `scan_negative`, `scan_inf`, `scan_over_max`, `valid_pairs`, `coverage`를 확인한다.
- 실패 시 확인할 항목:
  - `/scan_known` external publisher가 unknown 제거 beam을 실제로 `-1.0`으로 보내는지 확인한다.
  - `/scan_known`의 `range_max`, angle metadata, frame id가 `/scan`과 동일한지 확인한다.
  - known 서비스에서 `valid_pairs`가 지나치게 낮아지는지 확인한다.

## 문서화 계획

- 작업 계획 문서: `docs/plans/2026-05-07_2117_known-scan-negative-marker-plan.md`
- 기능 결과 문서: `docs/features/2026-05-07_HHMM_known-scan-negative-marker.md`
- 운영 문서 갱신: 없음

## 승인 상태

- 사용자 승인 문구: 진행해
- 승인 시각: 2026-05-07 21:25
