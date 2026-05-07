# CBGL Source Specific CAER Invalid Policy Plan

## 요청 요약

- `/robot/cbgl_node/global_localization`은 기존 `/scan` 기반 CBGL 동작을 유지한다.
- `/robot/cbgl_node/global_localization_known`은 `/scan_known`에서 unknown 장애물 때문에 제거된 beam만 CAER에서 제외한다.
- 두 서비스가 같은 `inf` skip 정책을 공유하면, 동적 장애물 포함 scan과 unknown 제거 scan의 성능 비교가 공정하지 않다.

## 현재 상태 확인

- 현재 CAER valid guard는 active scan source와 무관하게 `0 < range <= range_max`를 scan/map 양쪽에 적용한다.
- 이 상태에서는 raw `/scan` 서비스도 `inf`/range_max 초과 beam을 skip하므로 기존 CBGL과 달라진다.
- 기존 CBGL은 scan callback에서 `nan` 또는 `range_max` 초과 값을 `range_max`로 바꾸고, CAER 합계에 포함했다.

## 영향 범위

- 패키지: `cbgl/`
- 파일 후보:
  - `cbgl/include/cbgl_node/cbgl.h`
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `docs/features/2026-05-07_cbgl-source-specific-caer-invalid-policy.md`
- ROS topic/service/frame:
  - topic/service/frame 추가 없음.
  - 기존 `/robot/cbgl_node/global_localization`, `/robot/cbgl_node/global_localization_known`의 source별 처리만 분리한다.

## 구현 설계안

- scan preprocessing:
  - raw `/scan`: `nan` 또는 `range_max` 초과 값을 `range_max`로 clamp한다.
  - known `/scan_known`: `nan` 또는 `range_max` 초과 값을 skip 가능한 invalid sentinel로 유지한다.
- CAER scoring:
  - raw service: robot scan과 map raycast 모두 invalid/no-hit을 `range_max`로 정규화해서 기존처럼 비교한다.
  - known service: robot scan 쪽 invalid beam은 pair 전체를 skip한다.
  - known service에서도 map raycast no-hit/초과 값은 `range_max`로 정규화한다.
- `-1` 같은 unknown 전용 marker:
  - 이번 CAER helper는 `<= 0` scan 값을 invalid로 보므로, 향후 `/scan_known`에서 unknown을 `-1`로 보낼 경우 자동으로 skip된다.
  - 다만 이번 변경은 `/scan_known` publisher의 encoding은 바꾸지 않는다.

## 검증 계획

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- raw service 확인:

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
docker logs --tail 220 cbgl
```

- known service 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 220 cbgl
```

## 승인 상태

- 사용자 승인 문구: "ㅇㅇ 그렇게 하자"
- 승인 시각: 2026-05-07
