# CBGL Negative Scan Debug Plan

## 요청 요약

- `/scan_known`에서 unknown 장애물 제거 beam을 `-1.0`으로 표시할 수 있다.
- 현재 CBGL 동작은 `-1.0`을 known scan invalid beam으로 skip하지만, debug 로그에는 이 값이 별도로 표시되지 않는다.
- scan preprocessing 로그와 CAER debug 로그에 negative scan range 카운트를 추가한다.

## 영향 범위

- 패키지: `cbgl/`
- 파일 후보:
  - `cbgl/include/cbgl_node/cbgl.h`
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `docs/features/2026-05-07_cbgl-negative-scan-debug.md`
  - `docs/features/2026-05-07_cbgl-source-specific-caer-invalid-policy.md`

## 구현 설계안

- scan preprocessing debug:
  - raw scan과 processed scan 각각에서 `range < 0.0`인 beam 수를 출력한다.
  - `-1.0` unknown marker는 이 카운트에 포함된다.
- CAER debug:
  - 후보별/top-k/current TF debug에 `scan_negative`, `map_negative` 카운트를 추가한다.
  - summary에도 active scan의 negative beam 수를 출력한다.
- 알고리즘 동작:
  - 기존 source별 CAER 정책은 변경하지 않는다.
  - known scan에서 `-1.0`은 기존처럼 invalid로 skip된다.
  - raw scan에서 `-1.0`은 기존 `range <= 0.0` invalid 처리와 동일하게 CAER pair에 들어가지 않는다.

## 검증 계획

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 로그 형식 확인:

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 170 cbgl
```

## 승인 상태

- 사용자 요청 문구: "debug에 -1 값을 가지는 scan range도 표시해줘야 할 것 같아 문서도 업데이트 하고"
- 승인 시각: 2026-05-07
