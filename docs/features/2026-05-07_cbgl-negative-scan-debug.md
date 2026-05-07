# CBGL Negative Scan Debug

## 요청

- `/scan_known`에서 unknown 장애물 제거 beam을 `-1.0`으로 표시할 수 있으므로, CBGL debug 로그에도 음수 range 개수를 표시한다.

## 변경 내용

- `cbgl/src/cbgl_node/cbgl.cpp`
  - scan preprocessing debug 로그에 `raw_negative`, `processed_negative` 카운트를 추가했다.
  - CAER debug summary에 `scan_negative` 카운트를 추가했다.
  - CAER top-k/current TF 로그에 `scan_negative`, `map_negative` 카운트를 추가했다.
- `cbgl/include/cbgl_node/cbgl.h`
  - `CAERDebugStats`에 scan/map negative ray 카운터를 추가했다.

## 로그 형식

```text
[CBGL] Scan debug known: ... raw_negative=N; ... processed_negative=N ...
[CBGL] CAER debug summary: ... scan_valid=N scan_negative=N ...
[CBGL] CAER debug top[0]: ... scan_negative=N map_negative=N ...
[CBGL] CAER debug current_tf: ... scan_negative=N map_negative=N ...
```

## 동작 영향

- 알고리즘 동작은 변경하지 않았다.
- known scan에서 `-1.0`은 기존 source-specific CAER 정책에 따라 invalid beam으로 skip된다.
- 이번 변경은 해당 beam이 실제로 몇 개 들어왔는지 로그에 드러내는 디버깅 보강이다.

## 검증

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

- 결과: 성공. `All 1 packages succeeded`.

```bash
docker restart cbgl
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; timeout 5 rostopic echo -n 1 /scan_known'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
docker logs --tail 180 cbgl
```

- 결과:
  - `/scan_known` 수신 확인.
  - scan callback 로그에서 `raw_negative=0`, `processed_negative=0` 필드 출력 확인.
  - CAER summary/top/current TF 로그에서 `scan_negative=0`, `map_negative=0` 필드 출력 확인.

## 관측 메모

- 현재 확인 시점의 `/scan_known`에는 `-1.0` marker가 없었다.
- 따라서 negative 카운터는 모두 `0`으로 출력됐다.
- 향후 `/scan_known` 생성부가 unknown 제거 beam을 `-1.0`으로 보내면 이 값이 `raw_negative`, `processed_negative`, `scan_negative`에 반영된다.
