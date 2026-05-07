# Global Localization Known Scan Service

## 사용자 요청

기존 `/robot/cbgl_node/global_localization` 서비스는 `/scan` 토픽 기반으로 유지하고, 새 `/robot/cbgl_node/global_localization_known` 서비스는 `/scan_known` 토픽 기반으로 같은 CBGL global localization 알고리즘을 실행하도록 요청했다.

## 요구사항

- 기존 서비스와 `/scan` 동작은 유지한다.
- `/scan_known`을 구독하고 최신 known scan을 별도 캐시에 저장한다.
- 새 서비스 호출 시 `/scan_known` 기반 scan만 active scan으로 사용한다.
- `/scan_known` age threshold는 추가하지 않는다.
- CBGL CAER, raycasting, CSM/ICP scoring 알고리즘은 변경하지 않는다.

## 구현 내용

- `known_scan_topic` 파라미터를 추가했다. 기본값은 `/scan_known`이다.
- `global_localisation_known_service_name` 파라미터를 추가했다. 기본값은 `global_localization_known`이다.
- `/scan`과 `/scan_known`을 각각 구독하도록 subscriber를 분리했다.
- scan preprocessing을 `storeScan()`으로 공통화했다.
- raw scan과 known scan을 각각 `latest_raw_scan_`, `latest_known_scan_`으로 캐시한다.
- 서비스 호출 시 `requested_scan_source_`를 설정하고, 요청된 source의 callback만 `latest_world_scan_`을 활성화해 `processPoseCloud()`로 이어지게 했다.
- `/scan_known`이 아직 한 번도 들어오지 않은 상태에서 known 서비스를 호출하면 warning 후 실패하도록 했다.

## 변경 파일

- `cbgl/include/cbgl_node/cbgl.h`
- `cbgl/src/cbgl_node/cbgl.cpp`
- `cbgl/configuration_files/params_cbgl.yaml`
- `docs/plans/2026-05-07_global-localization-known-service-plan.md`
- `docs/features/2026-05-07_global-localization-known-service.md`

## 사용 방법

기존 scan 기반 실행:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
```

known scan 기반 실행:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
```

## 검증 결과

- 문법/공백 검사: `git diff --check` 통과.
- Docker 빌드: 성공.

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

결과: `All 1 packages succeeded`.

## 남은 리스크

- `/scan_known`과 `/scan`의 frame id, angle metadata, range metadata가 다르면 같은 알고리즘이어도 결과 차이가 생길 수 있다.
- known 서비스는 age threshold를 두지 않으므로, `/scan_known` publisher가 멈춘 상태에서는 마지막 known scan 이후 다음 known callback이 오지 않으면 실행이 진행되지 않을 수 있다.
