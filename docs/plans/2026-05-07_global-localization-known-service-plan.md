# Global Localization Known Scan Service Plan

## 요청 요약

- 사용자 요청: 기존 `/robot/cbgl_node/global_localization`은 `/scan`을 사용하고, 새 `/robot/cbgl_node/global_localization_known` 서비스는 `/scan_known`을 사용해 같은 CBGL global localization 알고리즘을 실행하게 한다.
- 목표: 기존 CBGL 알고리즘을 복제하지 않고, 서비스 호출에 따라 사용할 scan source만 `/scan` 또는 `/scan_known`으로 분리한다.
- 제외 범위: scan age threshold 추가는 하지 않는다. 사용자가 `/scan_known` 주파수를 높였으므로 최신성 제한 로직은 이번 설계에서 제외한다.

## 현재 상태 확인

- 기존 서비스: `/robot/cbgl_node/global_localization`
- 기존 scan topic: `/scan`
- 추가 예정 scan topic: `/scan_known`
- 추가 예정 서비스: `/robot/cbgl_node/global_localization_known`
- CBGL 기본 설정: `do_icp: true`, `do_fsm: false`
- 현재 CBGL 실행 흐름은 서비스 호출 직후 즉시 계산하기보다, `received_start_signal_`을 세운 뒤 scan/map/pose cloud 조건이 만족될 때 `processPoseCloud()`가 실행되는 구조다.

## 영향 범위

- 패키지: `cbgl/`
- 파일 후보:
  - `cbgl/include/cbgl_node/cbgl.h`
  - `cbgl/src/cbgl_node/cbgl.cpp`
  - `cbgl/configuration_files/params_cbgl.yaml`
- ROS topic/service/frame:
  - 기존 `/scan` 유지
  - 새 `/scan_known` 구독 추가
  - 기존 `/robot/cbgl_node/global_localization` 유지
  - 새 `/robot/cbgl_node/global_localization_known` 서비스 추가
  - frame 구조는 변경하지 않음. `/scan`과 `/scan_known`은 같은 LiDAR frame metadata를 갖는 것을 전제로 한다.
- Docker/launch/parameter 영향:
  - Docker 설정 변경 없음
  - launch 파일 변경은 가능하면 피하고, topic/service 이름은 YAML param으로 추가한다.

## 구현 설계안

### 핵심 방향

- CBGL 알고리즘, CAER, raycasting, CSM/ICP 코드는 공유한다.
- scan callback 전처리 로직은 공통 함수로 분리한다.
- `/scan`과 `/scan_known`은 각각 최신 scan cache에 저장한다.
- 서비스 호출 시 어떤 scan source를 사용할지 선택한다.

### 권장 구조

- `scan_topic`: 기존 `/scan`
- `known_scan_topic`: 기본값 `/scan_known`
- `global_localisation_service_name`: 기존 `global_localization`
- `global_localisation_known_service_name`: 기본값 `global_localization_known`

내부 상태 후보:

- `latest_raw_scan_`
- `latest_known_scan_`
- `received_raw_scan_`
- `received_known_scan_`
- `requested_scan_source_`

서비스 동작:

- `global_localization`
  - raw scan source를 요청 상태로 설정한다.
  - 기존 `/scan` 기반 흐름으로 CBGL을 실행한다.
- `global_localization_known`
  - known scan source를 요청 상태로 설정한다.
  - `/scan_known` 기반 흐름으로 CBGL을 실행한다.
  - `/scan_known`이 한 번도 들어오지 않은 경우는 실패 또는 warning 반환을 고려한다.

### 중요한 구현 주의점

기존 구조에서는 서비스 호출이 `received_start_signal_`을 세운 뒤, 다음 scan callback에서 `processPoseCloud()`가 실행될 수 있다. 따라서 `global_localization_known` 호출 후 일반 `/scan` callback이 먼저 들어와서 `latest_world_scan_`을 덮어쓰면 안 된다.

이를 막기 위해 pending localization 상태에서는 요청된 source의 callback만 active scan으로 반영하고 `processPoseCloud()`를 호출하도록 한다.

예시:

- known 서비스 호출 후 pending source = known
- `/scan` callback은 raw cache만 갱신하고 active scan/process는 하지 않음
- `/scan_known` callback이 들어오면 active scan을 known으로 설정하고 `processPoseCloud()` 실행

이 방식은 기존 비동기 실행 구조를 유지하면서 scan source 혼선을 막는다.

### 이번 설계에서 제외하는 것

- `/scan_known` age threshold 검사
- `/scan_known`과 `/scan` metadata 자동 비교 실패 처리
- 새 custom srv 파일 생성
- CBGL 알고리즘 scoring 방식 변경
- CAER 평균화 또는 threshold 변경

## 검증 계획

빌드:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

서비스 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice list | grep global_localization'
```

토픽 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo -n 1 /scan'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo -n 1 /scan_known'
```

서비스 호출:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization_known'
```

동작 검증:

- raw 서비스 호출 시 `world_scan`이 `/scan` 기반인지 확인한다.
- known 서비스 호출 시 `world_scan`이 `/scan_known` 기반인지 확인한다.
- known 서비스 호출 직후 `/scan`이 더 빠르게 들어와도 known scan source가 덮이지 않는지 확인한다.
- `/scan_known`이 없을 때 서비스가 조용히 pending 상태로 남지 않고 명확히 실패하거나 warning을 남기는지 확인한다.

## 문서화 계획

- 구현 후 `docs/features/2026-05-07_global-localization-known-service.md`를 작성한다.
- 문서에는 요청, 설계, 변경 파일, 서비스 호출법, 검증 결과, 남은 리스크를 기록한다.

## 승인 상태

- 사용자 승인 문구: "좋다 진행해"
- 승인 시각: 2026-05-07
