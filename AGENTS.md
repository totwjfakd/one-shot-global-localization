# AI 개발 가이드

이 문서는 AI 에이전트가 이 저장소를 수정하거나 조사할 때 먼저 따라야 하는 공통 규칙이다.

## 프로젝트 맥락

- 이 저장소는 ROS 1 Kinetic 기반 CBGL(global localization) 패키지를 Docker로 실행하고, 호스트 Gazebo/ROS 2 시뮬레이션과 `ros1_bridge`로 연결하는 워크스페이스다.
- 워크스페이스 루트는 `/home/csilab/global_localization`이다.
- 기본 실행 구성은 `docker/`의 Dockerfile과 compose 설정을 사용한다.
- CBGL 노드는 기본 네임스페이스 `robot` 아래에서 실행되며, 주요 서비스는 `/robot/cbgl_node/global_localization`이다.

## 패키지 경계

- `cbgl/`: ROS 1 Kinetic CBGL 패키지. 알고리즘, launch, parameter YAML, RangeLib/CSM/FSM 유틸리티를 포함한다.
- `docker/`: CBGL 컨테이너와 ROS 1 bridge 컨테이너 빌드 및 실행 정의.
- `docs/`: 프로젝트 설명, 조사 기록, 개발 메모.
- `relocalization_toolbox/`: 보조 relocalization 도구. 명시적 요청 없이는 CBGL 디버깅과 섞어서 수정하지 않는다.

## 작업 규칙

- 기존 ROS 네임스페이스, 프레임, 토픽 구조를 먼저 확인하고 변경한다.
- CBGL 알고리즘 동작을 바꾸는 수정은 작은 단위로 제한하고, 수정 전 원인 가설과 검증 방법을 사용자에게 먼저 설명한다.
- `cbgl/src/cbgl_node/cbgl.cpp`를 수정할 때는 CAER, ICP/FSM, map raycasting, particle generation 변경을 서로 분리한다.
- map origin, frame transform, scan convention 문제는 코드 수정 전 `/map`, `/scan`, `/tf`, `/clock`, `map_scan`, `world_scan`을 먼저 확인한다.
- Docker 실행 환경이 기준이다. native ROS 1 설치를 가정한 명령은 보조 정보로만 둔다.
- 생성물인 `build/`, `install/`, `log/`, catkin build output은 수정 대상으로 삼지 않는다.
- 사용자 또는 이전 디버깅에서 남긴 변경을 되돌리지 않는다. 되돌릴 때는 범위가 명확해야 한다.
- 디버깅용 토픽 publisher와 RViz 시각화 보조 변경은 알고리즘 변경과 구분해서 관리한다.

## 서브에이전트 운영 모델

작업은 아래 역할을 기준으로 나누어 수행한다. 실제 도구가 독립 서브에이전트 실행을 지원하지 않거나 사용자가 명시적으로 요청하지 않은 경우, 한 에이전트가 이 역할을 순서대로 적용한다.

### 1. 요구사항 분석가

역할:

- 사용자 요청을 목표, 범위, 제약 조건으로 정리한다.
- 영향 받을 파일, ROS 토픽/서비스/프레임, Docker 구성, launch/parameter YAML을 식별한다.
- 코드 수정이 필요한지, 설정/실행 문제인지, 관측이 먼저 필요한지 구분한다.

산출물:

- 요구사항 요약
- 영향 범위
- 확인할 topic/service/frame
- 검증 기준

### 2. ROS 디버거

역할:

- `/map`, `/scan`, `/tf`, `/tf_static`, `/clock`, CBGL 디버깅 토픽을 확인한다.
- ROS 1 bridge QoS, frame id, sim time, namespace 문제를 분리한다.
- `map_scan`과 `world_scan`의 range 분포를 비교해 raycasting 문제와 scan matching 문제를 구분한다.

산출물:

- 관측 명령
- 핵심 출력 요약
- 원인 후보와 다음 실험

### 3. 개발자

역할:

- 승인되거나 명시적으로 요청된 수정만 작은 단위로 구현한다.
- 알고리즘 수정, 디버깅 publisher 수정, Docker/launch/parameter 수정은 별도 변경으로 유지한다.
- 기존 코드 스타일과 ROS 1 Kinetic/C++11 호환성을 유지한다.

산출물:

- 구현 요약
- 변경 파일 목록
- 새 실행/빌드 요구사항

### 4. 테스터

역할:

- 변경 범위에 맞는 catkin build, Docker 재시작, ROS topic/service 검증을 수행한다.
- 테스트가 불가능하면 이유와 남은 리스크를 명확히 기록한다.

산출물:

- 실행한 검증 명령
- 성공/실패 결과
- 실패 시 원인과 후속 조치

### 5. 문서 작성자

역할:

- 중요한 디버깅 결론, 실행 절차, 파라미터 의미를 문서화한다.
- 반복되는 실험 명령은 `DEVELOPMENT.md`나 `docs/`에 정리한다.
- 실제 변경과 검증 결과를 기준으로 기록한다.

산출물:

- 문서 변경 요약
- 남은 이슈
- 재현/검증 명령

## 에이전트 실행 흐름

일반 작업은 아래 순서를 따른다.

1. 요구사항 분석가: 요청과 제약 정리
2. ROS 디버거: 현재 topic/frame/service 상태 확인
3. 설계 메모: 필요한 경우 수정 전 가설과 검증 방법 보고
4. 개발자: 명시된 범위만 수정
5. 테스터: 빌드와 최소 실행 검증
6. 문서 작성자: 필요한 운영 문서 갱신

CBGL 알고리즘 또는 map/raycasting 동작 변경은 특히 아래 규칙을 따른다.

- `map_scan`이 비정상인 동안 CAER/ICP scoring을 먼저 수정하지 않는다.
- origin, y-axis flip, OMap 생성, scan angle convention은 독립 실험으로 분리한다.
- wall/clearance 필터는 충돌 가능 pose 제거 목적일 때만 고려하고, 벽 자체를 feature에서 제거하지 않는다.
- 결과가 나빠진 수정은 원인 분석 없이 누적하지 않는다.

## 검증

가능하면 변경 후 아래 명령을 사용한다.

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

새 바이너리를 적용해야 하면 CBGL 컨테이너만 재시작한다.

```bash
docker restart cbgl
```

bridge 구성이 바뀌거나 ROS 2 topic bridge 상태를 다시 잡아야 할 때만 bridge를 재시작한다.

```bash
docker restart ros1_bridge
```

서비스 호출은 namespace를 포함한다.

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
```

## 완료 체크리스트

- 요청 범위와 직접 관련된 파일만 수정했는가?
- 알고리즘 변경과 디버깅/시각화 변경을 구분했는가?
- `/map`, `/scan`, `/tf`, `/clock` 관련 가정을 확인했는가?
- Docker 기준 빌드 또는 실행 검증을 수행했는가?
- 실행하지 못한 검증이 있다면 이유를 설명했는가?
- 사용자가 유지하라고 한 변경을 되돌리지 않았는가?
