# AI 개발 가이드

이 문서는 AI 에이전트가 이 저장소를 조사하거나 수정할 때 먼저 따라야 하는 공통 규칙이다.

**최우선 규칙:** 기능, 설정, 알고리즘, launch, Docker, 문서, 운영 절차를 바꾸는 작업은 구현 전에 현재 프로젝트 기준 설계안을 사용자에게 먼저 보고하고, 명시적 승인을 받은 뒤 시작한다. 승인 전에는 구현 관련 파일 생성, 수정, 삭제를 하지 않는다.

## 프로젝트 맥락

- 이 저장소는 ROS 1 Kinetic 기반 CBGL(global localization) 패키지를 Docker로 실행하고, 호스트 Gazebo/ROS 2 시뮬레이션과 `ros1_bridge`로 연결하는 워크스페이스다.
- 워크스페이스 루트는 `/home/csilab/global_localization`이다.
- 기본 실행 구성은 `docker/`의 Dockerfile과 compose 설정을 사용한다.
- CBGL 노드는 기본 네임스페이스 `robot` 아래에서 실행되며, 주요 서비스는 `/robot/cbgl_node/global_localization`이다.
- 기준 실행 환경은 Docker 컨테이너 안의 ROS 1 Kinetic이다. native ROS 1 설치를 기준으로 한 명령은 보조 정보로만 둔다.

## 패키지 경계

- `cbgl/`: ROS 1 Kinetic CBGL 패키지. 알고리즘, launch, parameter YAML, RangeLib/CSM/FSM 유틸리티를 포함한다.
- `docker/`: CBGL 컨테이너와 ROS 1 bridge 컨테이너 빌드 및 실행 정의.
- `docs/`: 프로젝트 설명, 조사 기록, 개발 메모, 작업 계획, 기능 개발 기록.
- `relocalization_toolbox/`: 보조 relocalization 도구. 명시적 요청 없이는 CBGL 디버깅과 섞어서 수정하지 않는다.
- `sample_map/`: 테스트/샘플 지도 자료. 명시적 요청 없이 알고리즘 변경과 함께 수정하지 않는다.

## 작업 규칙

- 기존 ROS 네임스페이스, 프레임, 토픽, 서비스 구조를 먼저 확인하고 변경한다.
- CBGL 알고리즘 동작을 바꾸는 수정은 작은 단위로 제한한다.
- `cbgl/src/cbgl_node/cbgl.cpp`를 수정할 때는 CAER, ICP/FSM, map raycasting, particle generation, scan preprocessing 변경을 서로 분리한다.
- map origin, frame transform, scan convention 문제는 코드 수정 전 `/map`, `/scan`, `/tf`, `/tf_static`, `/clock`, `map_scan`, `world_scan`을 먼저 확인한다.
- Docker 실행 환경이 기준이다. 빌드/실행/서비스 검증은 가능하면 Docker 컨테이너 기준 명령으로 수행한다.
- 생성물인 `build/`, `install/`, `log/`, catkin build output, cache 파일은 수정 대상으로 삼지 않는다.
- 사용자 또는 이전 디버깅에서 남긴 변경을 되돌리지 않는다. 되돌릴 때는 범위가 명확해야 하며, 사용자 변경과 AI 변경을 구분한다.
- 디버깅용 토픽 publisher와 RViz 시각화 보조 변경은 알고리즘 변경과 구분해서 관리한다.
- `relocalization_toolbox/` 수정은 사용자가 명시적으로 요청했을 때만 수행한다. CBGL 실험 편의를 이유로 함께 수정하지 않는다.
- 새로운 launch, config, script, message/service 파일을 추가할 때는 관련 `CMakeLists.txt`, `package.xml`, launch include, Docker copy/install 경로, 실행 문서를 함께 확인한다.

## 강제 승인 규칙

승인 전에는 아래 행위를 하지 않는다.

- 구현 파일 생성, 수정, 삭제
- ROS launch/YAML/URDF/SDF/world 파일 수정
- Dockerfile, compose, entrypoint, bridge 설정 수정
- `README.md`, `DEVELOPMENT.md`, `AGENTS.md` 같은 운영 문서 수정
- 알고리즘 동작, topic/service/frame, namespace, parameter 기본값 변경
- 테스트 명목으로 소스 파일을 임시 수정하거나 되돌리는 행위

예외는 아래 두 가지뿐이다.

- 분석을 위한 파일 읽기, 검색, diff 확인, ROS/Docker 상태 조회
- 요구사항 분석과 설계안 기록을 위한 `docs/plans/*.md` 생성 또는 갱신

`AGENTS.md` 자체를 수정하라는 사용자의 명시적 요청은 그 파일 변경에 대한 승인으로 간주한다. 이 경우에도 다른 구현 파일 변경으로 범위를 확장하지 않는다.

사용자가 `진행해`, `승인`, `구현 시작`, `좋아 반영해`, `그렇게 수정해`처럼 명시적으로 허락한 뒤에만 구현을 시작한다. `가능해?`, `어떻게 할까?`, `설명해줘`, `확인해봐`는 구현 승인으로 해석하지 않는다.

## 서브에이전트 운영 모델

작업은 아래 역할을 기준으로 나누어 수행한다. 실제 도구가 독립 서브에이전트 실행을 지원하지 않거나 사용자가 명시적으로 요청하지 않은 경우, 한 에이전트가 이 역할을 순서대로 적용한다.

### 1. 요구사항 분석가

역할:

- 사용자 요청을 목표, 범위, 제약 조건으로 정리한다.
- 영향 받을 파일, ROS 토픽/서비스/프레임, Docker 구성, launch/parameter YAML을 식별한다.
- 코드 수정이 필요한지, 설정/실행 문제인지, 관측이 먼저 필요한지 구분한다.
- 구현 전 확인이 필요한 애매한 점과 검증 기준을 정리한다.

산출물:

- 요구사항 요약
- 작업 범위
- 영향 파일 후보
- 확인할 topic/service/frame
- 검증 기준

### 2. ROS 디버거

역할:

- `/map`, `/scan`, `/tf`, `/tf_static`, `/clock`, CBGL 디버깅 토픽을 확인한다.
- ROS 1 bridge QoS, frame id, sim time, namespace 문제를 분리한다.
- `map_scan`과 `world_scan`의 range 분포를 비교해 raycasting 문제와 scan matching 문제를 구분한다.
- map origin, y-axis flip, OMap 생성, scan angle convention 의심은 코드 수정 전 독립 실험으로 분리한다.

산출물:

- 관측 명령
- 핵심 출력 요약
- 원인 후보
- 다음 실험

### 3. 개발자

역할:

- 승인된 범위만 작은 단위로 구현한다.
- 알고리즘 수정, 디버깅 publisher 수정, Docker/launch/parameter 수정은 별도 변경으로 유지한다.
- 기존 코드 스타일과 ROS 1 Kinetic/C++11 호환성을 유지한다.
- 새 service/message/script/config가 생기면 `CMakeLists.txt`, `package.xml`, Docker/launch 적용 경로를 함께 확인한다.

산출물:

- 구현 요약
- 변경 파일 목록
- 새 의존성 또는 실행 명령 변경 사항

### 4. 리팩터러

역할:

- 동작 변경 없이 구조 정리, 중복 제거, 네이밍 개선을 수행한다.
- 리팩터링이 기능 구현과 분리 가능한 경우 작은 단위로 제한한다.
- 외부/보조 패키지 성격의 `relocalization_toolbox/`는 명시적 요청 없이는 리팩터링하지 않는다.

산출물:

- 리팩터링 요약
- 동작 보존 근거
- 리스크가 있는 변경 여부

### 5. 테스터

역할:

- 변경 범위에 맞는 catkin build, Docker 재시작, ROS topic/service 검증을 수행한다.
- 테스트가 불가능하거나 실패한 경우 실패 원인과 남은 리스크를 기록한다.
- bridge 구성이 바뀌는 경우에만 `ros1_bridge` 재시작을 고려한다.

산출물:

- 실행한 검증 명령
- 성공/실패 결과
- 실패 시 원인과 후속 조치

### 6. 문서 작성자

역할:

- 기능 개발마다 `docs/features/`에 기능 개발 내용과 결과를 기록한다.
- 승인 전 설계안은 `docs/plans/`에 기록한다.
- 필요하면 `README.md`, `DEVELOPMENT.md`, `AGENTS.md`도 함께 갱신하되, 운영 문서 수정은 승인 후에만 수행한다.
- 문서는 실제 변경 사항과 검증 결과를 기준으로 작성한다.

산출물:

- `docs/plans/YYYY-MM-DD_HHMM_feature-name-plan.md`
- `docs/features/YYYY-MM-DD_HHMM_feature-name.md`
- 사용자 요청, 요구사항, 구현 내용, 검증 결과, 후속 작업 기록

## 기능 개발 흐름

기능 개발은 기본적으로 아래 순서를 따른다. 승인 전에는 구현 관련 파일 생성, 수정, 삭제를 하지 않는다. 단순 질의응답, 코드 변경 없는 조사, 설명, 코드 변경 없는 리뷰는 승인 게이트 대상에서 제외할 수 있다.

1. 요구사항 분석가: 요청 분석과 검증 기준 정의
2. ROS 디버거: 필요한 경우 현재 topic/frame/service 상태 확인
3. Design Proposal: 현재 프로젝트 구조 기준 설계안 보고 및 `docs/plans/` 기록
4. Approval Gate: 사용자 명시적 승인 대기
5. 개발자: 승인 후 실제 기능 구현
6. 리팩터러: 필요한 경우 동작 보존 리팩터링
7. 테스터: 빌드, 테스트, ROS/Docker 검증
8. 문서 작성자: `docs/features/`에 개발 내용과 결과 기록

기능 개발이 완료되려면 `docs/features/`에 해당 기능 문서가 추가되거나 갱신되어야 한다. 단순 질의응답, 일반 설명, 짧은 조사, 코드 변경 없는 리뷰, 일상적인 채팅은 기능 문서 작성 대상에서 제외한다.

## Approval Gate

구현 전에 사용자에게 아래 내용을 보고한다.

- 목표
- 변경 범위
- 영향 파일
- 구현 방식
- 검증 방법
- `docs/features/` 기록 계획

사용자가 명시적으로 승인하기 전에는 구현을 시작하지 않는다.

승인 전 파일 변경의 예외로, 요구사항 분석, 설계안, 구현 계획 기록을 위한 `docs/plans/*.md` 생성 또는 갱신은 허용한다. 이 예외는 구현 파일, ROS 설정 파일, launch 파일, Docker 설정 파일, `README.md`, `DEVELOPMENT.md`, `AGENTS.md` 같은 운영 문서에는 적용하지 않는다.

작업 계획서는 아래 템플릿을 기준으로 작성한다.

```text
docs/templates/task-plan.md
```

파일 이름은 날짜, 시분, 기능명을 포함한다. 시각은 로컬 작업 환경 기준 24시간 `HHMM` 형식을 사용한다.

```text
docs/plans/YYYY-MM-DD_HHMM_feature-name-plan.md
docs/features/YYYY-MM-DD_HHMM_feature-name.md
```

기존 날짜-only 문서는 링크 안정성을 위해 유지하고, 새 문서부터 timestamp 파일명 규칙을 적용한다.

## CBGL 알고리즘 변경 특별 규칙

CBGL 알고리즘 또는 map/raycasting 동작 변경은 특히 아래 규칙을 따른다.

- `map_scan`이 비정상인 동안 CAER/ICP/FSM scoring을 먼저 수정하지 않는다.
- origin, y-axis flip, OMap 생성, scan angle convention은 독립 실험으로 분리한다.
- wall/clearance 필터는 충돌 가능 pose 제거 목적일 때만 고려하고, 벽 자체를 feature에서 제거하지 않는다.
- scan preprocessing 변경은 CAER, CSM/FSM, debug publisher에 미치는 영향을 따로 기록한다.
- 결과가 나빠진 수정은 원인 분석 없이 누적하지 않는다.
- 알고리즘 변경은 승인된 계획에 없는 추가 개선으로 확장하지 않는다.

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

topic/frame 확인이 필요하면 Docker 기준으로 확인한다.

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic list'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo -n 1 /scan'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo -n 1 /map'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rosrun tf tf_echo map base_footprint'
```

## 완료 체크리스트

- 구현 전에 설계안을 보고하고 사용자 승인을 받았는가?
- 구현 전 작업 계획서를 `docs/plans/`에 기록했는가?
- 변경 범위가 요청과 직접 관련되어 있는가?
- 승인된 범위 밖의 파일을 만들거나 수정하지 않았는가?
- 알고리즘 변경과 디버깅/시각화 변경을 구분했는가?
- `/map`, `/scan`, `/tf`, `/tf_static`, `/clock` 관련 가정을 확인했는가?
- 새 파일/의존성이 있다면 `CMakeLists.txt`, `package.xml`, Docker/launch 적용 경로가 맞는가?
- Docker 기준 빌드 또는 실행 검증을 수행했는가?
- 실행하지 못한 검증이 있다면 이유와 남은 리스크를 설명했는가?
- 의미 있는 기능 구현 또는 프로세스 변경인 경우 `docs/features/`에 개발 내용과 결과를 기록했는가?
- 사용자가 유지하라고 한 변경을 되돌리지 않았는가?
