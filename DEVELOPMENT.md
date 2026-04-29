# 개발 문서

이 문서는 사람과 AI 에이전트가 공통으로 사용하는 개발, 빌드, 실행, 디버깅 절차를 정리한다.

## 환경

- 호스트: Gazebo/ROS 2 시뮬레이션
- CBGL 컨테이너: Ubuntu 16.04, ROS 1 Kinetic, catkin
- 브리지 컨테이너: ROS 1 bridge
- 워크스페이스 루트:

```bash
cd /home/csilab/global_localization
```

## 저장소 구조

- `cbgl/`: ROS 1 CBGL 패키지. `cbgl` 컨테이너 안에서 `/home/cbgl/catkin_ws/src/cbgl/`로 마운트된다.
- `docker/`: CBGL 및 bridge Docker 정의.
- `docs/`: 프로젝트 메모와 디버깅 기록.
- `relocalization_toolbox/`: 보조 relocalization 도구.

## Docker 빌드 및 실행

`docker/`의 Docker 구성을 사용한다.

```bash
cd /home/csilab/global_localization/docker
docker compose up --build
```

로컬 환경이 legacy compose 명령을 사용한다면:

```bash
docker-compose up --build
```

컨테이너가 이미 있고 CBGL 바이너리만 바뀐 경우:

```bash
docker restart cbgl
```

bridge 설정이 바뀌었거나 topic discovery를 다시 잡아야 할 때만 bridge를 재시작한다:

```bash
docker restart ros1_bridge
```

## CBGL 빌드

ROS 1 컨테이너 안에서 빌드한다:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

수동 명령을 실행할 때는 워크스페이스를 source 한다:

```bash
docker exec -it cbgl bash
source /opt/ros/kinetic/setup.bash
source /home/cbgl/catkin_ws/devel/setup.bash
```

## 실행 및 서비스 호출

launch 파일은 노드를 `robot` namespace 아래에서 실행한다.

```bash
roslaunch cbgl cbgl.launch
```

서비스 이름은 다음과 같다:

```bash
/robot/cbgl_node/global_localization
```

컨테이너 안에서 다음처럼 호출한다:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
```

## 주요 토픽과 프레임

입력:

- `/map`: `nav_msgs/OccupancyGrid`
- `/scan`: `sensor_msgs/LaserScan`
- `/tf`, `/tf_static`
- `use_sim_time`이 true일 때 `/clock`

CBGL 출력 및 디버깅 토픽:

- `/initialpose`
- `/robot/cbgl_node/particlecloud_all`
- `/robot/cbgl_node/particlecloud_top_k_caers`
- `/robot/cbgl_node/particlecloud_all_caers`
- `/robot/cbgl_node/world_scan`
- `/robot/cbgl_node/map_scan`

기본 프레임:

- fixed frame: `map`
- base frame: `base_footprint`
- lidar frame: `/scan.header.frame_id`에서 오며, 보통 `lidar_link`

## 디버깅 흐름

알고리즘 코드를 바꾸기 전에 데이터 경로를 먼저 확인한다.

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic list | sort'
```

서비스 존재 여부 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice list | grep global_localization'
```

scan 형태 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo -n 1 /scan'
```

map metadata 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo -n 1 /map/info'
```

base에서 lidar까지의 TF 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rosrun tf tf_echo base_footprint lidar_link'
```

CBGL 파라미터 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rosparam get /robot/cbgl_node'
```

## Map Scan 진단

- CAER나 ICP를 바꾸기 전에 `map_scan`과 `world_scan`을 먼저 비교한다.
- `world_scan`은 CBGL 전처리 이후의 실제/latest 입력 scan에 가깝다.
- `map_scan`은 hypothesis pose에서 occupancy map을 향해 raycast한 가상 scan이다.
- `map_scan`이 전부 `20.0`이면 raycaster가 장애물을 보지 못하고 있다는 뜻이다.
- `map_scan`이 전부 `0.0`이면 raycaster가 시작점을 occupied cell 안으로 보고 있을 가능성이 높다.
- `map_scan`에 그럴듯한 finite range가 많지만 ICP가 실패하면 그 다음에 CAER/ICP 파라미터나 scan convention을 의심한다.

origin이 0이 아닌 map은, CBGL 전용 shifted map을 의도적으로 테스트하는 경우가 아니라면 원본 YAML을 source of truth로 둔다.

## 파라미터 메모

주요 파일:

```text
cbgl/configuration_files/params_cbgl.yaml
```

자주 보는 파라미터:

- `map_scan_method`: `vanilla`, `ray_marching`, `bresenham`, `cddt` 중 하나
- `laser_z_orientation`: `upwards` 또는 `downwards`
- `do_undersample_scan`
- `undersample_rate`
- `top_k_caers`
- `tf_broadcast`

파라미터를 바꾸면 CBGL 노드/컨테이너를 재시작해야 한다.

```bash
docker restart cbgl
```

## 에이전트 실행 지침

AI 에이전트와 작업할 때는 `AGENTS.md`의 역할을 사용한다.

1. 요구사항 분석가: 질문과 영향 파일/topic을 정의한다.
2. ROS 디버거: topic, TF, map, scan, log 근거를 수집한다.
3. 개발자: 합의된 파일과 동작만 수정한다.
4. 테스터: 의미 있는 최소 빌드 또는 런타임 검증을 실행한다.
5. 문서 작성자: 반복 가능한 절차와 결과를 기록한다.

CBGL localization 실패는 아래 순서로 본다.

1. bridge와 topic을 확인한다.
2. `/clock`과 sim time을 확인한다.
3. `base_footprint`에서 scan frame까지의 TF를 확인한다.
4. `/scan` angle convention과 frame을 확인한다.
5. `/map` origin/resolution/dimensions를 확인한다.
6. `world_scan`과 `map_scan`을 비교한다.
7. 그 다음에만 CAER, ICP, particle generation 변경을 고려한다.

## 저장소 관리

- 생성된 `build/`, `install/`, `log/`, catkin artifact는 커밋하지 않는다.
- Docker/environment 변경과 CBGL 알고리즘 변경을 분리한다.
- debug publisher 변경과 동작 변경을 분리한다.
- 작업 대상이 명시되지 않은 외부/보조 패키지는 수정하지 않는다.
- bridge timing, missing TF, sim time 때문에 실패한 명령은 기록해 다음 조사에서 환경 문제와 알고리즘 문제를 구분할 수 있게 한다.
