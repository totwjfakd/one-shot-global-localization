# Relocalization Toolbox 컨테이너 구현 명세

이 문서는 `relocalization_toolbox`를 CBGL과 같은 Docker 기반 비교 실험 대상으로 실행하기 위한 구현 명세다.

목표는 CBGL이 약한 조건, 특히 non-panoramic 2D LiDAR FoV, 대칭 환경, sparse feature 환경에서 `relocalization_toolbox`와 결과를 비교할 수 있게 만드는 것이다.

## 대상 패키지

소스 위치:

```text
relocalization_toolbox/relocalization_toolbox/relocalization_toolbox
relocalization_toolbox/relocalization_toolbox/relocalization_toolbox_msgs
```

ROS 패키지:

```text
relocalization_toolbox
relocalization_toolbox_msgs
```

기준 환경:

```text
Ubuntu 20.04
ROS 1 Noetic
catkin
C++17
```

CBGL은 ROS Kinetic 기반이므로 같은 컨테이너에 합치지 않는다. `relocalization_toolbox`는 별도 Noetic 컨테이너로 구성한다.

## 실행 목표

컨테이너가 실행되면 다음 노드가 뜬다.

```text
/relocalization_2d_handler
```

기본 입력 topic:

```text
/map    nav_msgs/OccupancyGrid
/scan   sensor_msgs/LaserScan
/tf
/tf_static
/clock
```

출력 및 디버깅 topic:

```text
/initialpose
/relocalization_candidates
/relocalization_anchor_points
/relocalization_scan_cloud
/relocalization_map_cloud
```

서비스:

```text
/relocalization_simple_request
/relocalization_request
/get_relocalization_candidates_request
```

우선 사용할 서비스는 `/relocalization_simple_request`이다. 이 서비스는 request body 없이 `/map`, `/scan` topic에서 데이터를 받아 relocalization을 수행한다.

## Frame 및 Topic 정책

현재 Gazebo/bridge 환경 기준 frame은 다음으로 고정한다.

```yaml
map_frame: "map"
base_frame: "base_footprint"
lidar_frame: "lidar_link"
```

`base_frame`은 upstream 기본값인 `base_link`가 아니라 현재 프로젝트에서 사용하는 `base_footprint`로 변경한다.

map은 파일에서 직접 읽지 않고 topic을 사용한다.

```yaml
prefetch_map_from_topic: true
```

`relocalization_simple_request`는 내부적으로 다음 모드를 사용한다.

```text
map_mode  = MODE_FROM_TOPIC
scan_mode = MODE_FROM_TOPIC
```

따라서 초기 구현에서는 `map_path`, `map_name`을 사용하지 않는다. `/map` topic은 ROS 2 host sim에서 `ros1_bridge`를 통해 ROS 1 쪽으로 들어오는 `nav_msgs/OccupancyGrid`를 그대로 사용한다.

## Config 변경 사항

수정 대상:

```text
relocalization_toolbox/relocalization_toolbox/relocalization_toolbox/config/passive_fast_2d.yaml
relocalization_toolbox/relocalization_toolbox/relocalization_toolbox/config/passive_precise_2d.yaml
```

필수 변경:

```yaml
base_frame: "base_footprint"
prefetch_map_from_topic: true
```

현재 확인 기준으로 `prefetch_map_from_topic`은 이미 `true`이며, `base_frame`만 `base_footprint`로 맞춘다.

초기 실험은 `passive_fast_2d.yaml`을 기준으로 한다. `passive_precise_2d.yaml`은 성능 비교 또는 실패 케이스 재검증용으로 둔다.

## Docker 구성

추가할 파일:

```text
docker/Dockerfile.relocalization
```

권장 base image:

```dockerfile
osrf/ros:noetic-desktop
```

필요 apt 패키지:

```text
build-essential
cmake
git
python3-catkin-tools
python3-rosdep
ros-noetic-pcl-ros
ros-noetic-cv-bridge
ros-noetic-tf
ros-noetic-tf2
ros-noetic-tf2-ros
ros-noetic-tf2-sensor-msgs
ros-noetic-tf2-geometry-msgs
ros-noetic-octomap
ros-noetic-octomap-msgs
libyaml-cpp-dev
libopencv-dev
libpcl-dev
```

컨테이너 내부 workspace:

```text
/home/relocalization/catkin_ws
```

마운트:

```text
../relocalization_toolbox/relocalization_toolbox/relocalization_toolbox
  -> /home/relocalization/catkin_ws/src/relocalization_toolbox

../relocalization_toolbox/relocalization_toolbox/relocalization_toolbox_msgs
  -> /home/relocalization/catkin_ws/src/relocalization_toolbox_msgs
```

빌드:

```bash
source /opt/ros/noetic/setup.bash
cd /home/relocalization/catkin_ws
catkin build relocalization_toolbox_msgs relocalization_toolbox
```

entrypoint:

```bash
source /opt/ros/noetic/setup.bash
source /home/relocalization/catkin_ws/devel/setup.bash
roslaunch relocalization_toolbox passive_fast_2d.launch
```

## Docker Compose 서비스

`docker/docker-compose.yml`에 다음 서비스를 추가한다.

```yaml
  relocalization_toolbox:
    build:
      context: ..
      dockerfile: docker/Dockerfile.relocalization

    image: relocalization-toolbox:local
    container_name: relocalization_toolbox

    stdin_open: true
    tty: true
    network_mode: "host"
    ipc: host
    user: relocalization
    environment:
      - ROS_MASTER_URI=http://127.0.0.1:11311
      - ROS_HOSTNAME=127.0.0.1
      - ROS_IP=127.0.0.1
    volumes:
      - ../relocalization_toolbox/relocalization_toolbox/relocalization_toolbox:/home/relocalization/catkin_ws/src/relocalization_toolbox:rw
      - ../relocalization_toolbox/relocalization_toolbox/relocalization_toolbox_msgs:/home/relocalization/catkin_ws/src/relocalization_toolbox_msgs:rw
    depends_on:
      - ros1_bridge
```

주의: `depends_on`은 컨테이너 시작 순서만 보장한다. `/map`, `/scan`, `/tf_static` discovery는 별도 확인이 필요하다.

ROS 1 master 정책:

- `ros1_bridge` 컨테이너만 `roscore`를 시작한다.
- `cbgl`과 `relocalization_toolbox`는 `ROS_MASTER_URI=http://127.0.0.1:11311`에 붙는다.
- CBGL/relocalization entrypoint는 `rostopic list`가 성공할 때까지 기다린 뒤 `roslaunch`를 실행한다.
- 이렇게 해야 세 컨테이너가 같은 `/run_id`와 같은 parameter server를 공유하고, `port 11311 is already in use` 충돌을 피할 수 있다.

## 실행 순서

권장 순서:

```bash
cd /home/csilab/global_localization/docker
docker compose up --build cbgl ros1_bridge relocalization_toolbox
```

현재 개발 PC처럼 Docker Compose v2 플러그인이 없고 legacy `docker-compose`만 있는 환경에서는 아래 명령을 사용한다.

```bash
cd /home/csilab/global_localization/docker
docker-compose up --build cbgl ros1_bridge relocalization_toolbox
```

CBGL 없이 relocalization toolbox만 볼 때:

```bash
docker compose up --build ros1_bridge relocalization_toolbox
```

legacy compose 환경:

```bash
docker-compose up --build ros1_bridge relocalization_toolbox
```

ROS topic discovery가 꼬이면 bridge를 재시작한다.

```bash
docker restart ros1_bridge
docker restart relocalization_toolbox
```

## 검증 명령

노드 확인:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; source /home/relocalization/catkin_ws/devel/setup.bash; rosnode list'
```

topic 확인:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; rostopic info /map'
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; rostopic info /scan'
```

TF 확인:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; rosrun tf tf_echo lidar_link base_footprint'
```

서비스 확인:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; source /home/relocalization/catkin_ws/devel/setup.bash; rosservice list | grep relocalization'
```

실행:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; source /home/relocalization/catkin_ws/devel/setup.bash; rosservice call /relocalization_simple_request'
```

후보 여러 개 확인:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; source /home/relocalization/catkin_ws/devel/setup.bash; rosservice call /get_relocalization_candidates_request "{map_mode: 0, scan_mode: 0, max_num_of_candidates: 20}"'
```

## 비교 실험 계획

CBGL과 비교할 항목:

1. 360도 LiDAR
   - CBGL이 정상 수렴하는 baseline
   - relocalization toolbox도 같은 pose로 수렴하는지 확인

2. 180도 LiDAR
   - CBGL의 CAER 후보 압축이 얼마나 불안정해지는지 확인
   - relocalization toolbox의 non-panoramic sector handling이 유효한지 확인

3. 복도/긴 벽/대칭 구조
   - top candidate가 여러 개로 갈라지는지 확인
   - `/get_relocalization_candidates_request`로 후보 다중성을 비교

4. no-return ray가 많은 scan
   - CBGL은 내부 `world_scan`에서 `inf`를 `range_max`로 정규화해야 함
   - relocalization toolbox는 finite beam 위주로 계산하므로 valid beam 수와 confidence score를 확인

5. sparse feature 구역
   - CBGL top-k와 relocalization candidates의 정답 포함률 비교

## 예상 리스크

1. TF 방향
   - handler는 시작 시 `lookupTransform(lidar_frame, base_frame)`을 호출한다.
   - 현재 환경에서 `lidar_link -> base_footprint` transform이 조회되어야 한다.

2. ROS1 bridge discovery
   - `/map`, `/scan`, `/tf_static` publisher가 Noetic 컨테이너에서 보이는지 확인해야 한다.

3. message/service bridge
   - `relocalization_toolbox_msgs`는 custom service라 ROS 2 쪽으로 bridge하지 않는 것이 초기 목표다.
   - service call은 ROS 1 컨테이너 내부에서 수행한다.

4. map threshold 해석
   - 초기 구현은 `/map` topic을 사용하므로 map file loader의 PGM threshold 차이는 피한다.
   - file mode를 사용할 때만 `map_io_2d.cpp`의 threshold 해석을 별도 검증한다.

5. 실행 시간
   - RRT sampling, SMAD, GICP가 map 크기와 `map_sampling_ratio`, `anchor_point_min_dist`에 민감하다.
   - 처음에는 `passive_fast_2d.yaml`로 시작하고, 실패 케이스에서 precise config를 비교한다.

## 완료 기준

1. Noetic 컨테이너가 빌드된다.
2. `/relocalization_2d_handler`가 실행된다.
3. `/map`, `/scan`, `lidar_link -> base_footprint` TF를 모두 확인한다.
4. `/relocalization_simple_request`가 응답한다.
5. `/initialpose` 또는 service response pose가 map frame에서 반환된다.
6. `/get_relocalization_candidates_request`로 후보 여러 개를 받을 수 있다.
7. CBGL과 같은 정답 pose 기준으로 오차를 비교할 수 있다.
