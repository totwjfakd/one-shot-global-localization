# CBGL Global Localization 디버깅 정리

이 문서는 Docker 기반 CBGL 실행 중 `/robot/cbgl_node/global_localization` 호출이 정답 pose로 수렴하지 않던 문제를 추적한 기록이다.

## 상황

실행 환경:

```text
Host: Gazebo / ROS 2 simulation
CBGL container: ROS 1 Kinetic
Bridge container: ros1_bridge
서비스: /robot/cbgl_node/global_localization
입력 topic: /map, /scan, /tf, /tf_static, /clock
```

사용한 샘플 map:

```yaml
image: v2.pgm
mode: trinary
resolution: 0.05
origin: [-15, -24.1, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

`v2.pgm`은 599 x 982 pixel map이므로 map frame의 대략적인 범위는 다음과 같다.

```text
x: -15.0 ~ 14.95
y: -24.1 ~ 25.0
```

검증 시 정답 pose는 다음으로 두었다.

```text
x = 5.0
y = 5.0
theta = 0.0
```

## 관측된 증상

초기에는 다음 문제가 섞여 있었다.

- `map_scan`이 전부 `20.0`으로 나옴
- origin 보정을 잘못 넣으면 `map_scan`이 전부 `0.0`으로 나옴
- `particlecloud_all`에는 정답 근처 particle이 있는데 `particlecloud_top_k_caers`에는 정답 근처 후보가 없음
- ICP 단계에서 `before trimming, only 0 correspondences` 또는 매우 적은 correspondence가 나옴
- 최종 pose가 벽 근처나 map의 엉뚱한 위치로 튐

즉 단일 문제가 아니라 map raycasting, CAER yaw slicing, LaserScan invalid range 처리가 동시에 영향을 주고 있었다.

## 원인 1: map origin과 RangeLib grid 좌표계 불일치

ROS `map.yaml`의 `origin`은 이미지 좌하단 pixel의 world pose이다.

CBGL의 `scanMap()` / `scanMapPanoramic()`은 후보 pose의 world 좌표를 RangeLib image grid 좌표로 바꿔 raycasting한다. 이때 map origin을 빼지 않으면 음수 origin을 가진 map에서 raycaster가 전혀 다른 위치를 쏘게 된다.

올바른 변환은 개념적으로 다음과 같다.

```cpp
grid_x = (world_x - origin_x) / resolution;
grid_y = height - 1 - ((world_y - origin_y) / resolution);
```

또 ROS `OccupancyGrid` row order와 RangeLib image y축 방향이 반대이므로 OMap을 만들 때 y축 flip이 필요하다.

### 증거

origin 보정 전후로 `map_scan` 분포가 크게 바뀌었다.

```text
이상 상태 1: ranges가 거의 모두 20.0
이상 상태 2: ranges가 거의 모두 0.0
정상 상태: min=1.598, median=3.247, mean=4.698, max=20.000
```

## 원인 2: panoramic CAER 회전 방향 불일치

CBGL은 한 위치 후보마다 `da`개의 yaw 후보를 만든다. 구현은 먼저 panoramic map scan을 만든 뒤, 배열을 rotate해서 각 yaw 후보의 가상 scan slice를 만든다.

문제는 기존 rotate 방향과 yaw 갱신식이 서로 반대 convention을 보고 있었다는 점이다.

관련 로직:

```cpp
new_yaw_i = yaw_i - 2 * M_PI * k / da_;
```

이 yaw 갱신식을 유지한다면, panoramic range 배열은 반대 방향으로 rotate되어야 `scanMap(pose_ik)`가 직접 만든 scan과 같은 의미가 된다.

### 증거

정답 근처 particle에서 직접 raycast CAER와 panoramic rotate CAER를 비교했다.

```text
nearest init particle: pose=(5.162, 5.160, -1.234), dist=0.228

best by direct:
k=26 yaw=-0.056 direct=440.126 pano_left=1966.103 pano_right=510.180

best by current pano left rotate:
k=06 yaw=-2.412 direct=2138.198 pano_left=505.037

best by alternate right rotate:
k=26 yaw=-0.056 direct=440.126 pano_right=510.180
```

즉 현재 yaw에 대응되는 직접 map scan은 낮은 CAER를 보이는데, 기존 panoramic left rotate는 다른 yaw를 좋게 보고 있었다.

## 원인 3: LaserScan no-return 값이 CAER를 망가뜨림

ROS LaserScan에서 장애물을 못 본 ray는 `inf`로 들어올 수 있다. 실제 입력 `/scan`에서 다음이 관측됐다.

```text
range_min = 0.08
range_max = 20.0
n = 1440
inf = 211
```

CBGL 내부의 `caer()`는 finite 체크를 하지 않고 다음 조건만 본다.

```cpp
if (sr[i] > 0.0 && sv[i] > 0.0)
  c += fabs(sr[i] - sv[i]);
```

따라서 내부 `world_scan`에 `inf`가 남으면 CAER가 `inf`가 될 수 있다. 반대로 invalid range를 `0.0`으로 만들면 해당 ray가 CAER에서 제외되어, 일부 잘못된 후보가 낮은 누적 error를 얻는 편향이 생길 수 있다.

이 프로젝트에서는 no-return을 `range_max`로 정규화하는 것이 맞다. 실제 scan과 map scan 모두 "최대거리까지 장애물을 못 봄"이라는 같은 의미를 갖기 때문이다.

### 증거

수정 전:

```text
world_scan n=360 inf=54 nan=0 zero=0 finite_min=3.136 finite_max=19.913
top count=100 near1=0 near2=0 near3=0
```

수정 후:

```text
world_scan n=360 inf=0 nan=0 zero=0 finite_min=3.150 finite_max=20.000
top count=100 near1=22 near2=44 near3=48 nearest=0.228
result pose=(5.003, 5.104, -0.001), dist=0.104
```

## 최종 수정 요약

수정 범위는 `cbgl/src/cbgl_node/cbgl.cpp`이다.

1. `mapCallback()`
   - ROS `OccupancyGrid`를 RangeLib `OMap(width, height)`로 직접 구성
   - y축을 flip해서 `grid[x][y]`와 `raw_grid[x][y]`를 채움

2. `scanMap()` / `scanMapPanoramic()`
   - world 좌표를 grid 좌표로 바꿀 때 `map.info.origin`을 반영

3. `uniformPoseGenerator()`
   - map의 실제 world bounds를 사용
   - free-cell 검사도 같은 bounds 기준으로 수행

4. `siftThroughCAERPanoramic()`
   - yaw 갱신식에 맞도록 panoramic scan rotate 방향을 반대로 변경

5. `scanCallback()`
   - `NaN` 또는 `range_max` 초과값을 `range_max`로 정규화
   - min-range 보정도 정규화된 `latest_world_scan_` 기준으로 수행

## 현재 검증값

현재 실험 파라미터:

```text
laser_z_orientation = upwards
da = 32
top_k_caers = 100
undersample_rate = 4
```

서비스 호출 후 검증:

```text
world n=360 inf=0 nan=0 zero=0 finite_min=3.150 finite_max=20.000
top count=100 near1=22 near2=44 near3=48 nearest=0.228
nearest_top pose=(5.162, 5.160, -0.056)
result pose=(5.003, 5.104, -0.001) dist=0.104
```

## 재현 및 확인 명령

빌드:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build cbgl'
```

재시작:

```bash
docker restart cbgl
docker restart ros1_bridge
```

이 환경에서는 `cbgl`만 재시작하면 bridge discovery가 다시 붙지 않는 경우가 있었다. CBGL 재시작 후 `/map`, `/scan` publisher가 비어 있으면 `ros1_bridge`도 재시작한다.

토픽 연결 확인:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic info /map'
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic info /scan'
```

서비스 호출:

```bash
docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/cbgl/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization'
```

## 디버깅 순서

비슷한 문제가 다시 나오면 아래 순서로 본다.

1. `/map/info`의 width, height, resolution, origin 확인
2. `/scan`의 `angle_min`, `angle_max`, `angle_increment`, `range_max`, `inf` 개수 확인
3. `/tf`에서 `base_footprint -> lidar_link` 확인
4. `world_scan`에 `inf` 또는 `NaN`이 남는지 확인
5. `map_scan`이 전부 `20.0` 또는 `0.0`인지 확인
6. `particlecloud_all`에 정답 근처 particle이 있는지 확인
7. `particlecloud_top_k_caers`에 정답 근처 후보가 남는지 확인
8. 그 다음에 ICP correspondence 문제를 본다

## 주의할 점

- YAML origin을 `[0, 0, 0]`으로 임의 변경해서 맞추는 방식은 권장하지 않는다. Gazebo/world, map server, CBGL이 같은 map frame을 보게 원본 origin을 유지하는 편이 안전하다.
- 벽을 feature에서 제거하는 방식은 근본 해결이 아니다. 벽은 localization에 필요한 주요 feature다.
- `top_k_caers`를 크게 늘리는 것은 진단에는 도움이 되지만, CAER 점수 자체가 깨져 있으면 근본 해결이 되지 않는다.
- 원본 `/scan`에 `inf`가 있는 것은 자연스럽다. 문제는 CBGL 내부 비교용 scan에 non-finite 값이 남는 것이다.
