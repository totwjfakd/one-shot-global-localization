# CBGL 프로젝트 설명서

이 문서는 현재 저장소의 코드를 기준으로 CBGL 프로젝트의 목적, 구조, 실행 흐름, 주요 파라미터, 그리고 코드에 사용된 핵심 수식을 쉽게 풀어서 정리한 문서입니다.

## 1. 프로젝트 한 줄 요약

CBGL은 2D 점유 격자 지도(`nav_msgs/OccupancyGrid`)와 단 한 번의 2D 라이다 스캔(`sensor_msgs/LaserScan`)만으로 라이다/로봇의 전역 위치와 방향을 빠르게 찾는 ROS 패키지입니다.

전역 위치 추정은 보통 “처음 로봇이 지도 어디에 있는지 전혀 모르는 상태”에서 시작합니다. 이 프로젝트는 지도 전체의 자유 공간에 많은 후보 pose를 뿌리고, 각 후보에서 가상의 라이다 스캔을 만들어 실제 스캔과 비교한 뒤, 가장 그럴듯한 후보만 정밀 보정합니다.

## 2. 저장소 구성

| 경로 | 역할 |
| --- | --- |
| `README.md` | 원본 프로젝트 소개, Docker 설치/실행, 논문 인용 정보 |
| `cbgl/` | 원본 ROS1 Kinetic용 CBGL 패키지 |
| `cbgl/src/cbgl_node/cbgl.cpp` | CBGL의 핵심 실행 로직 |
| `cbgl/include/cbgl_node/cbgl.h` | CBGL 클래스, ROS 입출력, 파라미터, 내부 함수 선언 |
| `cbgl/configuration_files/params_cbgl.yaml` | CBGL 주요 파라미터 |
| `cbgl/configuration_files/params_csm.yaml` | ICP/CSM scan matching 파라미터 |
| `cbgl/configuration_files/params_fsm.yaml` | FSM scan matching 파라미터 |
| `cbgl/include/utils/fsm_core.h` | FFT 기반 FSM 알고리즘 구현 |
| `cbgl/src/utils/occupancy_grid_utils/` | OccupancyGrid 좌표 변환, Bresenham ray tracing |
| `cbgl/src/utils/pf/` | AMCL 계열 particle filter 유틸리티 |
| `cbgl/include/utils/range_libc/` | 빠른 2D ray casting 벤더 라이브러리 |
| `docker/` | ROS1 Kinetic Docker 빌드/실행 설정 |
| `ros2_humble_ws/` | 원본을 덮어쓰지 않고 분리한 ROS2 Humble 포팅 workspace |

## 3. 입출력

### 입력

| 입력 | 기본값 | 설명 |
| --- | --- | --- |
| 지도 topic | `/map` | 2D 점유 격자 지도 |
| 라이다 topic | `/scan` | 실제 라이다 거리 배열 |
| 서비스 호출 | `/robot/cbgl_node/global_localization` | 전역 위치 추정을 1회 실행 |
| TF | `base_footprint`, `odom`, `map`, 라이다 frame | 라이다와 base 사이의 정적 변환, 결과 TF broadcast에 사용 |

### 출력

| 출력 | 설명 |
| --- | --- |
| `/initialpose` | 최종 추정 pose를 `PoseWithCovarianceStamped`로 publish |
| `/robot/cbgl_node/execution_time` | 실행 시간 |
| `/robot/cbgl_node/best_particle` | 선택된 최고 후보 index |
| `/robot/cbgl_node/particlecloud_*` | 디버깅용 후보 pose 집합 |
| `map -> odom` TF | `tf_broadcast: true`일 때 publish |

## 4. 전체 실행 흐름

CBGL의 핵심 흐름은 다음과 같습니다.

1. 지도 수신
   - OccupancyGrid를 내부 map 구조로 변환합니다.
   - 자유 공간 면적을 계산합니다.
   - 자유 공간 면적과 `dl` 값을 이용해 위치 후보 수를 정합니다.

2. 라이다 스캔 수신
   - 필요하면 undersampling을 적용합니다.
   - invalid range, infinite range를 보정합니다.
   - 첫 스캔에서는 base와 laser 사이의 TF, FSM용 FFTW plan 등을 캐시합니다.

3. 서비스 호출
   - 지도 전체 자유 공간에 pose 후보를 균일하게 생성합니다.
   - 각 위치 후보에 대해 `da`개의 방향 후보를 만들어 전체 pose 후보 집합을 구성합니다.

4. CAER 기반 후보 압축
   - 각 후보 pose에서 지도 기반 가상 라이다 스캔을 생성합니다.
   - 실제 스캔과 가상 스캔의 차이를 CAER 값으로 계산합니다.
   - CAER가 가장 작은 `top_k_caers`개 후보만 다음 단계로 넘깁니다.

5. Scan-to-map-scan 정밀 보정
   - 기본값은 ICP/CSM(`do_icp: true`)입니다.
   - 선택적으로 panoramic sensor에서는 FSM(`do_fsm: true`)을 사용할 수 있습니다.
   - 각 후보 pose를 실제 스캔에 더 잘 맞게 미세 보정합니다.

6. 최종 후보 선택
   - 보정된 후보마다 다시 가상 스캔을 만들고 CAER를 계산합니다.
   - `1 / (CAER + 1e-10)` 점수가 가장 큰 후보를 최종 pose로 선택합니다.

7. 결과 publish
   - 최종 pose와 covariance를 `/initialpose`로 publish합니다.
   - 설정에 따라 `map -> odom` TF도 broadcast합니다.

## 5. 주요 파라미터

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `dl` | `15` | 자유 공간 1 m^2당 위치 후보 수 |
| `da` | `32` | 한 위치에서 검사할 방향 후보 수 |
| `top_k_caers` | `10` | CAER가 가장 낮은 후보 중 scan matching까지 보낼 개수 |
| `map_scan_method` | `ray_marching` | 가상 스캔 생성 방식: `vanilla`, `ray_marching`, `cddt`, `bresenham` |
| `laser_z_orientation` | `upwards` | 라이다 z축 방향. 각도 증가 방향 보정에 사용 |
| `do_undersample_scan` | `true` | 라이다 ray 수를 줄일지 여부 |
| `undersample_rate` | `1` | `4`이면 4개 중 1개 ray만 사용 |
| `do_icp` | `true` | ICP/CSM 기반 정밀 보정 사용 |
| `do_fsm` | `false` | FFT 기반 FSM 정밀 보정 사용 |
| `tf_broadcast` | `true` | 최종 pose를 `map -> odom` TF로 broadcast |

후보 수에 가장 큰 영향을 주는 값은 `dl`, `da`, 지도 자유 공간 면적입니다.

## 6. 핵심 수식 설명

### 6.1 자유 공간 면적

코드 위치: `CBGL::freeArea`

지도에서 자유 셀 수를 세고, 셀 하나의 실제 면적을 곱합니다.

```text
A_free = N_free * resolution^2
```

| 기호 | 의미 |
| --- | --- |
| `A_free` | 지도에서 이동 가능한 자유 공간 면적 |
| `N_free` | free cell 개수 |
| `resolution` | OccupancyGrid 한 칸의 크기, 단위 m/cell |

예를 들어 resolution이 `0.05 m/cell`이고 자유 셀이 10000개라면,

```text
A_free = 10000 * 0.05^2 = 25 m^2
```

### 6.2 초기 위치 후보 수

코드 위치: `CBGL::mapCallback`, `CBGL::siftThroughCAERPanoramic`

위치 후보 수는 자유 공간 면적에 위치 밀도 `dl`을 곱해 정합니다.

```text
N_pos = dl * A_free
```

각 위치마다 방향 후보 `da`개를 검사하므로 전체 pose 후보 수는 대략 다음과 같습니다.

```text
N_pose = N_pos * da = dl * A_free * da
```

예를 들어 `dl = 15`, `A_free = 100 m^2`, `da = 32`이면,

```text
N_pos = 15 * 100 = 1500
N_pose = 1500 * 32 = 48000
```

즉 위치 1500개를 뿌리고, 각 위치에서 32개 방향을 돌려보는 식입니다.

### 6.3 균일 pose 생성

코드 위치: `CBGL::uniformPoseGenerator`

후보 pose는 지도 bounds 안에서 랜덤하게 뽑습니다.

```text
x = x_min + rand(0, 1) * (x_max - x_min)
y = y_min + rand(0, 1) * (y_max - y_min)
theta = rand(0, 1) * 2*pi - pi
```

그 뒤 `(x, y)`가 지도 안의 free cell인지 검사합니다. 장애물이나 unknown cell이면 다시 뽑습니다.

직관적으로 말하면 “지도 전체에서 아무 곳이나 찍되, 실제로 로봇이 있을 수 있는 빈 공간만 통과시킨다”는 뜻입니다.

### 6.4 2D pose와 변환 행렬

코드 위치: `CBGL::createTfFromXYTheta`, `FSM::Utils::computeTransform`

2D pose는 보통 다음 3개 값으로 표현합니다.

```text
p = (x, y, theta)
```

이를 행렬로 쓰면 다음과 같습니다.

```text
T(x, y, theta) =
| cos(theta)  -sin(theta)   x |
| sin(theta)   cos(theta)   y |
|     0            0        1 |
```

이 행렬은 점을 회전한 뒤 평행이동합니다. CBGL에서는 base pose, laser pose, ICP 보정량, `map -> odom` TF를 조합할 때 같은 개념을 사용합니다.

### 6.5 라이다 ray 각도

코드 위치: `CBGL::laserScanToLDP`, `occupancy_grid_utils::ScanEndPoints`

라이다 스캔의 i번째 ray 각도는 다음과 같습니다.

```text
theta_i = angle_min + i * angle_increment
```

각 ray의 끝점은 다음처럼 계산됩니다.

```text
x_i = x_sensor + r_i * cos(theta_sensor + theta_i)
y_i = y_sensor + r_i * sin(theta_sensor + theta_i)
```

| 기호 | 의미 |
| --- | --- |
| `r_i` | i번째 라이다 거리 |
| `(x_sensor, y_sensor)` | 라이다 위치 |
| `theta_sensor` | 라이다 방향 |
| `theta_i` | 라이다 기준 i번째 ray 각도 |

### 6.6 지도에서 가상 라이다 스캔 생성

코드 위치: `CBGL::scanMap`, `CBGL::scanMapPanoramic`

CBGL은 후보 pose가 진짜 pose라고 가정하고, 지도 위에서 가상의 라이다 스캔을 쏩니다.

RangeLibc 방식에서는 먼저 월드 좌표를 grid 좌표로 바꿉니다.

```text
x_grid = x_world / resolution
y_grid = map_height - 1 - y_world / resolution
```

그 다음 i번째 ray의 grid 기준 각도는 다음과 같습니다.

```text
alpha_i = -yaw + sgn * (angle_min + i * angle_increment)
```

| 값 | 의미 |
| --- | --- |
| `yaw` | 현재 후보 pose의 방향 |
| `sgn` | 라이다 z축 방향 보정값. `upwards`이면 `-1`, `downwards`이면 `+1` |
| `alpha_i` | 지도 ray casting에 넘기는 각도 |

ray caster는 grid 단위의 거리를 반환하므로 실제 m 단위 range는 다음과 같습니다.

```text
r_i = resolution * calc_range(x_grid, y_grid, alpha_i)
```

### 6.7 Ray casting 거리

코드 위치: `occupancy_grid_utils::simulateRangeScan`, `ray_trace_iterator.h`, RangeLibc

기본적인 거리 계산은 유클리드 거리입니다.

```text
d = sqrt((x - x0)^2 + (y - y0)^2)
```

`vanilla` 방식은 Bresenham line tracing을 사용합니다. 시작 cell에서 ray 방향으로 cell을 하나씩 따라가다가 occupied cell을 만나면 그 cell까지의 거리를 range로 사용합니다.

RangeLibc의 `ray_marching` 방식은 “현재 위치에서 가장 가까운 장애물까지의 거리만큼은 안전하게 전진할 수 있다”는 아이디어를 사용합니다.

```text
p(t) = (x + t*cos(alpha), y + t*sin(alpha))
```

여기서 `t`를 조금씩 증가시키며 장애물을 찾습니다. 실제 구현은 RangeLibc에 들어 있으며, CBGL은 `ray_marching`, `cddt`, `bresenham` 중 하나를 선택해서 호출합니다.

### 6.8 Panoramic scan 후보 방향 생성

코드 위치: `CBGL::scanMapPanoramic`, `CBGL::siftThroughCAERPanoramic`

실제 라이다가 360도 전체가 아닐 수도 있으므로, CBGL은 후보 위치에서 360도 가상 스캔을 만든 뒤 여러 방향으로 회전시켜 실제 스캔과 맞춰봅니다.

먼저 기존 스캔 ray 수를 360도 기준 ray 수로 환산합니다.

```text
N_360 = N_scan * (2*pi) / (angle_max - angle_min)
```

코드는 정수화한 뒤 짝수 개수로 맞춥니다.

방향 후보 k에 대한 yaw는 다음과 같습니다.

```text
theta_k = wrap(theta_original - 2*pi*k / da)
```

| 기호 | 의미 |
| --- | --- |
| `k` | 방향 후보 index, `0 <= k < da` |
| `da` | 한 위치에서 검사할 방향 개수 |
| `wrap` | 각도를 `[-pi, pi]` 범위로 접는 함수 |

즉 `da = 32`이면 360도를 32등분해서 약 11.25도 간격으로 방향을 검사합니다.

### 6.9 각도 wrap

코드 위치: `CBGL::wrapAngle`, `FSM::Utils::wrapAngle`

각도는 계속 더하고 빼다 보면 `pi`보다 커지거나 `-pi`보다 작아질 수 있습니다. 그래서 항상 `[-pi, pi]` 범위로 되돌립니다.

```text
wrap(theta) = fmod(theta + 5*pi, 2*pi) - pi
```

예를 들어 `theta = 3*pi`처럼 큰 각도도 결국 같은 방향을 가리키는 `pi` 근처 값으로 바뀝니다.

### 6.10 CAER: 실제 스캔과 가상 스캔의 차이

코드 위치: `CBGL::caer`

CAER는 “각 ray별 거리 차이의 절댓값을 모두 더한 값”입니다.

```text
CAER(s_real, s_virtual) =
sum_i |s_real[i] - s_virtual[i]|
```

단, 코드에서는 두 값이 모두 0보다 큰 ray만 더합니다.

```text
if s_real[i] > 0 and s_virtual[i] > 0:
    add |s_real[i] - s_virtual[i]|
```

CAER가 작다는 것은 실제 스캔과 지도에서 만든 가상 스캔이 비슷하다는 뜻입니다. 따라서 낮을수록 좋은 후보입니다.

주의할 점은 함수 이름에는 “per ray”가 들어가지만, 현재 코드에서는 ray 개수로 나누지 않고 합만 계산합니다.

### 6.11 CAER 기반 점수

코드 위치: `CBGL::processPoseCloud`

정밀 보정 후 최종 후보를 고를 때는 CAER를 점수로 뒤집어 사용합니다.

```text
score = 1 / (CAER + 1e-10)
```

`1e-10`은 CAER가 0일 때 0으로 나누는 문제를 피하기 위한 아주 작은 값입니다.

CAER가 작을수록 `score`는 커집니다.

### 6.12 invalid range 보간

코드 위치: `CBGL::interpolateRanges`

라이다 값이 0이거나 최소 range보다 작으면 false measurement로 보고 주변 정상값으로 보간합니다.

연속된 invalid 구간이 있을 때, 앞뒤 정상 range의 평균을 넣습니다.

```text
r_interp = (r_before + r_after) / 2
```

직관적으로는 “튀는 값이나 비어 있는 구간을 양옆 값의 중간 정도로 메운다”는 의미입니다.

### 6.13 ICP/CSM 보정

코드 위치: `CBGL::doICP`, `CBGL::processScan`, `CBGL::correctICPPose`

ICP는 실제 스캔과 지도에서 만든 가상 스캔이 가장 잘 겹치도록 작은 보정량을 찾습니다.

개념적으로는 다음 오차를 줄입니다.

```text
E(delta) = sum_i w_i * distance(transformed_point_i, matched_line_i)^2
```

현재 설정은 `use_point_to_line_distance: 1`이므로 point-to-line ICP를 사용합니다.

CSM이 찾은 보정량은 라이다 frame에서의 작은 이동입니다.

```text
delta_laser = (dx, dy, dtheta)
```

이를 2D transform으로 만들면 다음과 같습니다.

```text
C_laser = T(dx, dy, dtheta)
```

하지만 최종 pose는 base frame 기준이어야 하므로, 라이다 보정을 base 보정으로 바꿉니다.

```text
C_base = T_base_laser * C_laser * T_laser_base
```

그 다음 후보 pose에 이 보정을 적용합니다.

```text
T_corrected = T_candidate * C_base
```

쉽게 말하면, “라이다 기준으로 이만큼 틀어져 있다”는 ICP 결과를 “로봇 base 기준으로 이만큼 틀어져 있다”로 바꾼 뒤 후보 pose를 업데이트합니다.

### 6.14 CSM 가중치 관련 수식

코드 위치: `params_csm.yaml`, CSM 외부 라이브러리

CSM 파라미터에는 다음 가중치 옵션이 있습니다.

```text
use_sigma_weights: correspondence weight = 1 / sigma^2
use_ml_weights:    incidence weight ~= 1 / cos(beta)^2
```

| 기호 | 의미 |
| --- | --- |
| `sigma` | range 측정 노이즈 표준편차 |
| `beta` | ray가 표면에 들어가는 입사각 |

현재 기본 설정에서는 둘 다 `0`이라서 이 가중치들은 사용하지 않습니다.

### 6.15 FSM의 scan-to-point 변환

코드 위치: `FSM::Utils::scan2points`

FSM은 panoramic scan을 점 집합으로 바꾸어 사용합니다.

```text
x_i = x + r_i * cos(i * angle_span / N + theta - angle_span/2)
y_i = y + r_i * sin(i * angle_span / N + theta - angle_span/2)
```

| 기호 | 의미 |
| --- | --- |
| `N` | scan ray 수 |
| `angle_span` | 스캔 시야각. 기본값은 `2*pi` |
| `(x, y, theta)` | 현재 pose |

360도 라이다라면 `angle_span = 2*pi`입니다.

### 6.16 FSM의 DFT

코드 위치: `FSM::DFTUtils`

FSM은 FFTW를 사용해 discrete Fourier transform을 계산합니다. 이산 신호 `x[n]`의 DFT는 개념적으로 다음과 같습니다.

```text
X[k] = sum_{n=0}^{N-1} x[n] * exp(-j * 2*pi*k*n/N)
```

역변환은 다음과 같습니다.

```text
x[n] = (1/N) * sum_{k=0}^{N-1} X[k] * exp(j * 2*pi*k*n/N)
```

DFT를 쓰는 이유는 회전 차이를 빠르게 찾기 위해서입니다. 360도 스캔은 원형 배열처럼 볼 수 있으므로, 한쪽 스캔을 회전시키는 문제를 correlation peak를 찾는 문제로 바꿀 수 있습니다.

### 6.17 FSM 회전 추정: Fourier-Mellin Transform 계열

코드 위치: `FSM::Rotation::fmt0Sequential`, `FSM::Rotation::fmt1Sequential`, batch 버전들

실제 스캔 `s`와 가상 스캔 `r`의 DFT를 구합니다.

```text
S = DFT(s)
R = DFT(r)
```

두 신호의 circular correlation은 다음처럼 계산합니다.

```text
q = IDFT(conj(S) * R)
```

여기서 `q`가 가장 큰 index를 찾습니다.

```text
k_peak = argmax_k q[k]
```

회전 오차 후보는 다음과 같습니다.

```text
dtheta = wrap((N - k_peak) / N * 2*pi)
```

직관적으로는 “가상 스캔을 몇 칸 밀면 실제 스캔과 가장 비슷해지는가?”를 찾는 것입니다. 몇 칸 밀었는지를 각도로 바꾼 것이 `dtheta`입니다.

### 6.18 FSM 회전 후보 평가 지표

코드 위치: `FSM::Rotation::fmt1Sequential`, `FSM::Rotation::fmt1Batch`

회전 correlation 결과 `q`에서 peak가 얼마나 뚜렷한지 평가합니다.

#### SNR

```text
SNR = |q_peak - mean(q_background)| / std(q_background)
```

peak 하나를 제외한 배경값의 평균과 표준편차를 구하고, peak가 배경보다 얼마나 튀는지 봅니다. 값이 클수록 회전 후보가 뚜렷합니다.

#### FAHM

```text
FAHM = count(q[i] >= 0.5 * q_peak) / N
```

peak 절반 이상인 구간이 얼마나 넓은지 보는 값입니다. 너무 넓으면 peak가 뭉툭해서 회전이 애매할 수 있습니다.

#### PD

```text
PD = 2 * q_peak / (q_self_real_peak + q_self_virtual_peak)
```

실제-가상 matching peak를 실제-실제, 가상-가상 자기상관 peak와 비교해 정규화한 값입니다. 두 스캔이 잘 맞으면 커집니다.

코드에서는 후보를 고를 때 `PD`를 주요 기준으로 쓰고, 기록용 회전 기준으로는 다음 값을 저장합니다.

```text
rc0 = PD
rc1 = SNR / FAHM
```

### 6.19 FSM translation 추정

코드 위치: `FSM::Translation::tff`, `FSM::Translation::tffCore`, `turnDFTCoeffsIntoErrors`

FSM translation 단계에서는 실제 스캔과 현재 pose에서 만든 가상 스캔의 차이를 구합니다.

```text
d_i = s_real[i] - s_virtual[i]
```

단, 차이가 너무 큰 ray는 translation 계산에서 제외하기 위해 0으로 만듭니다.

```text
if |d_i| <= inclusion_bound + eps:
    diff_i = d_i
else:
    diff_i = 0
```

이 `diff` 신호의 첫 번째 DFT 계수 `X_1`을 사용합니다.

```text
X_1 = a + j*b
```

현재 방향을 `theta`라고 하면 translation correction은 다음과 같습니다.

```text
phi = pi + theta

dx = (-a*cos(phi) - b*sin(phi)) / N
dy = (-a*sin(phi) + b*cos(phi)) / N
```

즉 range 차이 배열의 1차 Fourier 성분만으로 x, y 방향 보정량을 계산합니다. 직관적으로는 “전체 ray가 어느 방향으로 더 길거나 짧게 보이는가”를 보고 pose를 그 반대 방향으로 조금 옮기는 방식입니다.

### 6.20 FSM 반복 구조

코드 위치: `FSM::Match::fmtdbh`

FSM은 한 번에 끝내지 않고 다음 과정을 반복합니다.

1. 회전 후보를 찾습니다.
2. 후보 회전마다 짧은 translation 보정을 시험합니다.
3. translation criterion이 가장 작은 회전을 고릅니다.
4. 선택된 회전을 pose에 적용합니다.
5. translation 보정을 여러 번 수행합니다.
6. 더 이상 좋아지지 않거나 해상도 level을 올릴 조건이 되면 다음 magnification level로 넘어갑니다.

각 magnification level에서 가상 스캔 수는 다음과 같습니다.

```text
N_virtual_scans = 2^magnification_size
```

translation criterion은 평균 절대 range 차이에 가깝게 사용됩니다.

```text
tc = sum_i |d_i| / N
```

값이 작을수록 실제 스캔과 가상 스캔이 잘 맞습니다.

### 6.21 Particle filter 유틸리티의 weight 정규화

코드 위치: `pf_update_sensor`

현재 CBGL의 전역 후보 생성은 주로 `pf_init_model`을 사용하지만, 포함된 particle filter 유틸리티에는 일반적인 weight 정규화가 들어 있습니다.

```text
w_i_normalized = w_i / sum_j w_j
```

평균 weight도 계산합니다.

```text
w_avg = (sum_i w_i) / N
```

그리고 slow/fast running average를 업데이트합니다.

```text
w_slow = w_slow + alpha_slow * (w_avg - w_slow)
w_fast = w_fast + alpha_fast * (w_avg - w_fast)
```

이는 particle filter가 “현재 샘플들이 관측과 얼마나 잘 맞는지”를 천천히/빠르게 추적하는 값입니다.

### 6.22 KLD adaptive resampling 제한

코드 위치: `pf_resample_limit`

particle filter 유틸리티에는 Fox 계열 KLD-sampling에서 쓰이는 샘플 수 제한식이 들어 있습니다.

```text
n = ceil(
  (k - 1) / (2 * err)
  * (1 - 2/(9*(k-1)) + z * sqrt(2/(9*(k-1))))^3
)
```

| 기호 | 의미 |
| --- | --- |
| `k` | 현재 샘플이 차지하는 histogram bin 수 |
| `err` | 허용 오차 |
| `z` | 정규분포 분위수 |
| `n` | 필요한 샘플 수 |

`k`가 커질수록 분포가 넓고 복잡하다는 뜻이므로 더 많은 샘플이 필요합니다.

### 6.23 Gaussian sampling

코드 위치: `pf_pdf_gaussian_alloc`, `pf_pdf_gaussian_sample`, `pf_ran_gaussian`

가우시안 샘플링은 covariance를 회전 성분과 축별 분산으로 분해한 뒤 수행합니다.

```text
Sigma = R * D * R^T
```

각 축에서 랜덤 값을 뽑습니다.

```text
r_i ~ N(0, sqrt(D_ii))
```

최종 샘플은 다음과 같습니다.

```text
x_sample = mu + R * r
```

난수는 Box-Muller 변환의 polar form을 사용합니다.

```text
z = x2 * sqrt(-2 * log(w) / w)
sample = sigma * z
```

여기서 `w = x1^2 + x2^2`, `x1`, `x2`는 `[-1, 1]` 범위의 균일 난수입니다.

### 6.24 원형 평균

코드 위치: `pf_cluster_stats`

각도는 단순 평균을 내면 문제가 생깁니다. 예를 들어 `179도`와 `-179도`의 평균은 실제로는 `180도` 근처여야 하는데, 숫자로만 평균 내면 `0도`가 됩니다.

그래서 sin/cos 평균을 사용합니다.

```text
C = sum_i w_i * cos(theta_i)
S = sum_i w_i * sin(theta_i)
theta_mean = atan2(S, C)
```

이 방식은 각도가 원 위에 있다는 사실을 반영합니다.

### 6.25 좌표 변환과 grid index

코드 위치: `coordinate_conversions.h`, `map.h`

OccupancyGrid에서 cell index는 다음처럼 계산됩니다.

```text
index = x_cell + y_cell * width
```

월드 좌표를 cell 좌표로 바꿀 때는 map origin transform을 역으로 적용한 뒤 resolution으로 나눕니다.

```text
x_cell = floor(x_map / resolution)
y_cell = floor(y_map / resolution)
```

cell 중심의 월드 좌표는 다음과 같습니다.

```text
x_center = (x_cell + 0.5) * resolution
y_center = (y_cell + 0.5) * resolution
```

이 좌표 변환이 맞아야 ray casting과 후보 free-cell 검사가 정확해집니다.

### 6.26 실행 시간 경험식

README에는 실행 시간이 대략 다음에 비례한다고 설명되어 있습니다.

```text
T ~= 10e * A_free * N_s microseconds
```

| 기호 | 의미 |
| --- | --- |
| `A_free` | 자유 공간 면적 |
| `N_s` | 라이다 ray 수 |

엄밀한 이론식이라기보다는 환경 구조, ray casting 방법, 후보 수 설정에 따라 달라지는 경험적 추정입니다. 그래도 큰 흐름은 명확합니다.

```text
지도 자유 공간이 넓을수록 느려짐
라이다 ray 수가 많을수록 느려짐
dl, da, top_k_caers가 클수록 느려짐
```

## 7. 알고리즘을 쉬운 말로 다시 정리

CBGL은 다음 질문을 빠르게 푸는 프로그램입니다.

```text
이 라이다 스캔은 지도 어느 위치와 방향에서 봤을 때 가장 비슷하게 보일까?
```

이를 위해 먼저 지도 전체에 많은 후보를 뿌립니다. 모든 후보를 정밀하게 ICP하면 너무 느리므로, 먼저 가벼운 CAER 점수로 후보를 크게 줄입니다. 그 다음 정말 가능성이 높은 후보 몇 개에만 ICP 또는 FSM을 적용합니다.

비유하면 다음과 같습니다.

1. 지도 전체에 “여기일 수도 있음” 스티커를 많이 붙입니다.
2. 각 스티커 위치에서 가상의 라이다를 돌려봅니다.
3. 실제 라이다 모양과 너무 다른 스티커는 버립니다.
4. 남은 스티커를 정밀하게 맞춰봅니다.
5. 제일 잘 맞는 스티커를 최종 위치로 발표합니다.

## 8. ROS1 실행 방법

README 기준으로 Docker 실행을 권장합니다.

```bash
docker compose build
docker compose up
```

실행 후 전역 위치 추정 서비스 호출:

```bash
docker exec -it cbgl sh -c "source ~/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization"
```

launch 파일은 `cbgl/launch/cbgl.launch`입니다. 기본 namespace는 `robot`입니다.

## 9. ROS2 Humble 포팅본

이 저장소에는 `ros2_humble_ws/` 아래에 ROS2 Humble 포팅 결과가 따로 있습니다. 원본 ROS1 패키지를 직접 덮어쓰지 않고 분리한 구조입니다.

주요 변경점은 다음과 같습니다.

| ROS1 | ROS2 |
| --- | --- |
| `catkin` | `ament_cmake` |
| `roscpp` | `rclcpp` |
| `tf` | `tf2`, `tf2_ros`, `tf2_geometry_msgs` |
| ROS1 message include | ROS2 `*_msgs/msg/*` include |
| `std_srvs/Empty` | `std_srvs/srv/Empty` |
| `std_msgs/Duration` | `builtin_interfaces/msg/Duration` |

CSM은 ROS2 Humble 공식 패키지가 없어서 `csm_vendor` 패키지로 workspace 안에서 함께 빌드하도록 구성되어 있습니다.

ROS2 실행 명령은 다음 형태입니다.

```bash
cd ros2_humble_ws
colcon build
source install/setup.bash
ros2 launch cbgl cbgl.launch.py
```

서비스 호출:

```bash
ros2 service call /robot/cbgl_node/global_localization std_srvs/srv/Empty {}
```

단, `ros2_humble_ws/ROS2_HUMBLE_PORTING_RESULT.md`에 따르면 현재 세션에서는 시스템 의존성 미설치로 전체 빌드 검증은 완료되지 않았습니다. 필요한 패키지는 다음과 같습니다.

```text
libgsl-dev
libfftw3-3
libfftw3-dev
libcgal-dev
```

## 10. 성능과 정확도에 큰 영향을 주는 부분

| 항목 | 영향 |
| --- | --- |
| `dl` 증가 | 위치 후보가 많아져 성공률은 오를 수 있지만 느려짐 |
| `da` 증가 | 방향 후보가 촘촘해져 초기 방향 탐색은 좋아지지만 느려짐 |
| `top_k_caers` 증가 | 더 많은 후보를 ICP/FSM으로 정밀 보정하므로 느려짐 |
| `undersample_rate` 증가 | ray 수가 줄어 빨라질 수 있지만 scan 정보가 줄어듦 |
| `map_scan_method` | `ray_marching`, `cddt`는 빠른 ray casting을 노린 선택지 |
| 지도 대칭성 | 비슷한 구조가 반복되는 지도에서는 후보 구분이 어려워질 수 있음 |
| 라이다 FOV | FSM은 panoramic sensor, 즉 360도 스캔에 더 적합 |

## 11. 코드 기준 핵심 함수 지도

| 함수 | 역할 |
| --- | --- |
| `CBGL::mapCallback` | 지도 저장, 자유 공간 면적 계산, particle filter 후보 수 설정 |
| `CBGL::scanCallback` | 라이다 스캔 저장, undersampling, invalid range 처리 |
| `CBGL::startSignalService` | 서비스 호출 시 전역 위치 추정 시작 |
| `CBGL::uniformPoseGenerator` | 자유 공간 위 균일 pose 후보 생성 |
| `CBGL::processPoseCloud` | 전체 파이프라인 실행, 후보 평가, 최종 pose publish |
| `CBGL::siftThroughCAERPanoramic` | CAER로 상위 후보 선택 |
| `CBGL::scanMap` | 후보 pose에서 지도 기반 가상 스캔 생성 |
| `CBGL::scanMapPanoramic` | 360도 가상 스캔 생성 |
| `CBGL::caer` | 실제 스캔과 가상 스캔의 절대 오차 합 |
| `CBGL::doICP` | CSM ICP 보정 실행 |
| `CBGL::doFSM` | FFT 기반 FSM 보정 실행 |
| `CBGL::correctICPPose` | scan matcher 보정량을 최종 pose에 반영 |
| `FSM::Rotation::fmt*` | DFT correlation 기반 회전 추정 |
| `FSM::Translation::tff*` | 첫 DFT 계수 기반 translation 추정 |

## 12. 결론

CBGL은 전역 위치 추정을 다음 두 단계로 나누어 빠르게 처리합니다.

```text
1. 넓게 찾기: 지도 전체 후보를 CAER로 빠르게 줄임
2. 좁게 맞추기: 남은 후보만 ICP/FSM으로 정밀 보정
```

핵심 아이디어는 단순하지만 강력합니다. 실제 라이다 스캔과 지도에서 만든 가상 라이다 스캔이 가장 비슷해지는 pose를 찾는 것이며, 이 비슷함을 CAER, ICP 오차, FSM의 Fourier correlation 같은 수식으로 계산합니다.

