# 작업 계획서

## 요청 요약

- 사용자 요청: relocalization에서도 CBGL처럼 기본 simple service는 `/scan`을 사용하고, known simple service는 `/scan_known`을 사용하도록 만든다.
- 목표: `rosservice call /relocalization_simple_request`는 기존대로 `scan` topic을 사용하고, 새 `rosservice call /relocalization_simple_request_known`은 `scan_known` topic을 사용해 같은 relocalization flow를 실행한다.
- 제외 범위: TAM/SMAD/GICP 알고리즘 scoring 변경, scan invalid range 처리 정책 변경, launch/YAML/Docker/bridge 구성 변경, relocalization message/service 타입 추가.

## 현재 상태 확인

- 확인한 파일/토픽/서비스:
  - `relocalization_toolbox/relocalization_toolbox/relocalization_toolbox/src/relocalization/relocalization_2d/relocalization_2d_handler.cpp`
  - 기존 simple service: `relocalization_simple_request`
  - 기존 full service: `relocalization_request`
  - 기존 scan topic lookup: `ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan")`
- 관측 요약:
  - `relocalization_simple_request_callback()`은 `std_srvs::Empty` callback이며, 내부에서 `relocalization_request` request를 만들고 `MODE_FROM_TOPIC`으로 기존 callback을 호출한다.
  - 기존 `relocalization_request_callback()`의 topic mode는 `scan` topic 이름이 코드에 직접 고정되어 있다.
  - service 광고는 같은 파일 하단에서 `node_handle.advertiseService("relocalization_simple_request", ...)`로 등록된다.
- 관련 제약:
  - 구현 전 설계안 보고 및 승인 필요.
  - `relocalization_toolbox/`는 사용자가 명시 요청한 범위에서만 수정한다.
  - 새 message/service type은 추가하지 않고 기존 `std_srvs/Empty`를 재사용한다.

## 영향 범위

- 패키지: `relocalization_toolbox`
- 파일 후보:
  - `relocalization_toolbox/relocalization_toolbox/relocalization_toolbox/src/relocalization/relocalization_2d/relocalization_2d_handler.cpp`
  - 개발 완료 문서: `docs/features/2026-05-07_HHMM_relocalization-known-simple-service.md`
- ROS topic/service/frame:
  - 기존 유지: `/relocalization_simple_request` -> `scan`
  - 신규 추가: `/relocalization_simple_request_known` -> `scan_known`
  - map topic, frame, TF, init pose publish 동작은 기존과 동일하게 유지
- Docker/launch/parameter 영향:
  - Docker/launch/YAML 수정 없음.
  - 새 binary 적용을 위해 빌드 후 컨테이너 재시작은 필요할 수 있다.

## 구현 설계안

- 변경 방식:
  - 기존 `relocalization_request_callback()` 로직은 유지한다.
  - topic 이름을 인자로 받는 작은 내부 helper를 추가해 `MODE_FROM_TOPIC`일 때 `scan` 또는 `scan_known`을 선택하게 한다.
  - 기존 `relocalization_simple_request_callback()`은 helper를 통해 `scan`을 사용한다.
  - 새 `relocalization_simple_request_known_callback()`을 추가해 같은 flow에서 `scan_known`을 사용한다.
  - service 등록부에 `relocalization_simple_request_known`을 추가한다.
- 대안:
  - 기존 srv 타입에 topic option field를 추가하거나, 별도 service type을 만들 수 있다.
  - 하지만 message/service 타입 추가는 CMake/package 의존 변경과 bridge 영향이 커서 이번 요구에는 과하다.
- 선택 이유:
  - 기존 simple service 사용법을 깨지 않는다.
  - 알고리즘 입력 topic만 바꾸므로 CBGL의 `/global_localization` vs `/global_localization_known` 실험 구조와 대응된다.
  - 기존 relocalization request/response 처리, map loading, TF, init pose publish 경로를 그대로 재사용한다.
- 리스크:
  - `scan_known` topic이 publish되지 않으면 새 service는 scan 수신 실패로 끝난다.
  - `scan_known`의 frame_id/range metadata가 기존 `/scan`과 다르면 기존 코드처럼 `lidar_frame`으로 덮어쓰지만, angle/range 구성 차이는 알고리즘 입력 차이로 반영된다.

## 검증 계획

- 빌드:
  - `docker exec cbgl bash -lc 'source /opt/ros/kinetic/setup.bash; cd /home/cbgl/catkin_ws; catkin build relocalization_toolbox'`
- 실행/서비스 호출:
  - `rosservice list | grep relocalization_simple_request`
  - `rosservice call /relocalization_simple_request`
  - `rosservice call /relocalization_simple_request_known`
- topic/frame 확인:
  - `rostopic echo -n 1 /scan`
  - `rostopic echo -n 1 /scan_known`
  - 필요 시 `rosrun tf tf_echo map base_footprint`
- 실패 시 확인할 항목:
  - `/scan_known` publish 여부
  - service namespace
  - relocalization node가 새 binary로 재시작되었는지
  - `/map`, lidar/base transform 수신 여부

## 문서화 계획

- 작업 계획 문서: `docs/plans/2026-05-07_2138_relocalization-known-simple-service-plan.md`
- 기능 결과 문서: `docs/features/2026-05-07_HHMM_relocalization-known-simple-service.md`
- 운영 문서 갱신: 이번 변경은 코드와 기능 결과 문서만 갱신하고 README/AGENTS는 수정하지 않는다.

## 승인 상태

- 사용자 승인 문구: `진행해`
- 승인 시각: 2026-05-07 22:02
