# Relocalization Known Simple Service

## 사용자 요청

- `rosservice call /relocalization_simple_request`는 기존처럼 `/scan` topic을 사용한다.
- `rosservice call /relocalization_simple_request_known`는 `/scan_known` topic을 사용해 relocalization을 실행한다.

## 요구사항

- 기존 simple service 동작을 유지한다.
- 새 simple service는 scan source만 `/scan_known`으로 바꾼다.
- TAM, SMAD, GICP, invalid range 처리 정책은 변경하지 않는다.
- 새 srv/message 타입은 추가하지 않고 기존 `std_srvs/Empty` 방식을 유지한다.

## 구현 내용

- `relocalization_2d_handler.cpp`에 scan topic 이름을 받는 내부 helper를 추가했다.
  - 기본 full request callback은 helper를 `scan`으로 호출한다.
  - 기존 `relocalization_simple_request`는 기존대로 `scan`을 사용한다.
  - 신규 `relocalization_simple_request_known`는 helper를 `scan_known`으로 호출한다.
- 신규 service server 변수와 service advertise 구문을 추가했다.

## 변경 파일

- `relocalization_toolbox/relocalization_toolbox/relocalization_toolbox/src/relocalization/relocalization_2d/relocalization_2d_handler.cpp`
- `docs/plans/2026-05-07_2138_relocalization-known-simple-service-plan.md`
- `docs/features/2026-05-07_2202_relocalization-known-simple-service.md`

## 동작 정리

- `/relocalization_simple_request`
  - map source: `map`
  - scan source: `scan`
- `/relocalization_simple_request_known`
  - map source: `map`
  - scan source: `scan_known`
- `/relocalization_request`, `/get_relocalization_candidates_request`, `/score_relocalization_pose`의 topic mode는 기존대로 `scan`을 사용한다.

## 검증 결과

- `git diff --check`
  - 성공
- Docker 기준 빌드:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; cd /home/relocalization/catkin_ws; catkin build relocalization_toolbox --no-status'
```

  - 성공: `relocalization_toolbox_msgs`, `relocalization_toolbox` 모두 빌드 성공
- 새 바이너리 적용:

```bash
docker restart relocalization_toolbox
```

  - 성공
- service 등록 확인:

```bash
docker exec relocalization_toolbox bash -lc 'source /opt/ros/noetic/setup.bash; source /home/relocalization/catkin_ws/devel/setup.bash; rosservice list | grep relocalization_simple_request'
```

  - 확인 결과:
    - `/relocalization_simple_request`
    - `/relocalization_simple_request_known`

## 남은 작업

- 실제 알고리즘 실행 비교는 실험 시점의 `/scan`, `/scan_known`, `/map`, TF 상태를 맞춘 뒤 아래 명령으로 수행한다.

```bash
rosservice call /relocalization_simple_request
rosservice call /relocalization_simple_request_known
```
