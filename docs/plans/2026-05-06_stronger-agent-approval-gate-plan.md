# Stronger Agent Approval Gate Plan

## 요청 요약

- 사용자 요청: `AGENTS.md`의 승인 게이트를 더 강하게 수정하고, 문서 구조를 계획/기능/템플릿 방식으로 맞춘다.
- 목표: 향후 기능, 설정, 알고리즘, 운영 문서 변경 전에 설계안 보고와 사용자 승인을 강제한다.
- 제외 범위: CBGL 구현 파일, launch, YAML, Docker 설정 변경은 하지 않는다.

## 현재 상태 확인

- 확인한 파일/디렉터리: `AGENTS.md`, `docs/`
- 현재 문서 구조: 기존 저장소는 소문자 `docs/`를 사용한다.
- 관련 제약: 사용자 요청으로 `AGENTS.md` 변경은 승인된 범위로 간주한다.

## 영향 범위

- 파일: `AGENTS.md`
- 추가 문서 구조: `docs/plans/`, `docs/features/`, `docs/templates/task-plan.md`
- ROS topic/service/frame 영향: 없음
- Docker/launch/parameter 영향: 없음

## 구현 설계안

- `AGENTS.md`에 강제 승인 규칙, approval gate, 기능 개발 흐름, 문서화 규칙을 추가한다.
- 제공된 Fleet_sim 예시의 구조를 이 프로젝트의 ROS 1 Kinetic, CBGL, Docker, ros1_bridge 맥락에 맞게 바꾼다.
- 기존 저장소 관례에 맞춰 대문자 `Docs/` 대신 소문자 `docs/` 아래에 동일한 하위 구조를 만든다.

## 검증 계획

- `rg`로 서비스 선택 작업 흔적이 남지 않았는지 확인한다.
- `git diff --check`로 문서 변경의 공백 오류를 확인한다.
- 코드 변경이 아니므로 Docker 빌드는 수행하지 않는다.

## 문서화 계획

- 기능 결과 문서: `docs/features/2026-05-06_stronger-agent-approval-gate.md`
- 운영 문서 갱신: `AGENTS.md`

## 승인 상태

- 사용자 승인 문구: "Agent md파일 수정좀 해봐 좀 더 강하게"
- 승인 시각: 2026-05-06
