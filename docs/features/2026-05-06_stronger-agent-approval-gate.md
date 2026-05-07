# Stronger Agent Approval Gate

## 사용자 요청

사용자는 이전 작업에서 승인 전에 구현 파일을 수정한 문제를 지적했고, 향후 같은 문제가 반복되지 않도록 `AGENTS.md`를 더 강하게 수정하라고 요청했다.

## 요구사항

- 기능, 설정, 알고리즘, 운영 문서 변경 전에 설계안 보고와 사용자 명시적 승인을 강제한다.
- 승인 전 구현 파일 생성, 수정, 삭제를 금지한다.
- 승인 전 예외는 분석과 `docs/plans/*.md` 계획 기록으로 제한한다.
- 제공된 개발 가이드의 `plans`, `features`, `templates` 문서 구조를 이 프로젝트에 맞게 반영한다.

## 구현 내용

- `AGENTS.md`를 ROS 1 Kinetic 기반 CBGL, Docker, `ros1_bridge` 프로젝트 맥락에 맞게 재작성했다.
- 강제 승인 규칙과 Approval Gate를 추가했다.
- 기능 개발 흐름에 `docs/plans/` 계획 기록과 `docs/features/` 결과 기록을 포함했다.
- 작업 계획 템플릿 `docs/templates/task-plan.md`를 추가했다.
- 이번 프로세스 변경 계획과 결과를 각각 `docs/plans/`와 `docs/features/`에 기록했다.

## 검증 결과

- 문서 변경만 수행했으므로 Docker 빌드는 수행하지 않았다.
- `AGENTS.md`에 승인 전 구현 금지 규칙이 포함되었다.
- `docs/plans/`, `docs/features/`, `docs/templates/` 구조가 생성되었다.

## 후속 작업

- 이후 기능 개발 요청부터는 `docs/plans/*.md` 작성, 사용자 승인, 구현, 검증, `docs/features/*.md` 기록 순서를 따른다.
