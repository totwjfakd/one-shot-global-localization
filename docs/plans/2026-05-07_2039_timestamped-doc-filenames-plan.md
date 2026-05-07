# Timestamped Doc Filenames Plan

## 요청 요약

- 사용자 요청: 같은 날짜에 생성된 문서들의 순서를 알기 어렵기 때문에 문서 생성 규칙에 시, 분까지 포함한다.
- 목표: `docs/plans/`, `docs/features/` 문서 파일명 규칙을 `YYYY-MM-DD_HHMM_...` 형식으로 변경한다.
- 제외 범위: 기존 문서 파일명 일괄 rename은 이번 변경에 포함하지 않는다.

## 현재 상태 확인

- 확인한 파일/토픽/서비스:
  - `AGENTS.md`
  - `docs/templates/task-plan.md`
- 관측 요약:
  - 현재 규칙은 `docs/plans/YYYY-MM-DD_feature-name-plan.md`
  - 현재 규칙은 `docs/features/YYYY-MM-DD_feature-name.md`
  - 같은 날짜에 여러 문서가 생기면 파일명만으로 작성 순서를 알기 어렵다.
- 관련 제약:
  - 운영 문서 수정은 사용자 명시 요청 범위 안에서만 수행한다.

## 영향 범위

- 패키지: 없음
- 파일 후보:
  - `AGENTS.md`
  - `docs/templates/task-plan.md`
  - `docs/features/2026-05-07_2039_timestamped-doc-filenames.md`
- ROS topic/service/frame: 영향 없음
- Docker/launch/parameter 영향: 영향 없음

## 구현 설계안

- 변경 방식:
  - 새 plan 문서: `docs/plans/YYYY-MM-DD_HHMM_feature-name-plan.md`
  - 새 feature 문서: `docs/features/YYYY-MM-DD_HHMM_feature-name.md`
  - 시간은 로컬 작업 환경 기준 24시간 `HHMM` 형식을 사용한다.
  - 기존 문서는 링크 안정성을 위해 유지하고, 새 문서부터 새 규칙을 적용한다.
- 대안:
  - 문서 본문에만 승인 시각을 기록한다.
  - 파일명에 초까지 포함한다.
- 선택 이유:
  - 파일명 정렬만으로 같은 날짜 문서 순서를 파악할 수 있다.
  - 초까지 넣으면 이름이 과하게 길어지므로 분 단위가 적절하다.
- 리스크:
  - 기존 날짜-only 문서와 새 timestamp 문서가 한동안 섞여 보일 수 있다.

## 검증 계획

- 빌드: 문서/운영 규칙 변경만 수행하므로 Docker 빌드는 생략한다.
- 실행/서비스 호출: 해당 없음.
- topic/frame 확인: 해당 없음.
- 실패 시 확인할 항목:
  - `rg`로 기존 날짜-only 템플릿 문구가 남아 있는지 확인한다.
  - `git diff --check`로 문서 공백 오류를 확인한다.

## 문서화 계획

- 기능 결과 문서: `docs/features/2026-05-07_2039_timestamped-doc-filenames.md`
- 운영 문서 갱신: `AGENTS.md`, `docs/templates/task-plan.md`

## 승인 상태

- 사용자 승인 문구: "지금 같은 날짜 문서들의 경우 순서를 모르겠는데, 시, 분까지 나타내게 문서 만드는 규칙 변경좀 해줘"
- 승인 시각: 2026-05-07 20:39
