# Timestamped Doc Filenames

## 요청

- 같은 날짜에 생성된 `docs/plans/`, `docs/features/` 문서가 많아지면 파일명만으로 순서를 알기 어렵다.
- 새 문서 파일명에 시, 분까지 포함하도록 문서 작성 규칙을 변경한다.

## 변경 내용

- `AGENTS.md`
  - 문서 작성자 산출물 규칙을 `YYYY-MM-DD_HHMM_...` 형식으로 변경했다.
  - 작업 계획서 파일명 예시를 `docs/plans/YYYY-MM-DD_HHMM_feature-name-plan.md`로 변경했다.
  - 기능 결과 문서 파일명 예시를 `docs/features/YYYY-MM-DD_HHMM_feature-name.md`로 추가했다.
  - 기존 날짜-only 문서는 유지하고, 새 문서부터 timestamp 규칙을 적용한다고 명시했다.
- `docs/templates/task-plan.md`
  - 문서화 계획 항목에 timestamp 파일명 예시를 추가했다.
  - 승인 시각 예시를 `YYYY-MM-DD HH:MM`으로 명시했다.

## 새 규칙

```text
docs/plans/YYYY-MM-DD_HHMM_feature-name-plan.md
docs/features/YYYY-MM-DD_HHMM_feature-name.md
```

- `HHMM`은 로컬 작업 환경 기준 24시간 형식이다.
- 예: `docs/plans/2026-05-07_2039_timestamped-doc-filenames-plan.md`

## 검증

```bash
rg -n "YYYY-MM-DD_|YYYY-MM-DD_feature|docs/plans/YYYY-MM-DD|docs/features/YYYY-MM-DD|승인 시각" AGENTS.md docs/templates/task-plan.md docs/plans docs/features
git diff --check
```

- 결과: 새 규칙이 `AGENTS.md`, `docs/templates/task-plan.md`에 반영됐다.
- 기존 날짜-only 문서의 승인 시각 기록은 과거 기록으로 유지했다.

## 후속 작업

- 기존 문서 파일명은 링크 안정성을 위해 일괄 변경하지 않았다.
- 이후 새 plan/feature 문서는 timestamp 파일명 규칙을 따른다.
