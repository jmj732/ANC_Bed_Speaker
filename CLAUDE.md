# ANC Bed Speaker — Claude Code 컨텍스트

## 지식 그래프
작업 전 `graphify-out/GRAPH_REPORT.md`를 참조하세요.
- **God Nodes**: `DspConfig` (100 edges), `RingBuffer` (65), `FIRFilter` (53), `FxLMSController` (35), `run_anc()` (33)
- 그래프는 `src/` 파일 변경 시 자동 재생성됩니다

## 프로젝트 구조
- `src/anc.c` — 진입점
- `src/anc_run.c` — broadband FxLMS ANC
- `src/anc_nb.c` — narrowband pitch-tracking ANC (주 전략)
- `src/anc_measure.c` — secondary path 측정
- `src/anc_app.c` — 앱 레벨 로직
- `graphify-out/GRAPH_REPORT.md` — 코드 지식 그래프

## 현재 상태 (2026-04-20 기준)
- **narrowband(nb) 모드**가 주 전략 — ref_lead=0에서도 동작
- **broadband FxLMS**는 ref_lead=0 한계로 보류
- secondary path 최소 latency: period=32 → 2.96ms

## 핵심 원칙
- 한 번에 하나씩 변경 (발산 시 원인 분리 위해)
- C 단일파일 + ALSA + 표준 C 라이브러리만 사용
- Python/외부 분석 툴 전제 금지
- 테스트 전 스피커 소음 방지 안전장치 확인 필수

## 빌드
```bash
gcc -O3 -mcpu=cortex-a76 -DOUTPUT_SAFETY_LPF_HZ=800.0f -o anc src/anc.c -lasound -lm
# 또는
make
```
