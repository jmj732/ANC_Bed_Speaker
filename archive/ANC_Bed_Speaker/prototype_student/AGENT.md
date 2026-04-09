# Agent Guide (EN/KR)

This file is a short guide for anyone working on this repo.
KR: 이 파일은 이 저장소를 작업할 때 참고하는 짧은 가이드입니다.

## What this project is
- EN: A simple FxLMS ANC prototype with secondary-path learning.
- KR: 2차 경로 학습을 포함한 FxLMS 기반 ANC 프로토타입.

## Quick commands
- Learn secondary path: `python main.py learn-secondary --duration 10 --mu 0.02`
- Run ANC control: `python main.py run --secondary secondary_path.npz`
- Record baseline: `python main.py baseline --duration 30`
- Measure PSD: `python tools/measure_psd.py --baseline baseline.npz --anc anc_run.npz`

## Folder map
- `main.py` : CLI entry point / 실행 엔트리
- `anc/` : core ANC logic / 핵심 로직
  - `anc/dsp.py` : DSP utilities, FIR, ring buffer / DSP 유틸리티, FIR, 링버퍼
  - `anc/secondary_path.py` : S_hat estimation / 2차 경로 추정
  - `anc/fxlms.py` : FxLMS controller & runtime / FxLMS 컨트롤러 & 실행
  - `anc/baseline.py` : baseline recorder / 기준 신호 기록
- `tools/` : analysis helpers / 분석 도구
  - `tools/measure_psd.py` : PSD comparison / PSD 비교
- `docs/` : documentation / 문서
  - `docs/GLOSSARY.md` : glossary / 용어집

## Notes
- `--limiter-drive` controls tanh limiter strength.
- Secondary path learning step size is `--mu` (default 0.02).

## Rules (EN/KR)
- EN: Keep changes small and explain why when behavior changes.
- KR: 변경은 작게, 동작이 바뀌면 이유를 설명.
- EN: Avoid heavy dependencies; prefer numpy/scipy only.
- KR: 무거운 의존성 추가 금지, numpy/scipy 우선.
- EN: Do not remove logging fields in `.npz` outputs.
- KR: `.npz` 출력 필드는 제거하지 말 것.
- EN: Keep realtime DSP in block callbacks; avoid slow Python in loops.
- KR: 실시간 DSP는 콜백 안에서 처리하고, 루프의 느린 Python 연산은 피하기.
- EN: If a change affects audio I/O, note the expected device/channel layout.
- KR: 오디오 I/O 변경 시 장치/채널 레이아웃을 명시.

## Coding style (EN/KR)
- EN: Keep files short; split into `anc/`, `tools/`, `docs/` as needed.
- KR: 파일은 짧게 유지하고 필요 시 폴더로 분리.
- EN: Use clear function names and short docstrings (EN + KR).
- KR: 함수명은 명확하게, docstring은 짧게(영문+국문).
- EN: Prefer `np.float32` for DSP arrays and avoid implicit dtype upcasts.
- KR: DSP 배열은 `np.float32` 우선, 암묵적 dtype 업캐스트 지양.
- EN: Keep per-sample loops minimal; use vector ops where possible.
- KR: 샘플 단위 루프는 최소화, 가능한 벡터 연산 사용.
