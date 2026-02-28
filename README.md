# Bedside ANC (Beginner Friendly)

This repository implements a simple FxLMS-based ANC prototype with secondary-path learning.
이 저장소는 2차 경로 학습을 포함한 FxLMS 기반 ANC 프로토타입입니다.

## Quickstart (EN)
1) Learn secondary path
```
python main.py learn-secondary --duration 10 --mu 0.02
```
2) Run ANC control
```
python main.py run --secondary secondary_path.npz
```
3) Record baseline (off)
```
python main.py baseline --duration 30
```

## 빠른 시작 (KR)
1) 2차 경로 학습
```
python main.py learn-secondary --duration 10 --mu 0.02
```
2) ANC 실행
```
python main.py run --secondary secondary_path.npz
```
3) 기준 신호 기록(OFF)
```
python main.py baseline --duration 30
```

## Folder guide (EN/KR)
- `anc/` : core ANC logic / ANC 핵심 로직
  - `dsp.py` : DSP utilities, FIR, ring buffer / DSP 유틸리티, FIR, 링버퍼
  - `secondary_path.py` : S_hat estimation / 2차 경로 추정
  - `fxlms.py` : FxLMS controller & runtime / FxLMS 컨트롤러 & 실행
  - `baseline.py` : baseline recorder / 기준 신호 기록
- `tools/` : analysis helpers / 분석 도구
  - `measure_psd.py` : PSD comparison / PSD 비교
- `docs/` : documentation / 문서
  - `GLOSSARY.md` : glossary / 용어집

## Notes
- Output limiter can be tuned with `--limiter-drive`.
- Secondary path learning step size is `--mu` (default 0.02).
