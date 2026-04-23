# Next Session Prompt

```text
너는 Raspberry Pi 5 + HiFiBerry DAC+ADC Pro 환경에서 C 기반 실시간 ANC/오디오 I/O를 튜닝하는 DSP/시스템 엔지니어다.

이 프로젝트에는 두 개의 독립된 ANC 바이너리가 있다:
- `/home/admin/anc.c` → `./anc` (FxLMS broadband ANC, 서비스: `anc-rt.service`)
- `/home/admin/anc-rt/src/anc.c` → `snore_anc` (NB narrowband ANC, 서비스: `anc-snore.service`)

우선 읽을 파일:
- `/home/admin/anc-rt/docs/daily/2026-04-15.md`
- `/home/admin/NEXT_SESSION_PROMPT.md` (이 파일)

---

## anc (FxLMS broadband) 현재 상태

서비스: `anc-rt.service`
운영값: `period=64 buffer=768 start-fill=2 xrun-fill=3 mb-bands=1 adapt-delay-ms=400`

### 서비스 ExecStart
```
ExecStartPre=amixer cset "name=Analogue Playback Boost Volume" "1,1"
ExecStartPre=amixer cset "name=DSP Program" 1    ← Low latency IIR
ExecStart=./anc run --period=64 --buffer-mult=12 --start-fill=2 --xrun-fill=3 --mb-bands=1 --adapt-delay-ms=400
```

### 코드 변경 이력 (2026-04-15)
- `set_sw_params`: playback `start_threshold = buf` (auto-start 방지)
- `rt_prepare_memory`: `SCHED_FIFO` priority 49 추가
- `#include <sched.h>` 추가

### sec_path.bin 현재값
- peak = 192 samples (4.00ms), Low latency IIR (DSP mode 1), period=64

### 달성된 목표
- SW operational latency < 2ms: total avg 1.333ms, max 1.466ms ✓
- start-fill=2, xrun-fill=3에서 895초 xr=0 ✓

### 미완 실험
- period=32: passthrough/run 29s xr=0 확인, soak test 미완, 서비스 배포 보류
  - period=32 시 start-fill=3 필요 (sf=2로는 startup xrun)
  - sec_path: 129 samples (2.69ms)
  - compute avg: 0.153ms

### 핵심 병목 (코드로 해결 불가)
- ref mic lead = 0 → feedforward 인과성 조건 미충족
- ANC reduction +1~5dB에 머무는 근본 원인
- 해결책: 물리 배치 변경 필요
  1. error mic → 스피커 콘 5cm 이내
  2. ref mic → noise source 방향

---

## snore_anc (NB narrowband) 현재 상태

운영값: `period=32 buffer-mult=4 start-fill=1 xrun-fill=2 n-harm=4 nb-mu=0.0005 nb-leak=0.9999`

### 미해결 문제
- 2세션에서 DSP mode 3 시도 후 스피커 무음 발생
- PCM5122 내부 레지스터 오염 추정
- **재부팅으로 cold reset 필요**

---

## 다음 세션 우선순위

1. 재부팅 후 스피커 정상 복구 확인 (`anc-snore.service` 재시작)
2. 배치 변경 여부 확인 (error mic / ref mic 이동)
3. 배치 변경 시: `./anc measure` → sec_path 재측정 → 효과 정량 확인
4. period=32 soak test (start-fill=3, 5분 이상)

## 절대 하지 말 것
- DSP Program을 mode 3으로 바꾸지 말 것 (스피커 무음 재발)
- start-fill을 period=64에서 3으로 되돌리지 말 것 (퇴보)
- sec_path.bin을 측정 없이 교체하지 말 것

## 문서 규칙
- 작업한 날: `docs/daily/YYYY-MM-DD.md`에 날짜별 작업 로그
- 코드 변경, 서비스 변경, 측정 명령, 결과 숫자, 남은 리스크 기록
```
