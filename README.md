<div align="center">

# 🔇 ANC-RT

**Raspberry Pi 5 기반 실시간 능동 소음 제거 (Active Noise Cancellation)**

[![Language](https://img.shields.io/badge/language-C99-blue?style=flat-square&logo=c)](https://en.wikipedia.org/wiki/C99)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-c51a4a?style=flat-square&logo=raspberry-pi)](https://www.raspberrypi.com/)
[![Audio](https://img.shields.io/badge/audio-HiFiBerry%20DAC%2BADC%20Pro-green?style=flat-square)](https://www.hifiberry.com/)
[![ALSA](https://img.shields.io/badge/ALSA-hw%3AsndrpihifiP-orange?style=flat-square)](https://alsa-project.org/)
[![License](https://img.shields.io/badge/license-MIT-yellow?style=flat-square)](LICENSE)

*FxLMS 광대역 · 협대역 피치추적 · VSS 적응 스텝 · NEON 가속*

</div>

---

## 📖 목차

- [개요](#-개요)
- [신호 흐름](#-신호-흐름)
- [하드웨어 요구사항](#-하드웨어-요구사항)
- [빠른 시작](#-빠른-시작)
- [모드 레퍼런스](#-모드-레퍼런스)
- [CLI 옵션](#-cli-옵션)
- [소스 구조](#-소스-구조)
- [알고리즘 개요](#-알고리즘-개요)
- [알려진 한계](#-알려진-한계)
- [문서](#-문서)

---

## 🎯 개요

ANC-RT는 Raspberry Pi 5 + HiFiBerry DAC+ADC Pro 위에서 동작하는 **C 단독 실시간 ANC**입니다.  
Python 없이, 단일 프로세스가 캡처 → 연산 → 재생을 **period 32~64 프레임(< 1ms)** 사이클로 처리합니다.

| 바이너리 | 용도 | 서비스 |
|----------|------|--------|
| `build/anc` | FxLMS 광대역 ANC (70~170 Hz) | `anc-rt.service` |
| `build/snore_anc` | 협대역 피치추적 ANC (60~250 Hz) | `anc-snore.service` |

---

## 🔀 신호 흐름

```
  외부 소음원
      │
      ▼
  ┌───────────┐        ┌─────────────────────────────┐
  │  REF MIC  │──────▶ │   FxLMS / NB-ANC (src/)     │
  │ (R ch)    │        │                              │
  └───────────┘        │  x(n) ──▶ W(z) ──▶ y(n)     │
                       │                    │          │
  ┌───────────┐        │  S_hat(z)          │          │
  │  ERR MIC  │──────▶ │  (sec_path.bin)   │          │
  │ (L ch)    │        │                    ▼          │
  └───────────┘        │  e(n) ◀──── 오차 마이크       │
        ▲              └──────────────┬───────────────┘
        │                             │
        │              anti-noise y(n)│
        │                             ▼
        │              ┌──────────────────────┐
        └──────────────│   스피커 (출력 채널)  │
                       └──────────────────────┘
```

> **feedforward 조건**: ref mic이 err mic보다 소음원에 가까워야 인과성이 성립합니다.  
> 현재 `ref_lead ≈ 0` — 물리 배치 개선이 필요합니다. ([자세히](#-알려진-한계))

---

## 🛠 하드웨어 요구사항

| 항목 | 사양 |
|------|------|
| **SBC** | Raspberry Pi 5 (aarch64) |
| **오디오 HAT** | HiFiBerry DAC+ADC Pro |
| **ALSA 장치** | `hw:sndrpihifiberry,0` |
| **샘플레이트** | 96 000 Hz |
| **채널** | 2ch 인터리브 (L = err mic, R = ref mic) |
| **OS** | Raspberry Pi OS (64-bit) |
| **의존성** | `libasound2-dev`, `gcc`, `make` |

```bash
# 의존성 설치
sudo apt install -y gcc make libasound2-dev
```

---

## ⚡ 빠른 시작

### 1. 빌드

```bash
cd /home/admin/anc-rt
make          # build/anc + build/snore_anc 생성
```

### 2. 이차 경로 측정 *(최초 1회 필수)*

```bash
cd /home/admin/anc-rt/runtime

# ⚠️  스피커 볼륨을 낮춘 상태에서 시작하세요 (화이트 노이즈 출력됨)
../build/anc measure
# → runtime/sec_path.bin 생성
```

### 3. ANC 실행

```bash
# 광대역 FxLMS
cd runtime && ../build/anc run

# 협대역 피치추적 (코골이용)
cd runtime && ../build/snore_anc run
```

### 4. systemd 서비스 등록

```bash
sudo cp systemd/anc-rt.service    /etc/systemd/system/
sudo cp systemd/anc-snore.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now anc-rt.service
```

---

## 🎛 모드 레퍼런스

```bash
./anc <mode> [options]
```

| 모드 | 설명 |
|------|------|
| `measure` | 화이트 노이즈로 이차 경로 추정 → `sec_path.bin` 저장 |
| `run` | FxLMS 광대역 ANC 실행 |
| `nb` | 협대역 피치추적 ANC 실행 |
| `passthrough` | 캡처 → 재생 에코백 (하드웨어 확인용) |
| `capture-bench` | 캡처 전용 I/O 타이밍 벤치마크 |
| `sim` | 오프라인 FxLMS 시뮬레이션 (`sec_path.bin` 기반) |
| `sim --sweep` | mu / leak / mu_n_max 파라미터 그리드 탐색 |

---

## 🔧 CLI 옵션

### 알고리즘

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--mu=F` | `0.010` | VSS base 스텝 사이즈 |
| `--leak=F` | `0.999999` | Leaky LMS 망각 계수 |
| `--mu-n-max=F` | `0.030` | FxNLMS 정규화 스텝 상한 |
| `--eps-abs=F` | `2e-4` | NLMS 분모 절대 하한 |
| `--eps-rel=F` | `0.010` | NLMS 분모 상대 계수 |
| `--band-lo=F` | `70.0` | 제어 대역 하한 (Hz) |
| `--band-hi=F` | `170.0` | 제어 대역 상한 (Hz) |
| `--mb-bands=N` | `1` | 멀티밴드 분할 수 (1~8) |
| `--tone-hz=F` | `0` | 단음 추적 주파수 (0 = 비활성) |

### ALSA / 타이밍

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--period=N` | `32` | ALSA period 크기 (프레임) |
| `--buffer-mult=N` | `12` | buffer = period × mult |
| `--start-fill=N` | `2` | 시작 전 prefill 횟수 |
| `--xrun-fill=N` | `2` | xrun 후 refill 횟수 |
| `--adapt-delay-ms=N` | `400` | 기준선 후 적응 시작 지연 (ms) |

### 협대역 ANC (nb 모드)

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--n-harm=N` | `4` | 추적할 고조파 수 |
| `--nb-mu=F` | `0.001` | 협대역 NLMS 스텝 사이즈 |
| `--nb-leak=F` | `0.9999` | 협대역 망각 계수 |

### 측정 모드

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--measure-secs=N` | `60` | 측정 지속 시간 (초) |
| `--measure-noise=F` | `0.2` | 화이트 노이즈 진폭 |
| `--capture-secs=N` | `10` | capture-bench 지속 시간 |

---

## 📁 소스 구조

```
anc-rt/
├── src/
│   ├── anc_defs.h       ← 전역 상수 · 런타임 설정 구조체
│   ├── anc_dsp.h        ← DSP 커널 (NEON 내적 · 순환버퍼 · 대역필터)
│   ├── anc_algo.h       ← FxLMS · 멀티밴드 · 협대역 피치추적 알고리즘
│   ├── anc_io.h         ← RT 메모리 설정 · ALSA 초기화 · xrun 복구
│   ├── anc_log.h        ← 런타임 로거 · 캡처벤치 · 측정 타이밍
│   ├── anc_measure.h    ← 이차 경로 측정 (Phase1 xcorr + Phase2 LMS)
│   ├── anc_run.c/h      ← 광대역 FxLMS 런타임 루프
│   ├── anc_nb.c/h       ← 협대역 피치추적 런타임 루프
│   ├── anc_pass.c/h     ← 패스스루 루프
│   ├── anc_sim.c/h      ← 오프라인 FxLMS 시뮬레이터
│   ├── anc_app.c/h      ← CLI 파서 · 모드 라우터
│   ├── anc_main.c       ← main() → anc 바이너리
│   ├── snore_anc.c      ← main() → snore_anc 바이너리 (파라미터 오버라이드)
│   ├── sim_compare.c    ← FxNLMS vs VSS vs 혼합 비교 시뮬레이터
│   └── anc.c            ← 단일 파일 프로토타입 (레거시 참조용)
├── runtime/
│   ├── sec_path.bin     ← 이차 경로 계수 (measure 후 생성)
│   └── snore_sec_path.bin
├── docs/
│   ├── ANC_RT_SETUP.md  ← 운영 설정 · 측정 결과
│   ├── AGENT.md         ← 코드 수정 원칙
│   └── daily/           ← 날짜별 작업 로그
├── systemd/             ← anc-rt.service · anc-snore.service
├── scripts/             ← IRQ affinity · 커널 튜닝
├── Makefile
└── README.md
```

---

## 🧠 알고리즘 개요

### FxLMS (Filtered-x Least Mean Squares)

```
          x(n) ─── S_hat(z) ───▶ x'(n)
            │                      │
            ▼                      │
         W(z) ──▶ y(n) ──▶ 스피커  │
                                    │
         e(n) ◀── err mic           │
            │                      │
            └── μ_n · e(n) · x'(n) ┘  ← 가중치 갱신
```

| 구성 요소 | 설명 |
|-----------|------|
| **Akhtar VSS** | `μ(n+1) = α·μ(n) + γ·e²` — 오차에 반응하는 적응 스텝 |
| **FxNLMS** | `μ_n = μ / (‖x'‖² · (1+ε_rel) + ε_abs)` — 전력 정규화 |
| **Leaky** | `w(n+1) = λ·w(n) - μ_n·∇` — 발산 억제 |
| **멀티밴드** | 대역을 로그 분할 후 밴드별 독립 FxLMS |

### 협대역 피치추적 (NB-ANC)

```
ref mic → [4× 데시메이션] → 정규화 교차상관 → f0 검출
                                                   │
f0 × [1, 2, …, N] ──▶ 고조파별 cos/sin 발진기     │
                              │                    │
                         W_c, W_s 갱신 ◀── e(n) ──┘
                              │
                    anti = Σ (W_c·cos + W_s·sin)
```

> f0 소실 시 하드 뮤트 대신 **200ms 지수 페이드아웃** 적용

### 실시간 루프 타이밍 (period=64, 96kHz)

```
period budget = 64 / 96000 = 0.667 ms

├─ write (play) ─────────────────── ~0.3 ms
├─ read  (cap)  ─────────────────── ~0.6 ms  ← 지배적
├─ compute (FxLMS 1024tap × 1band) ─ ~0.1 ms
└─ total avg ─────────────────────── 1.3 ms  ✓ < 2 ms
```

---

## ⚠️ 알려진 한계

> 현재 가장 큰 병목은 **코드가 아닌 물리 배치**입니다.

| 문제 | 원인 | 해결책 |
|------|------|--------|
| `ref_lead ≈ 0` | ref mic과 err mic이 소음원 등거리 | ref mic을 소음원 방향으로 이동 |
| ANC 감쇠 +1~5 dB 한계 | feedforward 인과성 조건 미충족 | err mic → 스피커 콘 5cm 이내 |
| 코골이 모드 무음 현상 | DSP mode 3 전환 후 ALSA lock | DSP mode 1 (Low latency IIR) 유지 |

**마이크 배치 권장:**
```
소음원
  │
  ├──[REF MIC]  ← 소음원 가까이
  │
  │    (거리차 = ref_lead > 0 이어야 함)
  │
  ├──[스피커]
  │
  └──[ERR MIC]  ← 스피커 콘 5cm 이내
```

---

## 📚 문서

| 파일 | 내용 |
|------|------|
| [`docs/ANC_RT_SETUP.md`](docs/ANC_RT_SETUP.md) | 운영 설정, 측정 결과, ALSA 파라미터 |
| [`docs/AGENT.md`](docs/AGENT.md) | 코드 수정 시 지켜야 할 구현 원칙 |
| [`docs/handoff/NEXT_SESSION_PROMPT.md`](docs/handoff/NEXT_SESSION_PROMPT.md) | 세션 인수인계 메모 |
| [`docs/daily/`](docs/daily/) | 날짜별 작업 로그 |

---

<div align="center">

*Raspberry Pi 5 · HiFiBerry DAC+ADC Pro · ALSA · C99 · NEON*

</div>
