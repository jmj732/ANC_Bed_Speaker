# §18 코골이 임상 / 음향 발생 모델

코골이의 물리적 생성 메커니즘과 임상 분류. 알고리즘 가정의 타당성 검증과 새 카드 발굴의 근거.

---

## 18.1 코골이 생성 메커니즘 (생체역학)

**원리**: 상기도(연구개·인두벽·혀뿌리·후두덮개) 부분 폐쇄 → 공기 흐름이 Bernoulli 효과로 조직 흡인 → 진동 → flutter (자려진동).

**진동 위치별 분류 (Pringle & Croft 1993; VOTE classification 2011)**:
| 위치 | 영문 | 주파수 대역 (Hz) | 특징 |
|---|---|---|---|
| Velum (연구개) | V | 70~250 | 가장 일반적, 저주파 강함 |
| Oropharynx (인두벽) | O | 150~400 | 중주파, 측벽 진동 |
| Tongue base (혀뿌리) | T | 100~300 | OSA 환자 우세 |
| Epiglottis (후두덮개) | E | 200~800 | 고주파, 드물지만 격함 |

**시사**: 본 시스템 분석 데이터 median 70Hz / spectrogram dominant 434Hz 패턴은 **V+E 또는 T+E 혼합** 가능성 큼. 단일 harmonic ANC가 한계인 이유.

**참고**: Kezirian, E.J. et al. (2011). "Drug-induced sleep endoscopy: the VOTE classification." *Eur Arch Otorhinolaryngol* 268, 1233.

---

## 18.2 코골이 음향 특성

**시간 구조**:
- Burst (1~2s) — 흡기 시 진동
- Silence (1~3s) — 호기/일시 정지
- 호흡 주기 약 3~5s (15~20 BPM)

**주파수 구조**:
- Fundamental (f0): 50~250Hz (대부분 100~150Hz)
- Harmonics: f0의 2~10배까지 강한 에너지
- Formant-like resonance: 800~1500Hz 영역 envelope peak
- High-freq hiss: 2~5kHz 작은 에너지

**진폭**: 60~90 dB SPL (1m 거리). 정상 호흡 (30~40 dB) 대비 +30~50 dB.

**참고**: Pevernagie, D. et al. (2010). "The acoustics of snoring." *Sleep Med Rev* 14, 131.

---

## 18.3 자세별 / 시간별 변화

**자세**:
- 앙와위 (supine): f0 ↓, intensity ↑, 빈도 ↑
- 측와위 (lateral): f0 ↑, intensity ↓
- 복와위 (prone): 코골이 최소

**시간**:
- REM 수면: 빈도 ↓, intensity 변동 ↑
- NREM 3 (deep): 빈도 ↑, intensity 일정
- 음주/약물 후: f0 ↓, intensity ↑

**시사**: ANC는 자세 변화 시 sec_path 변화도 발생 → online sec_path 식별 (§06 §6.7) 필요성 ↑.

---

## 18.4 단순 코골이 vs OSA 동반 코골이

**단순 코골이 (primary snoring)**:
- 규칙적, f0 안정 (변동 < 30%)
- 흡기 시만 발생
- 단일 진동원 (V 또는 O)

**OSA 동반 코골이**:
- 불규칙, f0 변동 큼 (50%+)
- 호기 시에도 발생
- 다중 진동원, multi-tonal
- Apnea (10s 이상 호흡 정지) 후 큰 burst (resumption snore)

**시사**: 본 시스템 분석 데이터의 std=90Hz / 큰 변동은 OSA 가능성. **다중 진동원 ANC** 필요.

---

## 18.5 음향 발생 모델 (수학)

**Helmholtz resonator 모델**:
```
f0 ≈ (v/2π) · sqrt(A / (V·L))
```
- v: 음속, A: 개구 면적, V: 공동 부피, L: 개구 길이
- 상기도 V 변화 (수면 자세) → f0 변화

**Self-excited oscillation 모델 (collapsible tube)**:
- Reynolds수 ~ 2000~5000 (난류)
- Strouhal수 ~ 0.2 → flutter 주파수
- Bernoulli + 탄성 복원력

**시사**: f0는 quasi-periodic이지만 진폭은 chaotic 가능. Lyapunov exponent > 0 보고 (Hadjileontiadis 2009).

---

## 18.6 ANC 알고리즘 가정 vs 임상 현실

| 알고리즘 가정 | 임상 현실 | 정합? |
|---|---|---|
| f0 quasi-stationary (50ms) | OSA에선 burst 내 변동도 큼 | ⚠️ |
| harmonic structure | V/T/E 혼합 시 inharmonic | ⚠️ |
| 단일 source location | V+E 동시 진동 흔함 | ❌ |
| Gaussian noise model | chaotic, heavy-tailed | ❌ |
| sec_path stationary | 자세 변화 시 변화 | ⚠️ |

**시사**: 본 시스템 +5dB 천장의 본질적 원인은 **알고리즘 모델 단순화** vs 코골이 복잡도 미스매치.

---

## 18.7 임상 측정 표준

**Apnea-Hypopnea Index (AHI)**: 시간당 호흡 정지 횟수. 5+ 경증 OSA, 30+ 중증.

**Snore Index (SI)**: 시간당 코골이 sound burst 횟수. >30/h 비정상.

**Loudness**: dB SPL, A-weighted, 1m 거리.

**시사**: 본 프로젝트의 success metric은 dB(A) reduction at user ear position이 적절. err_rms는 mic 위치 한정.

---

## 18.8 알고리즘 카드 영감

| 임상 사실 | 알고리즘 카드 |
|---|---|
| 진동원 다중 (V+T+E) | 독립 톤 추적 (§09 §9.6), Sparse OMP (§17.4) |
| f0 burst 내 변동 | Non-stationary chirp (§17.8), KF rate 추정 (§08 §8.4) |
| 호기 정지 (silence) | VAD (§09 §9.2), HMM scheduler (§14.4) |
| 자세 변화 sec_path | Online sec_path (§06 §6.7) |
| Multi-tonal inharmonic | 독립 톤, Sparse, n_harm 확장 |
| OSA resumption burst | Burst onset detection (§09 §9.4) |

---

## 18.9 코골이 데이터셋 (학술 reference)

| 데이터셋 | 출처 | 용도 |
|---|---|---|
| MIT-BIH PSG | physionet.org | OSA 분류 |
| St Vincent's HSP | physionet.org | 다채널 PSG |
| Munich Snore | TUM | 청각 분류 |
| 본 프로젝트 데이터 | freesound.org 29파일 | 알고리즘 검증 |

**시사**: 본 프로젝트 데이터는 mixed quality, single channel 48kHz mp3. 학습용으로는 부족. 알고리즘 검증에는 충분.

---

## 18.10 우선순위 시사

1. **Multi-tonal 필요성 확립**: §09 §9.6 독립 톤 추적은 임상적으로도 타당. Phase 2 카드 강력 추천.
2. **VAD/HMM 필요성**: 호기 정지 활용 → adapt schedule. Phase 1~2.
3. **Online sec_path**: 자세 변화 대응. Phase 3.
4. **목표 메트릭 재정의**: err_rms → dB(A) at virtual user ear. §15.2 virtual mic + §19 metrics.

---

## 18.11 참고문헌

- Pringle, M.B. & Croft, C.B. (1993). "A grading system for patients with obstructive sleep apnea." *Clin Otolaryngol* 18, 480.
- Kezirian, E.J. et al. (2011). "Drug-induced sleep endoscopy: VOTE classification." *Eur Arch Otorhinolaryngol* 268, 1233.
- Pevernagie, D. et al. (2010). "The acoustics of snoring." *Sleep Med Rev* 14, 131.
- Hadjileontiadis, L.J. (2009). "A novel technique for denoising explosive lung sounds..." *IEEE EMB* 28, 41.
- Karunajeewa, A.S. et al. (2011). "Snoring sound analysis." *Physiol Meas* 32, 1.
