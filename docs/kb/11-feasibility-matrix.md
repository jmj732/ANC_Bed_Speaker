# §11 실현가능성 매트릭스 — 본 시스템 제약 기준

> §06~§10에 정리된 기법을 본 시스템 제약과 교차 채점.
> 출력: **다음 실험 카드 우선순위** (§05 §D 결정트리의 데이터 입력).

---

## 제약 정의

| 제약 | 조건 | 차단 의미 |
|---|---|---|
| C1. ref_lead=0 | xcorr lag = 0 (§02 §6) | broadband feedforward 인과성 ❌ |
| C2. C 단일파일 + 표준 lib | ALSA + libm만 | DL/Python/외부 lib ❌ |
| C3. Period budget | 0.333 ms (period=32 @ 96kHz) | 샘플당 ≤ 0.01ms 권장 |
| C4. Pi5 ARM Cortex-A76 | FP cheap, no GPU/NPU | DL 추론 ❌ |
| C5. 메모리 | RAM 충분, no swap | 큰 buffer/lookup table 허용 |
| C6. Silent safety | f0 미검출 시 anti=0 (§04 #5) | 새 알고리즘은 silent baseline 필수 |
| C7. 측정 신뢰 | 다중 신호 평균 (§09 F1) | 단일 14s run 비교 무의미 |

---

## A. 매트릭스 (요약)

채점: ★★★ (즉시 시도) / ★★ (조건부) / ★ (실험적) / ✗ (배제)

| # | 기법 | C1 ref_lead | C2 C-only | C3 budget | C4 ARM | Tier | 이득 dB | 점수 |
|---|---|---|---|---|---|---|---|---|
| **§06 적응 알고리즘** | | | | | | | | |
| 06-A2 | NLMS (현 사용) | ✓ | ✓ | ✓ | ✓ | 1 | baseline | (사용 중) |
| 06-B1 | Kwong VSS-LMS | ✓ | ✓ | ✓ | ✓ | 2 | dip -2dB | ★★★ |
| 06-B2 | NPVSS-LMS | ✓ | ✓ | ✓ | ✓ | 2 | dip -2dB | ★★★ |
| 06-C1 | APA (broadband) | ✗ | ✓ | ⚠️ | ✓ | 2 | broadband +3 | ★★ (광대역 부활 후) |
| 06-C3 | RLS | ✗ | ✓ | ✗ N>10 | ✓ | 3 | cosmetic | ✗ |
| 06-D1 | SM-NLMS | ✓ | ✓ | ✓ | ✓ | 2 | compute ↓ | ★★ |
| 06-D2 | Sign-LMS | ✓ | ✓ | ✓ | ⚠️ | 2 | benefit 0 | ✗ |
| 06-G3 | Modified FxLMS | ✓ | ✓ | ✓ | ✓ | 2 | 미미 | ★ |
| 06-G5 | Online sec_path | ✓ | ✓ | ⚠️ | ✓ | 3 | 환경 변화 robust | ★★ |
| 06-H1 | Weight cap (현) | ✓ | ✓ | ✓ | ✓ | 1 | (사용 중) | — |
| **§07 인과성 우회** | | | | | | | | |
| 07-A1 | **Feedback ANC (IMC)** | ✓ 해결 | ✓ | ✓ | ✓ | 3 | +5~15 | ★★★ |
| 07-A2 | Hybrid FF+FB | ✓ 해결 | ✓ | ✓ | ✓ | 3 | A1+2~5 | ★★★ (A1 후) |
| 07-B1 | Linear Predictor | ⚠️ | ✓ | ✓ | ✓ | 2 | 협대역 0 | ★ |
| 07-B2 | ALE | ⚠️ | ✓ | ✓ | ✓ | 2 | 미미 | ★ |
| 07-C1 | ILC | ⚠️ | ✓ | ✓ | ✓ | 3 | burst 변동 시 0 | ★ |
| 07-C2 | Repetitive Control | ⚠️ | ✓ | ✓ | ✓ | 3 | 부분 | ★★ |
| 07-C3 | PLL Reference | ⚠️ | ✓ | ✓ | ✓ | 2 | jitter ↓ | ★★ |
| 07-D2 | Kalman 외란 | ⚠️ | ✓ | ✓ | ✓ | 3 | dip ↓ | ★★ |
| 07-E1 | Virtual Sensing | ✗ HW | ✓ | ✓ | ✓ | 4 | — | ✗ (HW 필요) |
| 07-F1 | **ref mic 재배치** | ✓ 완전 | — | — | — | 4 | +10~15 | ★★★ (최우선) |
| 07-F2 | sec_path 단축 | ⚠️ | — | — | — | 4 | 부분 | ★★ |
| **§08 피치 추적** | | | | | | | | |
| 08-A1 | ACF (현 사용) | ✓ | ✓ | ✓ | ✓ | 1 | baseline | (사용 중) |
| 08-A2 | **YIN** | ✓ | ✓ | ✓ | ✓ | 2 | 슬립 -50% | ★★★ |
| 08-A3 | pYIN | ✓ | ✓ | ⚠️ HMM | ✓ | 3 | — | ★ |
| 08-B1 | Cepstrum | ✓ | ✓ | ✗ FFT | ✓ | 3 | 등급 | ✗ |
| 08-B2 | HPS | ✓ | ✓ | ⚠️ FFT | ✓ | 3 | 슬립 -90% | ★★ |
| 08-B5 | Sliding DFT | ✓ | ✓ | ⚠️ | ✓ | 3 | 재설계 | ★★ |
| 08-C1 | **IIR ANF** | ✓ | ✓ | ✓ | ✓ | 2 | 저주파 검출 | ★★★ |
| 08-D1 | **Kalman f0 smoother** | ✓ | ✓ | ✓ | ✓ | 2 | dip ↓ | ★★★ |
| 08-D2 | Particle Filter | ✓ | ✓ | ✗ heavy | ✓ | 3 | — | ✗ |
| 08-E1 | Median 후처리 | ✓ | ✓ | ✓ | ✓ | 1 | jitter ↓ | ★★★ |
| 08-E2 | Hysteresis | ✓ | ✓ | ✓ | ✓ | 1 | (부분 사용) | ★★ |
| 08-E3 | Conf-weighted | ✓ | ✓ | ✓ | ✓ | 1 | 미세 | ★★ |
| 08-G3 | CREPE (CNN) | ✓ | ✗ | ✗ | ✗ | 4 | — | ✗ |
| **§09 코골이 특화** | | | | | | | | |
| 09-A1 | f0_active (현) | ✓ | ✓ | ✓ | ✓ | 1 | (사용 중) | — |
| 09-A2 | **Snore VAD** | ✓ | ✓ | ✓ | ✓ | 2 | false adapt ↓ | ★★★ |
| 09-A3 | Burst Onset | ✓ | ✓ | ✓ | ✓ | 2 | dip ↓ | ★★★ |
| 09-A4 | Burst Offset | ✓ | ✓ | ✓ | ✓ | 2 | 잔향 ↓ | ★★ |
| 09-B1 | **Phase-Aware Mu** | ✓ | ✓ | ✓ | ✓ | 2 | dip -2dB | ★★★ |
| 09-B2 | Inter-Burst Weight | ✓ | ✓ | ✓ | ✓ | 2 | onset ↓ | ★★ |
| 09-B3 | Phase rotation | ✓ | ✓ | ✓ | ✓ | 2 | (§04 #8 폐기) | ✗ 재시도 시 조건부 |
| 09-C2 | **Dynamic mu_scale** | ✓ | ✓ | ✓ | ✓ | 3 | n_harm 실효화 +1~2dB | ★★★ |
| 09-C3 | 독립 톤 추적 | ✓ | ✓ | ✓ | ✓ | 3 | best +2~4dB | ★★★ |
| 09-C4 | Formant-aware | ✓ | ✓ | ⚠️ | ✓ | 3 | — | ★★ |
| 09-C5 | Pitch ramp 보상 | ✓ | ✓ | ✓ | ✓ | 3 | 미세 | ★★ |
| 09-D3 | Snore Template | ✓ | ✓ | ✓ | ✓ | 3 | 일반화 ✗ | ✗ |
| 09-D4 | HMM Snore Model | ✓ | ✓ | ⚠️ | ✓ | 3 | — | ★ |
| 09-E2 | Watchdog Reset | ✓ | ✓ | ✓ | ✓ | 2 | 안정성 | ★★ |
| 09-F1 | **다중 신호 평균 측정** | — | ✓ | — | — | 1 | 측정 신뢰 | ★★★ |
| 09-F2 | Per-Burst Analysis | — | ✓ | — | — | 1 | 측정 신뢰 | ★★★ |
| **§10 주파수/멀티레이트/비선형** | | | | | | | | |
| 10-A1 | FDAF | ⚠️ | ✓ | ⚠️ | ✓ | 3 | (broadband 부활 후) | ★ |
| 10-A3 | PBFDAF | ⚠️ | ✓ | ✓ | ✓ | 3 | broadband 표준 | ★★ |
| 10-B1 | Subband FxLMS | ⚠️ 일부 | ✓ | ⚠️ | ✓ | 3 | broadband +3~7 | ★★ |
| 10-D1 | Volterra FxLMS | ✓ | ✓ | ⚠️ | ✓ | 3 | 비선형 +0.5~2 | ★ |
| 10-D2 | FsLMS | ✓ | ✓ | ✓ | ✓ | 3 | — | ★ |
| 10-D4 | NN ANC | ✓ | ✗ | ✗ | ✗ | 4 | — | ✗ |
| 10-F1 | DXHS | ✓ | ✓ | ✓ | ✓ | 2 | cosmetic | ★ |
| 10-F3 | **Recursive Cos-Sin** | ✓ | ✓ | ✓ | ✓ | 1 | compute ↓ | ★★★ |
| 10-G1 | MPC ANC | ✓ | ⚠️ QP | ⚠️ | ✓ | 3 | — | ★ |

---

## B. 즉시 시도 가능 (★★★) 정렬 — Top 15

| 순위 | ID | 기법 | Tier | 영역 | 예상 이득 | 차단 게이트 |
|---|---|---|---|---|---|---|
| 1 | 07-F1 | **ref mic 재배치 (물리)** | 4 | broadband 천장 깸 | +10~15 dB | 물리 작업 |
| 2 | 07-A1 | **Feedback ANC (IMC)** | 3 | broadband 천장 깸 (SW) | +5~15 dB | sec_path 정확도, 안정성 가드 |
| 3 | 09-C3 | **독립 톤 추적 (multi-tonal)** | 3 | best 천장 깸 | +2~4 dB | SNR 추정 인프라 |
| 4 | 09-F1+F2 | **다중 신호 평균 + per-burst 측정** | 1 | 측정 신뢰 회복 | 튜닝 효율 ↑ | 측정 변경만 |
| 5 | 09-B1 | **Phase-Aware Mu Schedule** | 2 | dip 단축 | dip -2 dB | onset 정확도 |
| 6 | 09-A2 | **Snore VAD** | 2 | false adapt 제거 | 안정성 +0.5 | feature 튜닝 |
| 7 | 09-A3 | **Burst Onset Detection** | 2 | onset 빠른 수렴 | dip ↓ | onset 오검출 시 발산 |
| 8 | 09-C2 | **Dynamic mu_scale (SNR)** | 3 | n_harm=2 실효화 | +1~2 dB | SNR 추정 |
| 9 | 08-D1 | **Kalman f0 smoother** | 2 | f0 jitter, dip | dip -1.5 dB | Q/R 튜닝 |
| 10 | 08-C1 | **IIR ANF f0 tracker** | 2 | 저주파 검출, 옥타브 슬립 | best +1~2 | 수렴 속도 |
| 11 | 08-A2 | **YIN** | 2 | 옥타브 슬립 | 슬립 -50% | threshold 튜닝 |
| 12 | 08-E1+E2+E3 | **f0 후처리 패키지** | 1 | jitter | ±0.5 dB | 거의 즉시 |
| 13 | 06-B1/B2 | **VSS-LMS (Kwong / NPVSS)** | 2 | burst 수렴 | dip -2 dB | μ_max clamp |
| 14 | 06-D1 | **SM-NLMS** | 2 | compute 절감 | 다른 곳 budget | 없음 |
| 15 | 10-F3 | **Recursive cos-sin** | 1 | compute ↓ | 미세 | numerical drift |

---

## C. 시너지 그래프 (Combos)

| 조합 | 효과 | 주의 |
|---|---|---|
| 07-A1 + 07-A2 (FB → Hybrid) | broadband 천장 → 추가 +2~5dB | A1 안정 후 |
| 09-A2 (VAD) + 06-B1 (VSS) + 09-B1 (Phase-Aware Mu) | burst-aware 적응 패키지 | onset 정확도 게이트 |
| 08-D1 (Kalman) + 08-A2 (YIN) | f0 robust + smooth | 별 conflict 없음 |
| 09-F1+F2 (측정) → 모든 튜닝 | 모든 후속 실험 신뢰도 ↑ | **선행 필수** |
| 09-C2 (dyn mu_scale) + 09-C3 (multi-tonal) | n_harm 본격 활용 | 발산 안전 가드 필수 |

---

## D. 권장 로드맵 (Phase 분할)

### Phase 0 — 측정 신뢰 회복 (Tier 1, 0.5일)
- 09-F1: 다중 신호 평균 (mollyroselee + 28개)
- 09-F2: per-burst dB 분포 logger
- 10-F3: recursive cos-sin (compute 마진 확보, 옵션)
- **검증**: §05 §C 게이트 #3, #4

### Phase 1 — 즉시 SW 개선 (Tier 1~2, 1~2일)
- 08-E1+E2+E3: f0 후처리 (median, hysteresis, conf-weight)
- 08-D1: Kalman f0 smoother
- 09-A2+A3+A4: snore VAD + onset/offset
- 09-B1: phase-aware mu schedule
- 06-B1 or B2: VSS-LMS
- **예상 누적 이득**: dip -3 → -1dB, best +5 → +6~7dB
- **검증**: §05 §C 모든 게이트

### Phase 2 — 천장 깨기 SW (Tier 3, 3~5일)
- 09-C2+C3: dynamic mu_scale + multi-tonal
- 08-C1: IIR ANF (저주파 50~80Hz 커버)
- **예상 이득**: best +6~7 → +8~10dB

### Phase 3 — 천장 깨기 본 카드 (Tier 3~4)
- **선택 A (SW only)**: 07-A1 Feedback ANC + 07-A2 Hybrid → +10~15dB 광대역
- **선택 B (물리)**: 07-F1 ref mic 재배치 → +10~15dB 광대역, broadband 부활
- **권장**: B (작업 비용 ≤ A의 안정성 분석 비용, 효과 확실)

### Phase 4 — broadband 부활 후 (B 선택 시)
- 10-A3 PBFDAF
- 10-B1 Subband FxLMS
- 06-C1 APA
- **예상 이득**: broadband +5~10dB 추가

---

## E. ✗ 배제 사유 명시

| 기법 | 배제 사유 | 재검토 조건 |
|---|---|---|
| 06-C3 RLS | budget 위반 | broadband 부활 + N 작을 때 |
| 06-D2 Sign-LMS | ARM에서 이득 없음 | 영구 배제 |
| 06-D4 PNLMS | NB-FxLMS 무관 | 영구 배제 |
| 07-E1 Virtual Sensing | HW 추가 | mic ≥3 추가 시 |
| 08-G3 CREPE / 10-D4 NN ANC | C-only 위배 | 영구 배제 (또는 별도 추론 가속기) |
| 09-D3 Snore Template | 일반화 ✗ | 사용자 캘리브레이션 모드 도입 시 |
| 09-B3 Phase Rotation | §04 #8 폐기 | scale clamp 제거 후 단독 실험 시 |
| 10-D3 Bilinear | sec_path FIR로 충분 | 영구 배제 |
| 10-D4 NN ANC | C-only + budget | 영구 배제 |
| 10-G1 MPC ANC | QP solver 필요 | 영구 배제 |

---

## F. 다음 세션 액션 (Cheat Sheet)

```
1) 측정부터: 09-F1 + 09-F2 → 다중 신호 평균, per-burst 분포
2) 그 다음: 08-E (f0 후처리) — Tier 1, 1 commit
3) 그 다음: 09-A2+A3 (snore VAD + onset) — Tier 2, 별도 commit
4) 그 다음: 09-B1 (phase-aware mu) — Tier 2
5) 그 다음: 08-D1 (Kalman) — Tier 2
6) 천장 깨기 선택:
   - 물리: 07-F1 ref mic 재배치 (작업 의뢰)
   - SW: 07-A1 Feedback ANC (별 모드 추가)
7) broadband 부활 후: 10-A3 / 10-B1
```

**최종 KB 활용 흐름**: §11 §F → §05 결정트리 통과 → §04 실패 카탈로그 회피 → §01~§03 이론/신호로 정당화 → 구현.
