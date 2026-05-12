# §19 검증 / 측정 메트릭 / Calibration

알고리즘 변경 채택 기준의 통계적 근거. §04 #10 (14초 단일 run 무의미) 재발 방지.

---

## 19.1 현재 메트릭 (한계)

| 메트릭 | 의미 | 한계 |
|---|---|---|
| `err_rms` | error mic의 RMS | 단일 마이크, 위치 의존 |
| `best_db` | adapt 중 peak attenuation | burst 정렬 의존, 1회 발생만 봄 |
| `r = err / baseline` | 즉시 비율 | 단일 1초 윈도우 |
| `xrun` | underrun 카운트 | 안정성만, dB 무관 |
| `tone_db` | tone band 한정 dB | band 정의 의존 |

**문제**: 모두 **단일 run의 단일 윈도우**에서 측정. 통계 검정 없음.

---

## 19.2 통계적으로 신뢰 가능한 메트릭

### 19.2.1 Per-burst dB distribution
- baseline (adapt OFF) 1회 측정
- adapt ON, burst N개 처리, **각 burst별 dB 계산**
- 출력: mean, median, p10, p90, std

### 19.2.2 Per-frame (50~100ms) dB
- 0.1s window dB 시계열
- 출력: histogram, percentile

### 19.2.3 Cumulative reduction
- 전체 측정 시간의 ∫err² / ∫baseline² (-dB)
- 단일 통합 지표

**판정**: 모든 변경 평가는 **§19.2.1 (per-burst) + §19.2.3 (cumulative)** 두 가지로.

---

## 19.3 통계적 검정

**문제**: A vs B 알고리즘, dB 차이가 진짜 개선인가 noise인가?

**검정**:
- **Paired t-test** (burst별 dB 짝지어): N=20+ burst, p<0.05 요구
- **Wilcoxon signed-rank** (non-parametric): outlier 강함
- **Bootstrap CI** (1000 resample): mean dB의 95% CI

**적용 예**: A안 (BUF_LEN=2048) vs B안 (BUF_LEN=4096+NEON)
- 30 burst씩 측정
- paired t-test, Δμ_dB 와 p-value 보고
- p < 0.01 & CI에 0 미포함 시 채택

**판정**: ★★★ (검증 게이트 강화). 본 KB §05 §C에 추가 필수.

---

## 19.4 청각적 메트릭 (Perceptual)

**A-weighted SPL**:
- IEC 61672. 인간 청감 보정 (1kHz 기준).
- 코골이 100Hz는 A-weight -20dB → 청감상 작게 들림. 80Hz +30dB attenuation해도 dB(A)로는 +10dB만 체감.

**계산**: A-weight filter (IIR 4-cascade biquad). float 비용 무시할 만큼 적음.

**판정**: ★★★ — 본 프로젝트 success metric은 dB(A) reduction이 적절.

**PESQ / POLQA**:
- 음성 품질 평가. 코골이엔 부적절.

**STOI**:
- intelligibility. 음성용. 부적절.

**Loudness (Zwicker / Glasberg-Moore)**:
- 청감 모델. 정확하나 비용 크고 복잡.
- 본 프로젝트엔 A-weight로 충분.

---

## 19.5 측정 위치 / 보정

**현재**: error mic 위치 = 스피커 가까이 (sec_path 짧게 유지).

**문제**: 사용자 귀 위치는 다름. error mic에서 dB(A)는 user ear와 다름.

**해법**:
1. **Virtual mic (§15.2)**: M(z) 보정으로 user ear dB 추정.
2. **2회 측정**:
   - run1: SPL meter를 user ear 위치에 두고 baseline 측정
   - run2: 동일 위치에서 ANC ON 측정
   - 차이가 진짜 attenuation
3. **HW 추가**: user ear 위치에 monitor mic (ANC loop와 분리).

**보정 절차**:
- 1kHz tone -20 dBFS 재생, mic input dBFS 측정
- mic sensitivity dB SPL/Pa 알려져 있으면 dBFS → dB SPL 변환
- MAX4466 calibrated value: ~-44dB/Pa (datasheet)

---

## 19.6 실측 프로토콜 (변경 채택용)

매 변경마다 동일 절차:

1. **사전 준비**
   - 방 조용히 (HVAC OFF)
   - 스피커 볼륨 고정 (앰프 노브 사진/위치 표시)
   - mic gain 동일

2. **Baseline (adapt OFF) 측정**
   - 60s 코골이 raw loop 재생
   - err_rms 시계열 기록
   - burst별 분할 (VAD 또는 envelope threshold)

3. **변경 적용 측정**
   - 동일 60s, adapt ON
   - 동일 burst 분할

4. **분석**
   - Per-burst dB N=20+
   - Cumulative dB
   - dB(A) reduction (A-weight filter 적용)
   - Bootstrap CI

5. **채택 기준 (KB §05 §C 강화)**
   - mean Δ dB > 0
   - p < 0.05 (paired t-test)
   - 95% CI에 0 미포함
   - **silent safety 통과** (§04 §C 게이트 1)

---

## 19.7 분산 측정 (Wiener bound)

**§14.1과 연결**: 동일 데이터에 Wiener filter optimal achievable 추정.

**활용**: "현재 +5dB 천장이 알고리즘 한계인가, 신호의 본질적 한계인가?" 답.

**Wiener bound가 +5dB**: 본 KB 모든 카드 무의미, 본질 한계.
**Wiener bound가 +15dB**: 알고리즘 개선 여지 +10dB.

**판정**: ★★★ (Phase 0 필수 측정).

---

## 19.8 측정 도구

**필요**:
- Python (offline 분석) — 사용자 정책상 호스트 PC에서만, target 코드 비포함
- numpy, scipy, matplotlib
- 또는 C++ post-processor (target 호환)

**현재 보유**: 
- `archive/ANC_Bed_Speaker/snoring_analysis/` Python notebook
- `analysis_output/*.csv` 산출

**확장**: ANC 결과 분석용 후처리 스크립트 1개 추가.

---

## 19.9 우선순위

| 카드 | 점수 | Phase |
|---|---|---|
| **Per-burst + cumulative dB** (§19.2) | ★★★ | 0 (즉시 가능) |
| **A-weight filter** (§19.4) | ★★★ | 0 |
| **Wiener bound 계산** (§19.7) | ★★★ | 0 |
| **통계 검정 (paired t-test)** (§19.3) | ★★★ | 0 |
| **실측 프로토콜 표준화** (§19.6) | ★★★ | 0 |
| **User ear SPL 측정** (§19.5) | ★★ | 3 (HW 추가) |

**Phase 0 패키지** (= "측정 신뢰 재구축"):
1. 60s 표준 run + per-burst 분할 도구
2. A-weight filter post-processor
3. Wiener bound 계산기
4. paired t-test 결과 reporter

이 패키지 없이는 §11 §22 카드 우선순위가 통계적 근거 없음. **§05 §C 게이트에 통계 검정 추가 필수**.
