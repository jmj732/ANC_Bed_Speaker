# ANC Knowledge Base

ANC Bed Speaker 프로젝트의 의사결정 근거. 새 변경/실험 전에 반드시 통과해야 하는 reference.

---

## 구성

### Core (의사결정 5종) — 변경 제안 직전 필수 통과

| # | 파일 | 내용 | 언제 본다 |
|---|---|---|---|
| 01 | [01-theory.md](01-theory.md) | FxLMS / NB-FxLMS / pitch tracking / 안정성 조건 / 인과성 / 검출 floor 식 | 알고리즘 동작이 의심될 때 |
| 02 | [02-hw-constraints.md](02-hw-constraints.md) | HW, ALSA 설정, sec_path 측정, compute budget, 변경 가능/불가 표 | "이 매개변수 바꿀 수 있나?" 의문 |
| 03 | [03-signal-profile.md](03-signal-profile.md) | snoring_analysis 통계, 알고리즘 가정 ↔ 신호 정합 | 신호 매칭이 의심될 때 |
| 04 | [04-failure-catalog.md](04-failure-catalog.md) | 이미 시도-실패한 12개 항목 (메모 + 5/12 세션) | 변경 제안 직전 (재시도 방지) |
| 05 | [05-decision-tree.md](05-decision-tree.md) | 증상 → 진단 → Tier → 검증 게이트 → 다음 카드 | **모든 변경 시작점** |

### Survey 1 (알고리즘/HW 풀) — 새 기법 후보 탐색 시 참조

| # | 파일 | 내용 |
|---|---|---|
| 06 | [06-adaptive-algorithm-survey.md](06-adaptive-algorithm-survey.md) | LMS 패밀리 변형 (VSS, APA, RLS, SM-NLMS, online sec_path) |
| 07 | [07-causality-workarounds.md](07-causality-workarounds.md) | ref_lead=0 우회 (Feedback ANC/IMC, ILC/RC, PLL, 물리 재배치) |
| 08 | [08-pitch-tracking-survey.md](08-pitch-tracking-survey.md) | f0 추정 대안 (YIN, IIR ANF, HPS, Kalman, sliding DFT) |
| 09 | [09-snore-domain-techniques.md](09-snore-domain-techniques.md) | 코골이 특화 (VAD, burst-aware mu, multi-tonal) |
| 10 | [10-frequency-domain-multirate.md](10-frequency-domain-multirate.md) | FDAF / PBFDAF / Subband / Volterra / DXHS |
| 11 | [11-feasibility-matrix.md](11-feasibility-matrix.md) | 30+ 기법 × 본 시스템 제약 매트릭스 + Phase 0~4 로드맵 |

### Survey 2 (Deep / Robust / Bayesian / Spatial / Frontier) — 깊은 탐색

| # | 파일 | 내용 |
|---|---|---|
| 12 | [12-deep-learning-ml.md](12-deep-learning-ml.md) | DNN-ANC, Tiny RNN/LSTM, FxNLMS-DNN hybrid, RL, GAN, on-device |
| 13 | [13-robust-model-based-control.md](13-robust-model-based-control.md) | H∞, LQG/MPC, SMC, signed/Huber LMS, μ-synthesis, Lyapunov |
| 14 | [14-bayesian-statistical.md](14-bayesian-statistical.md) | Wiener bound, KF 변형, particle filter, HMM/GMM, EM, VB |
| 15 | [15-multichannel-spatial.md](15-multichannel-spatial.md) | MIMO FxLMS, virtual mic, headrest/pillow, sound zone |
| 16 | [16-hardware-dsp-pi5.md](16-hardware-dsp-pi5.md) | NEON, fixed-point, ALSA mmap, isolcpus, governor, mlockall |
| 17 | [17-unconventional-transforms.md](17-unconventional-transforms.md) | Wavelet packet, EMD/HHT, Sparse/OMP, phase vocoder, chirp |
| 18 | [18-snore-clinical-acoustics.md](18-snore-clinical-acoustics.md) | 코골이 임상 (VOTE), 자세별/OSA 차이, Helmholtz 모델 |
| 19 | [19-validation-metrics.md](19-validation-metrics.md) | per-burst dB, A-weight, Wiener bound, paired t-test, 프로토콜 |
| 20 | [20-acoustic-mechanical.md](20-acoustic-mechanical.md) | 흡음재, 스피커/mic 재배치, 헤드레스트, 방진, 모달 처리 |
| 21 | [21-frontier-2020-2026.md](21-frontier-2020-2026.md) | Deep ANC, Selective FF, federated, listening tests, frontier |

### Master Matrix — **모든 의사결정의 최종 권위**

| # | 파일 | 내용 |
|---|---|---|
| 22 | [22-grand-matrix.md](22-grand-matrix.md) | §06~§21 통합 매트릭스 / Top 15 우선순위 / **Phase 0~6 로드맵** / 자동 거절 패턴 |

---

## 표준 사용 시나리오

### 시나리오 1: 새 변경 제안 검토
```
1. §22 §E 자동 거절 패턴에 매칭 → 즉시 차단
2. §22 §A 매트릭스에서 카드 점수 (★★★/★★/★/❌) 확인
3. §04 카탈로그에서 동일/유사 시도 검색
4. §03 신호 시사점과 정합?
5. §01 식 + §13.9 Lyapunov와 모순 없는가?
6. §22 §D 6게이트 통과 가능한가?
```

### 시나리오 2: 실험 결과 해석
```
1. §22 §C 현재 Phase에서 예상한 효과인가?
2. 발산? → §05 §A + §04 #4 #5 + §13.9
3. 천장? → §22 §F Wiener 가설 분류, Phase 진행 결정
4. dB 들쭉날쭉? → §19 통계 검정, paired t-test
```

### 시나리오 3: 새 기법 후보 떠올림
```
1. §22 §A 매트릭스에 이미 있는가? → 점수 확인
2. 없으면 §06~§21 해당 카테고리에 entry 추가
3. §22 §A 매트릭스 갱신, Top 15 재평가
4. Phase 정합 확인 (§22 §D §5)
```

---

## 갱신 규칙

| 이벤트 | 갱신 대상 |
|---|---|
| 새 실패 발생 | §04 entry, §22 §E 자동 거절 패턴 |
| 새 카드 발굴 | §06~§21 해당 분류, §22 §A 매트릭스, Top 15 재평가 |
| 새 측정값 (sec_path, autocorr cost) | §02 표 |
| 새 데이터셋 분석 | §03 |
| 카드 적용 완료 | §22 §C Phase 완료 마킹, 다음 Phase 활성화 |
| Wiener bound 측정 후 | §22 §F 가설 확정, Phase 3+ 결정 |
| 결정 규칙 변경 | §05 §A/§B/§C/§E, §22 §D |
| 이론적 결론 변경 | §01 (드물어야 함) |

**원칙**:
- KB = "현재 상태의 source of truth"
- 메모리(`memory/project_anc.md`) = 시간순 로그
- 새 사실은 KB 먼저, 메모리 나중
- **§22 §A 매트릭스가 최종 권위**. 다른 파일과 충돌 시 §22 우선

---

## 관련 외부 reference

- `memory/MEMORY.md` — Claude 자동 로드 인덱스
- `memory/project_anc_kb_index.md` — KB 진입점 stub
- `memory/project_anc.md` — 시간순 작업 로그
- `memory/project_anc_latency_goals.md` — sec_path 분해
- `memory/feedback_anc_noise.md` — silent safety 원칙
- `memory/feedback_anc_style.md` — 협업 스타일
- `archive/ANC_Bed_Speaker/snoring_analysis/analysis_output/*.csv` — 신호 raw 통계
- `src/anc_algo.h` — 알고리즘 구현
- `graphify-out/GRAPH_REPORT.md` — 코드 의존 그래프

---

## 빠른 시작

새 세션에서:
1. **§22 §G** — 다음 세션 즉시 시작점
2. **§22 §B** Top 15에서 다음 카드 확인
3. **§22 §C** Phase 0 측정 신뢰 패키지부터 (필수 선행)
4. **§22 §D** 6게이트 검증 후 채택

당장 막혔으면 **§22 §E 자동 거절** 또는 **§05 §E 즉답 매트릭스**부터.

새 기법을 떠올렸으면:
- **§22 §A 매트릭스**에 점수 있는가? 없으면 §06~§21 해당 카테고리 추가 → §22 갱신
