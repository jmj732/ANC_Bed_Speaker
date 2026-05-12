# §22 Grand Matrix — 통합 카드 매트릭스 & 최종 로드맵

§06~§21에서 발굴된 50+ 기법을 통합. **본 시스템 제약 통과 + 천장 깨기 잠재력** 기준으로 Tier 재분류. 다음 세션의 단일 진입점.

---

## A. 카드 통합 매트릭스 (50+ 항목)

### 점수 표기
- ★★★ — 즉시 시도 권장 (Tier 1~2, 검증된 효과 또는 큰 잠재 이득)
- ★★ — Phase 2~3 카드 (조건부 또는 중간 비용)
- ★ — 연구/조건 추가 필요
- ❌ — 본 시스템 부적합 (제약 위반)

### A.1 알고리즘 카드 (§06, §10, §13, §14, §17)

| Card | 출처 | 점수 | 비용 | 차단조건 | 노트 |
|---|---|---|---|---|---|
| Huber-LMS | §13.5 | ★★★ | 1줄 | — | burst onset robust |
| VSS-LMS (Kwong) | §06.2 | ★★★ | 50줄 | — | mu 자동 조정 |
| Online sec_path ID | §06.7 | ★★★ | 200줄 | 안전성 검증 | 자세 변화 대응 |
| SM-NLMS (Set Membership) | §06.6 | ★★ | 30줄 | — | CPU ↓ |
| APA (Affine Projection) | §06.4 | ★★ | 200줄 | matrix inv 비용 | 수렴 ↑ |
| RLS | §06.5 | ★★ | 300줄 | 메모리 ↑ | n_harm 작아서 가능 |
| FDAF / PBFDAF | §10.1~2 | ★★ | 500줄 | block latency | broadband 부활 후 |
| Subband ANC | §10.4 | ★★ | 큼 | — | 광대역 |
| Recursive cos-sin | §10.6 | ★★★ | 30줄 | numerical drift | NEON 안 써도 빠름 |
| Wavelet packet | §17.1 | ★★ | 큼 | — | 시간-주파수 sparse |
| Sparse OMP | §17.4 | ★★ | 큼 | — | multi-tonal 자동 |
| Non-stationary chirp (KF rate) | §17.8 | ★★ | KF 확장 | — | f0 변화 추적 |
| Modulation envelope | §17.9 | ★★ | 100줄 | — | VAD 보강 |
| EMD/EEMD/HHT | §17.2~3 | ★ | 비결정적 | RT 부적합 | offline만 |
| Wiener bound 계산 | §14.1 | ★★★ | offline tool | — | 천장 검증 |
| Kalman f0 smoother | §14.2 / §08.4 | ★★★ | 100줄 | — | jitter ↓ |
| IMM Kalman | §14.2 | ★★ | 200줄 | — | burst on/off |
| HMM scheduler | §14.4 | ★★ | 200줄 | — | adapt on/off |
| GMM noise classifier | §14.5 | ★★ | 큼 | MFCC | 환경 잡음 거부 |
| Particle filter | §14.3 | ★★ | 큼 | N particle | multi-modal |
| EM joint estimator | §14.8 | ★★ | 큼 | — | n_harm 안정화 |
| Bayesian online | §14.6 | ★★ | 큼 | — | RLS 일반화 |
| H∞ controller | §13.1 | ★ | offline design | — | robust |
| Sign-error LMS | §13.4 | ★★ | 1줄 | — | fixed-point 시너지 |
| IMP + IMC | §13.8 | ★★ | §07.1과 묶음 | — | feedback ANC |
| Lyapunov 사전 검증 | §13.9 | ★★ | 도구 | — | 새 mu 검토 게이트 |
| SMC | §13.3 | ★ | — | chattering | research |
| MPC | §13.2 | ❌ | QP solver | budget | — |
| μ-synthesis | §13.6 | ❌ | offline 과대 | — | — |

### A.2 피치 추적 카드 (§08)

| Card | 출처 | 점수 | 비용 | 차단조건 |
|---|---|---|---|---|
| YIN | §08.1 | ★★ | 100줄 | autocorr 대비 무게 |
| Cepstrum | §08.2 | ★★ | 큼 | FFT 의존 |
| IIR Adaptive Notch | §08.3 | ★★★ | 200줄 | — |
| Kalman smoother (post) | §08.4 | ★★★ | 100줄 | — |
| HPS | §08.5 | ★★ | FFT | — |
| f0 후처리 패키지 (median+hyst+hold) | §08.7 | ★★★ | 100줄 | — |
| AMDF, ZCR | §08.6 | ★ | — | — |

### A.3 인과성 우회 (§07, §15)

| Card | 출처 | 점수 | 비용 | 차단조건 |
|---|---|---|---|---|
| **Feedback ANC (IMC)** | §07.1 | ★★★ | 큼 (새 모드) | sec_path 정확도 |
| ref mic 재배치 | §07.2 / §15.3 | ★★★ | HW 1~2h | 침대 작업 |
| Hybrid FB+FF | §07.5 | ★★★ | nb 위에 FB 추가 | 안정성 검증 |
| Prediction filter | §07.4 | ★★ | 100줄 | burst 변동 |
| Virtual mic | §15.2 | ★★★ | 측정 1회 + 코드 | 사용자 머리 |
| Headrest/Pillow | §15.4 | ★★★ | HW 큼 | Phase 4 |
| MIMO 2ch | §15.1 | ★★ | HW 추가 | — |
| ILC (Iterative Learning) | §07.3 | ★ | 데이터 누적 | — |
| Repetitive Control | §07.6 | ★ | RC ring | — |
| Coherence channel select | §15.7 | ★★ | 100줄 | — |
| Beam-forming ref | §15.8 | ★★ | mic array | — |

### A.4 코골이 특화 (§09)

| Card | 출처 | 점수 | 비용 |
|---|---|---|---|
| 다중 신호 평균 측정 | §09.1 | ★★★ | offline 도구 |
| Snore VAD | §09.2 | ★★★ | 100줄 |
| Phase-aware mu schedule | §09.3 | ★★★ | 50줄 |
| Burst onset detection | §09.4 | ★★★ | 100줄 |
| Dynamic mu_scale (SNR/coherence) | §09.5 | ★★★ | 100줄 |
| 독립 톤 추적 (multi-tonal) | §09.6 | ★★★ | 300줄 |
| Snore template matching | §09.7 | ★★ | template DB |

### A.5 HW / DSP 최적화 (§16)

| Card | 출처 | 점수 | 비용 | Phase |
|---|---|---|---|---|
| CPU governor performance | §16.6 | ★★★ | 1줄 | 1 |
| mlockall | §16.8 | ★★★ | 1줄 | 1 |
| isolcpus=3 | §16.4 | ★★★ | boot param | 1 |
| **NEON autocorr** | §16.1 | ★★★ | 1 함수 | 2 (BUF_LEN 확장 차단 해소) |
| ALSA mmap | §16.3 | ★★ | API 교체 | 2 |
| LTO | §16.10 | ★★ | 1 flag | 2 |
| Cache prefetch | §16.7 | ★★ | hint | 2 |
| Fixed-point | §16.2 | ★ | 큼 | 3 |
| PREEMPT_RT | §16.5 | (보류) | — | 사용자 정책 |
| fs 변경 | §16.11 | ★ | 큼 | 장기 |

### A.6 측정 / 검증 (§19)

| Card | 출처 | 점수 | Phase |
|---|---|---|---|
| Per-burst + cumulative dB | §19.2 | ★★★ | 0 (필수) |
| A-weight filter | §19.4 | ★★★ | 0 |
| Wiener bound 계산기 | §19.7 | ★★★ | 0 |
| Paired t-test reporter | §19.3 | ★★★ | 0 |
| 실측 프로토콜 표준 | §19.6 | ★★★ | 0 |
| User ear SPL 측정 | §19.5 | ★★ | 3 |
| Listening tests | §21.16 | ★★★ | 1+ |

### A.7 음향 / 기계 (§20)

| Card | 출처 | 점수 | Phase |
|---|---|---|---|
| 스피커/mic 재배치 | §20.2 | ★★★ | 1~3 |
| 헤드레스트/베개 통합 | §20.4 | ★★★ | 4 (최대 이득) |
| 흡음재 천장/벽 | §20.1 | ★★★ | 4 |
| 방진 mount | §20.5 | ★★ | 1 |
| 방 모달 처리 | §20.6 | ★★ | 5 |
| 스피커 어레이 | §20.3 | ★★ | 4 |
| 차음 창/문 | §20.8 | ★ | 5 |

### A.8 ML / Frontier (§12, §21)

| Card | 출처 | 점수 | Phase |
|---|---|---|---|
| FxNLMS+DNN scheduler (변형 A) | §12.2 / §21.2 | ★★ | 4 |
| Tiny GRU f0 smoother | §12.3 | ★★ | 4 |
| Selective Fixed-Filter (Shi) | §21.1 | ★★ | 5 |
| Impulsive/α-stable robust | §21.14 | ★★ | 1 (Huber와) |
| PSO/GA offline tuning | §21.10 | ★★ | 2~3 |
| Deep ANC end-to-end | §12.1 / §21.1 | ★ | 5+ |
| DNN reference predictor | §12.2 | ★ | 5+ |
| RL ANC | §12.5 | ❌ | silent safety |
| Federated / Transfer | §12.7 / §21.4 | ★ | 장기 |
| Diffusion / GAN / Quantum / Neuromorphic | §21.3, §21.7, §21.8 | ❌ | RT/HW |

---

## B. Top 15 단일 우선순위 (★★★ 중)

순서는 **(즉시 가능성) × (예상 이득) × (선행 의존성)**.

| 순 | Card | 분류 | 예상 효과 | 비용 |
|---|---|---|---|---|
| 1 | 측정 신뢰 패키지 (§19 §22 §A6) | Phase 0 | 의사결정 신뢰 ↑ | 0.5일 |
| 2 | Wiener bound 계산 (§14.1, §19.7) | Phase 0 | 천장 진단 | 0.5일 |
| 3 | CPU governor + mlockall + isolcpus (§16) | Phase 1 | xrun 0, jitter ↓ | 30분 |
| 4 | f0 후처리 패키지 (§08.7) | Phase 1 | dip 완화 | 0.5일 |
| 5 | Snore VAD (§09.2) | Phase 1 | adapt schedule | 0.5일 |
| 6 | Huber-LMS (§13.5) | Phase 1 | 발산 마진 ↑ | 5분 |
| 7 | 스피커/mic 재배치 (§20.2) | Phase 1 | +5~10dB 가능 | 1~2h |
| 8 | VSS-LMS (§06.2) | Phase 2 | mu 자동 | 1일 |
| 9 | Burst onset mu schedule (§09.4) | Phase 2 | dip 완화 | 1일 |
| 10 | Dynamic mu_scale (§09.5) | Phase 2 | n_harm 실효화 | 1일 |
| 11 | **NEON autocorr (§16.1)** | Phase 2 | BUF_LEN 4096 가능 | 1일 |
| 12 | 독립 톤 추적 (§09.6) | Phase 2 | multi-tonal 커버 | 2일 |
| 13 | **Feedback ANC (IMC) (§07.1)** | Phase 3 | broadband 부활 SW | 3~5일 |
| 14 | **ref mic 재배치 + virtual mic (§07.2 + §15.2)** | Phase 3 | broadband 부활 물리 | HW 1일 + 보정 |
| 15 | **헤드레스트/베개 통합 (§20.4 + §15.4)** | Phase 4 | +15~20dB 최대 | HW 큼 |

---

## C. Phase 0~6 통합 로드맵

### Phase 0 — 측정 신뢰 재구축 (0.5~1일)
**목표**: 의사결정의 통계적 근거 확보.
- 60s 표준 run 도구
- per-burst 분할 (VAD 또는 envelope)
- cumulative dB 시계열
- A-weight filter post-processor
- Wiener bound 계산기 → 천장 진단
- paired t-test reporter
- 실측 프로토콜 표준 문서화

**출구 조건**: 모든 후속 변경이 t-test p<0.05 + CI excludes 0 으로 검증 가능.

### Phase 1 — RT 안정성 + 손쉬운 알고리즘 (1~2일)
**목표**: xrun=0 안정, 발산 마진, f0 후처리.
- CPU governor performance
- mlockall, isolcpus=3
- f0 후처리 (median, hyst, hold)
- Snore VAD
- Huber-LMS
- 스피커/mic 재배치 + sec_path 재측정
- 청취 테스트 시작

**예상 누적**: dip -4dB → -1~0dB, best +5 → +6~7dB. xrun 0.

### Phase 2 — Tier 2~3 SW (3~5일)
**목표**: 알고리즘 정교화, BUF_LEN 확장.
- NEON autocorr → BUF_LEN=4096 (저주파 50% 커버)
- VSS-LMS
- Burst-aware mu schedule
- Dynamic mu_scale (SNR/coherence)
- 독립 톤 추적 (n_harm 대안)
- ALSA mmap, LTO, cache prefetch

**예상 누적**: best +7 → +8~10dB. dip 안정화.

### Phase 3 — 인과성 우회 (천장 깨기) (5~10일)
**목표**: broadband 부활.
- 본 카드 둘 중 선택:
  - **C-SW**: Feedback ANC (IMC) — coupling 보정, virtual mic
  - **C-HW**: ref mic 물리 재배치 + sec_path 재측정 + virtual mic 보정
- 둘 다 적용 시 hybrid FB+FF
- Online sec_path ID

**예상 누적**: best +10 → +15~20dB (광대역). 단 안정성 검증 비용 큼.

### Phase 4 — 통합 시스템 (HW + ML hybrid) (수 주)
**목표**: 최대 잠재 이득.
- 헤드레스트/베개 통합 (HW 큼)
- 흡음재 천장/벽
- MIMO 2ch err + 2ch spkr
- FxNLMS+DNN scheduler (hybrid)
- PSO/GA offline 하이퍼파라미터

**예상 누적**: dB(A) reduction +20~25dB at user ear.

### Phase 5 — Frontier ML
**목표**: 학습 기반 적응.
- Selective Fixed-Filter (controller bank)
- Tiny GRU f0 smoother
- Transfer learning across users
- Online continual learning

### Phase 6 — 비전 (양산성)
- Quality assurance: 자동 calibration
- User-friendly 설치 가이드
- 안전 인증 (IEC 60601 등 의료기기 기준 검토)

---

## D. 의사결정 게이트 (§05 보강안)

**모든 변경은 다음 통과해야 채택**:

1. **§04 카탈로그**: 동일/유사 시도 이력 검색 → 재시도 방지
2. **§A 매트릭스**: ★★★ 또는 ★★ + 조건 명시 시만 진행
3. **§19 측정 게이트**:
   - Per-burst N≥20, paired t-test p<0.05, 95% CI excludes 0
   - silent safety (§04 게이트 1) 통과
   - xrun ≤ 1 in 60s
4. **§13.9 Lyapunov 사전 검증** (μ/leak 변경 시): 이론 안정 마진 표 확인
5. **§22 Phase 정합**: 현재 Phase 범위 내 카드만 (Phase jump 금지)
6. **사용자 청취 확인** (Phase 1+): 청취 테스트 OK 필요

---

## E. 카드 거절 자동 응답

다음 패턴은 KB가 즉시 거절:

| 제안 패턴 | 거절 근거 |
|---|---|
| "Deep ANC end-to-end" | §12.1 budget 미달, §21.1 |
| "MPC / QP based" | §13.2 budget 미달 |
| "RL agent로 mu 조정" | §12.5 silent safety 위반 |
| "PREEMPT_RT 켜자" | §16.5 사용자 정책, baseline 안정화 전 금지 |
| "μ를 0.001 초과로" | §04 #4 #5, §13.9 |
| "BUF_LEN ≥4096" (NEON 없이) | §04 #2 #3 |
| "n_harm=2 (안정화 카드 없이)" | §04 #6 #7 |
| "phase rotation" | §04 #8 (이미 revert) |
| "PESQ로 검증" | §19.4 (음성용, 부적합) |
| "Diffusion / Quantum / Neuromorphic" | §21.3, §21.7, §21.8 |

---

## F. 본 시스템 천장의 본질적 한계 진단

**Wiener bound** 측정 필요 (§14.1, Phase 0).

**가설 1** (Wiener +5dB): 코골이 신호의 본질적 sparse harmonic 부족 → 어떤 알고리즘도 +5dB 한계.
- 대응: §15 §20 (HW + 기하) 외에는 길 없음.

**가설 2** (Wiener +15dB, narrowband 한정 +5dB): 알고리즘 simplification 한계.
- 대응: §07 §09 §10 § (multi-tonal, broadband 부활) 효과 큼.

**가설 3** (Wiener +20dB+): 본 시스템 알고리즘이 매우 미흡.
- 대응: 모든 Phase 카드 효과 큼.

**판정**: Wiener bound 측정 전까지 Phase 1~2까지만 진행. Phase 3+ 결정은 측정 후.

---

## G. 다음 세션 즉시 시작점

```
1. cat docs/kb/22-grand-matrix.md §B Top 15
2. cat docs/kb/22-grand-matrix.md §C Phase 0
3. Phase 0 측정 신뢰 패키지부터:
   - 60s 표준 run 스크립트
   - per-burst dB reporter
   - Wiener bound 도구
4. 결과 → §F 가설 분류 → Phase 진행 결정
```

**원칙 재확인**:
- 모든 변경은 §D 6게이트 통과
- §E 자동 거절 패턴 확인
- 한 번에 하나씩 (§04 #10 burst 정렬 lottery 회피)
- silent safety (§04 #5 mu 발산 회피)
- 측정 ≥ 60s, ≥20 burst, paired t-test

---

## H. KB 자체의 유지보수

**갱신 시점**:
- 새 카드 발굴 → §A 해당 분류에 추가, §22 §B Top 15 재평가
- 새 실패 → §04 entry, §E 자동 거절 갱신
- Phase 완료 → §C 완료 마킹, 다음 Phase 카드 활성화
- Wiener bound 측정 후 → §F 가설 확정, Phase 3+ 결정

**KB 충돌 해결**: §22 §A 매트릭스가 최종 권위. 다른 파일과 충돌 시 §22 우선.
