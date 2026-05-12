# 05. 결정 트리

증상 → 원인 → Tier → 검증 게이트 → 다음 카드. **변경을 제안하기 전에 이 문서를 통과해야 함.**

---

## A. 진단 분기 (증상 → 원인 카테고리)

```
[증상]
├─ 발산: err > 2× baseline 이 ≥1초 지속
│   → 원인: μ 과대 / weight cap / leak
│   → 참조: §01 §1.3, §04 #4, #5
│   → 조치: 즉시 출력 OFF (frozen), μ 절반, weight reset
│
├─ 천장: best_db가 +5dB 부근에서 정체
│   → 원인: 신호 매칭 부족 (n_harm, BUF_LEN, ref_lead)
│   → 참조: §03 시사점 표, §04 #6, #7
│   → 조치: §D 다음 카드 검토 (A 또는 B)
│
├─ 검출 흔들림: f0 옥타브 슬립 / conf 변동 / SILENT 자주
│   → 원인: BUF_LEN floor / hold / octave-lock
│   → 참조: §01 §1.4 §1.5, §03 §3.2
│   → 조치: conf threshold, hold 강화. floor 확장은 분산 구현 필요
│
└─ xrun: 12s에 ≥ 5회
    → 원인: period budget vs autocorr / compute cost
    → 참조: §02 §2.4, §04 #2 #3
    → 조치: BUF_LEN 또는 NB_F0_UPDATE_SAMPLES 보수적으로
```

---

## B. Tier 분류 (변경 비용 / 위험)

| Tier | 예 | 비용 (시간) | 위험 (시스템) | 검증 깊이 |
|---|---|---|---|---|
| **1** | CLI flag (`--nb-mu`, `--nb-leak`, `--n-harm`, `--snore-file`) | 분 | 낮음 (런타임만) | 단일 run 가능 |
| **2** | 매크로 / 단일 함수 (`NB_F0_BUF_LEN`, `mu_scale[]` 분포, `nb_set_f0` reset 임계) | 시간 | 중 (재빌드 + 부작용) | 3-run 평균 필수 |
| **3** | 알고리즘 추가 (autocorr 분산, 동적 mu_scale, SNR 추정기) | 일 | 높음 (회귀 가능) | §C 게이트 전체 |
| **4** | 물리 (ref mic 배치, 침대-스피커 거리, 부품 교체) | 일 ~ 주 | 매우 높음 (sec_path 재측정 필요) | §C 게이트 + sec_path remeasure |

> **원칙 (메모리 feedback_anc_style.md):** Tier 1부터, 그리고 한 번에 하나씩. baseline 성공 전 Tier 3/4 시도 금지.

---

## C. 검증 게이트 (변경 채택 기준)

채택하려면 다음 5개 모두 통과:

| # | 게이트 | 기준 | 근거 |
|---|---|---|---|
| C1 | **Silent safety** | f0 미검출(conf < THR) 시 anti-output = 0 | feedback_anc_noise.md |
| C2 | **μ 안전 마진** | 이전 stable 값의 2× 이내 | §04 #4 #5 |
| C3 | **baseline ≥ 1 신호 루프** | mollyroselee 기준 8s 이상 (또는 `BASELINE_SECS=8` 빌드) | §04 #10 |
| C4 | **3-run 평균 개선** | best/dip 두 지표 모두 개선해야 채택. 한쪽만 좋아지면 reject | §04 #6 #7 |
| C5 | **xrun ≤ 1 in 14s** | 시스템 안정성 | §02 §2.4 |

> 게이트 미통과 변경은 코드를 commit하지 말고 revert 또는 보존-branch.

### 게이트 적용 시 실제 명령 시퀀스
```bash
# 1. 빌드 (매크로 변경 시 clean 필수)
cd /home/admin/anc-rt && make clean && make

# 2. 안전 점검 (silent safety) - n_harm 그대로 짧게
cd runtime && timeout 8 ../build/anc passthrough 2>&1 | head -20

# 3. 14s 본 테스트 ×3 (서로 다른 burst 위상)
for i in 1 2 3; do
  timeout 22 ../build/anc nb \
    --n-harm=N --nb-mu=M --nb-leak=L \
    --snore-file=/tmp/snoring.raw 2>&1 \
    | tee /tmp/nb_run${i}.log
  sleep 2
done

# 4. 평균
grep "best=" /tmp/nb_run*.log | awk '{print $NF}'
```

---

## D. 다음 카드 우선순위 (현재 천장 +4.4~+5.7dB 기준)

상태: master commit `b18cf63`. 5/12 세션 종료점.

| Card | 변경 | Tier | 예상 이득 | 차단 조건 / 비용 | 권장 순위 |
|---|---|---|---|---|---|
| **A** | autocorr 분산 (period 분할 또는 별도 스레드) → BUF_LEN=4096 활성화 | 3 | +1~3dB (저주파 50% 커버) | 분산 코드 복잡도, RT 안전성 | 2nd |
| **B** | 동적 mu_scale (instantaneous SNR/coherence 기반 per-harmonic) | 3 | +1~2dB (n_harm=2 실효화) | SNR 추정 정확도, false high in low-conf | 3rd |
| **C** | ref mic 물리 재배치 (스피커 반대편 / 환자 머리쪽) → broadband 부활 | 4 | +10~15dB (가장 큰 잠재 이득) | 침대-스피커 재배치, sec_path 재측정 | **1st** |
| **D** | 안정 단일 파일(mollyroselee) 한정 튜닝 | 1 | +0.5dB | 일반화 안 됨, 검증용으로만 | dev only |

### 권장 다음 카드: **C** 또는 **A**
- **C가 가장 큰 잠재 이득** (broadband 부활). 사용자 침대 배치 한 번 변경 + sec_path 재측정.
- C 불가 시 **A**가 SW 한도 내 최대치. 분산 구현 작업량 중간.
- **B는 A 또는 C 이후**. n_harm=1 천장이 +5dB여서 동적 mu_scale 단독으로는 부가 이득 작음.

### 카드 시작 전 체크리스트
1. §A 진단 분기로 현 증상이 카드 의도와 맞는가?
2. §B Tier가 §C 검증 게이트 깊이를 감당할 수 있는가?
3. §04 카탈로그에 이미 시도된 게 있는가? (특히 #1, #2, #6 ~ #8)
4. 코드 변경이라면 백업: `git stash` 또는 별도 branch.
5. 실측 전 silent safety 짧은 run으로 검증.

### 추가 카드 풀
본 §D의 A~D는 5/12 세션 종료 시점의 최상위 4개만. **30+ 기법의 전체 매트릭스 + Phase 0~4 로드맵은 §11 (`11-feasibility-matrix.md`)** 참조. 새 카드 후보:
- **Phase 0 (측정 신뢰)**: §11 §D Phase 0 — §09 F1 다중 신호 평균, §09 F2 per-burst 분포 logger
- **Phase 1 (즉시 SW)**: §08 E (f0 후처리), §08 D1 (Kalman), §09 A2/A3 (VAD/onset), §09 B1 (phase-aware mu), §06 B1/B2 (VSS)
- **Phase 2 (천장 깨기 SW)**: §09 C2 (dynamic mu_scale), §09 C3 (multi-tonal), §08 C1 (IIR ANF 저주파)
- **Phase 3 (천장 깨기 본 카드)**: 본 §D §C (물리 재배치) 또는 §07 A1 (Feedback ANC/IMC) — 둘 중 택일
- **Phase 4 (broadband 부활 후)**: §10 A3 (PBFDAF), §10 B1 (Subband), §06 C1 (APA)

---

## E. "이 변경 해도 될까?" 즉답 매트릭스

| 제안 | 즉답 | 근거 |
|---|---|---|
| "μ 올려서 빠르게 수렴" | ❌ 0.001 초과 시도 금지 | §04 #4 #5 |
| "BUF_LEN 8192 / 4096" | ❌ 분산 없이는 xrun | §04 #2 #3 |
| "n_harm 2로 늘리자" | ⚠️ 동적 scaling 함께 (Card B) | §04 #6 #7 |
| "phase rotation 다시 시도" | ⚠️ sx_phase 안정화 선행 | §04 #8 |
| "NB_F0_MIN_HZ 더 낮춰" | ❌ BUF_LEN floor가 우선 | §01 §1.4, §04 #9 |
| "ref mic 위치 바꿔보자" | ✅ Card C, sec_path 재측정 필수 | §02 §2.5, §05 D |
| "단발 14초 run으로 비교" | ❌ 3-run 평균 필수 | §04 #10, §C |
| "고정 mu_scale 다른 분포" | ❌ 효과 미미, 동적으로 | §04 #7 |
| "NB_F0_UPDATE_SAMPLES 다시 19200(200ms)로" | ❌ 4800(50ms)이 +2.5→+4.2dB 검증값 | §04 #12 |
| "매크로 수정 후 `make` 만" | ❌ `make clean && make` 또는 `OPTFLAGS=-DXXX` | §04 #11 |

---

## Source

- §01 ~ §04 KB 본문
- `memory/feedback_anc_style.md` (한 번에 하나씩, baseline 우선)
- `memory/feedback_anc_noise.md` (silent safety)
- 5/12 세션 advisor 지적
