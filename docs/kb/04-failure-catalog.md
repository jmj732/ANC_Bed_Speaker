# 04. 실패 카탈로그

이미 시도하고 실패한 것을 재시도하지 않기 위한 기록. 각 항목 포맷:

> **시도 / 기대 / 결과 / 원인 / 재발 방지**

날짜는 메모 또는 이번 세션 기준. 출처는 항목 끝.

---

## #1. broadband FxLMS @ ref_lead = 0
- **시도**: period=32/64/128 모든 설정에서 broadband FxLMS `run` 모드.
- **기대**: ANC 동작 (>5dB 감쇠).
- **결과**: 모든 설정에서 발산 또는 +0.5dB 이하 미미한 감쇠. period 작을수록 더 빨리 발산(period=32 → 9초).
- **원인**: ref_lead = 0. xcorr lag=0 모든 측정. 인과성 미충족(§01 §1.6, §02 §2.5).
- **재발 방지**: broadband FxLMS는 ref mic 물리 재배치(05 Card C) 없이 시도 금지. NB-FxLMS 우회 사용.

출처: `memory/project_anc.md`, `memory/project_anc_latency_goals.md`.

---

## #2. `NB_F0_BUF_LEN = 8192` @ period=32
- **시도**: 저주파 검출 floor 23Hz로 확장 (5/12).
- **기대**: median 70Hz burst 커버.
- **결과**: **xrun 219회 / 12s**. compute max 1.001ms (budget 0.333ms의 3배). best_db는 측정 가능 (+4.7dB) 하지만 시스템 불안정.
- **원인**: autocorr 비용 O(BUF_LEN²)가 한 period에 집중. budget 초과.
- **재발 방지**: BUF_LEN ≥ 4096은 **autocorr 분산(여러 period로 split) 또는 별도 스레드** 없이는 시도 금지.

출처: 5/12 세션, `project_anc.md` 5/12 entry.

---

## #3. `NB_F0_BUF_LEN = 4096` (분산 없음)
- **시도**: 8192보다 가벼우면 OK일까 검증 (5/12).
- **기대**: xrun 없이 검출 floor 47Hz.
- **결과**: **xrun 263회 / 14s**. 8192보다 더 많음 (테스트 길이 차이).
- **원인**: autocorr 비용 4096도 여전히 한 period budget 초과.
- **재발 방지**: 분산 구현 없는 BUF_LEN 4096은 **검증 불필요**, 카탈로그로 확정.

출처: 5/12 세션.

---

## #4. `μ = 0.01` (이력)
- **시도**: 수렴 속도 향상 위해 mu 상향 (메모 4/23).
- **기대**: best_db ↑.
- **결과**: weight cap(0.3) 도달 후 새 burst에서 **위상 역전** → 에러 증폭.
- **원인**: NLMS 안정 영역 초과 (§01 §1.3). cap 도달 후 그래디언트 잘림.
- **재발 방지**: μ > 0.001 시도 금지. 수렴 속도가 필요하면 **신호 매칭(n_harm)** 으로 해결, μ 상향은 마지막.

출처: `memory/project_anc.md`.

---

## #5. `μ = 0.003` (5/12)
- **시도**: 0.001 → 0.003 단순 3배 상향. burst 수렴 시간 단축 의도.
- **기대**: dip 완화, best_db ↑.
- **결과**: 발산. err 0.0093 → 0.097, **best=−18.5dB**(=-18.5dB 감쇠 = err가 baseline의 8배), pk 출력 0.224, **FROZEN 3회 (스피커 소음 위험 — feedback_anc_noise.md 위반)**.
- **원인**: #4와 동일. 0.001을 안전 마진으로 봤어야 함.
- **재발 방지**: **μ는 한 번에 ≤ 2× 변경만**. 안전 검증(silent safety) 통과 후에만 추가 변경.

출처: 5/12 세션.

---

## #6. `n_harm = 2`, mu 그대로 (5/12)
- **시도**: fundamental만으로 천장이 +5dB이니 고조파 추가.
- **기대**: best_db ↑ (예: +6~7dB).
- **결과**: best 동급 (+4.5dB), **dip -3.7 → -5.8dB 악화**, xrun 1.
- **원인**: 코골이 burst마다 고조파 분포 다름(§03 §3.4). h=1이 실제로 약한 burst에선 잘못된 위상 학습 → 다음 burst에서 역작용.
- **재발 방지**: n_harm=2 단순 enable 금지. 고조파별 SNR/coherence 기반 동적 활성화 필요 (Card B).

출처: 5/12 세션.

---

## #7. `n_harm = 2` + `mu_scale[1] = 0.5` (5/12)
- **시도**: #6 부작용 완화. h=1 학습 속도 절반.
- **기대**: dip 회복, best 유지.
- **결과**: best 동급, dip -5.5dB. 발산은 안 함 (#6 대비 0.3dB 개선).
- **원인**: 고정 scale은 burst별 에너지 분포 무시. 효과 미미.
- **재발 방지**: 고정 mu_scale 추가 튜닝 무의미. **동적 mu_scale** (instantaneous SNR/coherence) 만 시도 가치 있음 (Card B).

> 코드는 master에 보존 (commit b18cf63). 동적 버전 작업 시 base로 활용.

출처: 5/12 세션.

---

## #8. Phase rotation (Δsx_phase로 weights 회전) (5/12)
- **시도**: f0 미세 변화 시 `Δφ = old_sx_phase − new_sx_phase`만큼 weights 회전 + |S| 비율 스케일 (clamp 0.5~2.0).
- **기대**: f0 5~30% 변동 시 weights 재수렴 시간 단축 → dip 완화.
- **결과**: dip −3.2dB → **−5.2dB 악화**. revert.
- **원인 (추정)**: sx_phase는 single-bin DFT라 측정 노이즈 큼. scale clamp가 sec_path null 근처에서 부작용. LMS 자연 수렴 궤적 교란.
- **재발 방지**: phase rotation 단독 적용 금지. 적용하려면 sx_phase 측정 안정화 + scale clamp 제거 후 large jump만 적용 (h≥2 reset 임계 매우 높일 때만).

출처: 5/12 세션. 코드는 revert됨 (master).

---

## #9. `NB_F0_MIN_HZ = 50` 단독 (5/12)
- **시도**: 저주파 burst 잡으려고 매크로 변경.
- **기대**: 검출 범위 확장.
- **결과**: 로그상 50~94Hz 검출 없음. 효과 없음.
- **원인**: 검출 floor는 매크로가 아니라 `BUF_LEN`이 결정 (식: `2·fs / BUF_LEN`). §01 §1.4.
- **재발 방지**: `NB_F0_MIN_HZ` 변경 전 항상 `BUF_LEN` 식 검증.

출처: 5/12 세션.

---

## #10. `baseline=3s` + `14s test` 단일 비교 (5/12)
- **시도**: 변경마다 14초 run 1회로 best_db 비교.
- **기대**: 변경 효과 측정.
- **결과**: best_db ±1dB 변동이 burst 정렬에 의해 좌우. **통계적으로 무의미한 비교**가 다수.
- **원인**: 8s 신호 루프 vs 3s baseline + 11s adapt → burst 시작 위상이 매 run 다름. advisor 지적.
- **재발 방지**: **baseline ≥ 1 신호 루프** (mollyroselee 기준 8s), **adapt ≥ 2 루프** (16s+), 그리고 3 run 평균. → §05 검증 게이트.

출처: 5/12 세션, advisor.

---

## #11. Makefile 매크로 의존성 미추적 (5/12)
- **시도**: `anc_defs.h` 매크로만 수정 후 `make`.
- **기대**: 변경 반영.
- **결과**: `make: Nothing to be done for 'all'`. 이전 바이너리 그대로 실행됨.
- **원인**: Makefile 규칙이 헤더 수정시간 추적 안 함.
- **재발 방지**: 매크로/헤더 수정 후 항상 `make clean && make`. 또는 `OPTFLAGS="-DXXX"` 임시 오버라이드.

출처: 5/12 세션.

---

## #12. `NB_F0_UPDATE_SAMPLES`: 19200 → 4800 (4/23)
- **시도**: f0 추정 주기 200ms → 50ms.
- **기대**: 빠른 추적 → best ↑.
- **결과**: +2.5dB → **+4.2dB** (성공).
- **원인**: burst마다 f0가 바뀌는 신호(§03 §3.5)에 대해 추적 빠르면 위상 매칭 ↑. autocorr cost는 update당 동일이라 평균 cost는 4× 증가하지만 BUF_LEN=2048이면 budget 안.
- **재발 방지**: 이 값을 다시 줄이려면 분산 구현이 같이 필요. 현재 값 유지.

출처: `memory/project_anc.md` 4/23 entry.

---

## 패턴 정리 (메타)

| 패턴 | 근거 | 예방 게이트 |
|---|---|---|
| "μ 한 번에 ≥2× 상향" → 발산 + 스피커 소음 | #4, #5 | μ 변경 ≤ 2×, silent safety 검증 |
| "BUF_LEN 분산 없이 ≥4096" → xrun | #2, #3 | autocorr cost 사전 추정 (식: BUF_LEN²) |
| "고정 파라미터 (mu_scale, NB_F0_MIN_HZ)" → 효과 없음 | #7, #9 | 우선 작동 모델 (signal-driven) 확인 |
| "단일 14s run 비교" → 통계적 무의미 | #10 | baseline ≥ 1 루프, 3 run 평균 |
| "매크로 수정 후 make" → 미반영 | #11 | `make clean && make` 또는 OPTFLAGS |
| "이론 검증 없이 적용" → 부작용 | #8 (phase rotation) | §01 식 → §03 신호 정합 → 시도 |

---

## Source

- `memory/project_anc.md` (4/9, 4/23, 5/12 entries)
- `memory/project_anc_latency_goals.md`
- `memory/feedback_anc_noise.md` (silent safety 원칙)
- 5/12 세션 직접 측정
- 관련 KB: §01 안정 영역, §02 budget, §05 검증 게이트
