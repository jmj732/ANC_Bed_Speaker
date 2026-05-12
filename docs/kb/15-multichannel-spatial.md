# §15 Multi-Channel / Spatial / MIMO ANC

본 시스템은 현재 SISO (1 ref + 1 err + 1 anti). Multi-channel 확장은 HW 변경 필요하나 큰 잠재 이득.

---

## 15.1 MIMO FxLMS

**원리**: M reference × K error × L anti-noise channels. weight 행렬 W[L×M], 각 element 독립 LMS.

**공식**: y_l(n) = Σ_m W_lm(z) · x_m(n), update via filtered-x with cross secondary paths S_kl.

**비용**: O(M·K·L·N_tap). 본 시스템 narrowband에선 O(M·K·L·2n_harm) → n_harm=1, MKL=8이면 16개 weight. 매우 가벼움.

**필요 HW**:
- 추가 mic: MAX4466 ×1~2개 (~$3/ea)
- 추가 speaker: 작은 풀레인지 (~$10/ea)
- HiFiBerry DAC2 ADC Pro는 2-in 2-out → 추가는 USB audio 또는 multi-codec 필요

**효과 예상**: 
- 2 err mic → null point ↑, head movement robust
- 2 ref mic → coherent reference 활용 (spatial filtering)

**판정**: ★★★ (Phase 3, HW 추가 필요). budget 충분.

---

## 15.2 Virtual Microphone Technique

**원리**: 실제 err mic은 스피커 근처에 두고, **사용자 귀 위치의 virtual error**를 추정하여 거기서 null 생성.

**식**: virtual err = M(z) · physical err, where M(z)는 미리 측정한 transfer.

**참고**: Garcia-Bonito et al. (1997). "Virtual microphone for ANC..."

**본 시스템 적용**: 
- err mic을 현재 위치(스피커 가까이)에 두면 sec_path 짧음 → 인과성 ↑
- 사용자 귀(베개) 위치는 virtual로 처리

**필요**: 보정 단계 — 사용자 머리 위치에서 1회 측정.

**효과 예상**: 베개 위치 음압 ↓, sec_path 짧아져 broadband 부활 보조.

**판정**: ★★★ (Phase 3, 측정 1회 + 코드 추가).

---

## 15.3 Remote Microphone / Acoustic Sensing

**원리**: 잡음원 근처에 마이크 (코골이 발생: 환자 입/코 근처). ref_lead 크게 확보.

**본 시스템 적용**: 현재 R채널 ref mic을 침대 옆이 아닌 **환자 머리 옆** 또는 마스크형으로 배치.

**효과**: ref_lead = (입~mic 0.05m) - (입~error mic 0.5m) → 약 1.3ms lead 확보. sec_path 2.96ms 대비 부족하나 절반 보충.

**한계**: 진정한 broadband에는 sec_path 만큼 lead 필요 (즉, ref mic을 더 멀리 또는 sec_path 단축).

**판정**: ★★★ (§07 §7.2 카드 F1과 동일, Phase 3 물리 작업).

---

## 15.4 Headrest / Pillow-integrated ANC

**원리**: 사용자 귀 5cm 이내에 스피커 + err mic 통합. local zone control.

**구현 예시**:
- Bose QuietComfort 헤드폰 = 단일 사용자 ANC의 정점
- 헤드레스트 ANC = 자동차 운전석에 상용화 중

**본 프로젝트 적용**: 베개 양옆 슬림 스피커 + 베개 표면 mic. 
- sec_path 5~10cm → 0.3~0.6ms (현재 2.96ms 대비 5× 단축)
- ref_lead = 외부 노이즈 mic 대비 +1~2ms
- → broadband 부활 + 광대역 +15~20dB 가능

**비용**: HW 작업 큼. 침대 배치 변경.

**판정**: ★★★ (Phase 4, 최대 잠재 이득 카드).

---

## 15.5 Sound Zone Control / Pressure-Matching

**원리**: 다중 스피커 + 다중 mic으로 "bright zone" (사용자 귀) 음압 ↓, "dark zone" (옆사람) 그대로. Pressure Matching (PM) 또는 ACC (Acoustic Contrast Control).

**참고**: Druyvesteyn, W.F. & Garas, J. (1997); Choi & Kim (2002).

**본 프로젝트 적용**: 부부 침대에서 한쪽만 코골이 차단.

**비용**: 스피커 ≥4개, mic ≥4개, RT MIMO 처리 ~수 ms.

**판정**: ★ (overkill, 우선순위 낮음).

---

## 15.6 Wave Field Synthesis / Ambisonics for ANC

**원리**: 평면파/구면파 합성으로 공간 모든 점에서 캔슬.

**판정**: ❌ 학술 영역, 임베디드 부적합.

---

## 15.7 Coherence-based channel selection

**원리**: M 개의 ref mic 중 err와 coherence γ²(f) 높은 것만 활용.

**식**: γ²(f) = |S_xe(f)|² / (S_xx(f) · S_ee(f))

**본 시스템 활용 (mic 1개라도)**: ref mic frequency band별 coherence 측정 → low-coherence band에선 adapt 끄기. §09 §9.5 보조.

**판정**: ★★ (현재 mic 구성에서도 1차원적 적용 가능).

---

## 15.8 Beam-forming for reference

**원리**: ref mic array로 코골이 방향만 강조. delay-and-sum 또는 MVDR.

**필요 HW**: mic array ≥3개, 일정 간격 배치.

**효과**: 환경 잡음(fan, AC)에서 코골이만 추출 → adapt 정확도 ↑.

**판정**: ★★ (Phase 4, MIMO 확장과 묶음).

---

## 15.9 본 시스템 점진적 확장 경로

| Step | HW 추가 | 효과 |
|---|---|---|
| 1 | (현 HW) ref mic 재배치만 | ref_lead ↑ → broadband 회복 |
| 2 | err mic 1개 추가 (총 2) | virtual mic + 머리 움직임 robust |
| 3 | 스피커 1개 추가 (총 2) | sound zone, 양귀 캔슬 |
| 4 | 베개 통합 (mic+spkr 4+) | local zone, +15~20dB |
| 5 | 환자 마스크 mic | remote sensing, +20dB |

---

## 15.10 우선순위

| 카드 | 점수 | Phase |
|---|---|---|
| ref mic 재배치 (§15.3) | ★★★ | 3 (= §07 F1 = §11 1st) |
| Virtual mic (§15.2) | ★★★ | 3 |
| Headrest/Pillow integration (§15.4) | ★★★ | 4 (최대 이득) |
| MIMO 2ch err (§15.1) | ★★ | 3~4 |
| Coherence channel selection (§15.7) | ★★ | 2 (현 HW에서 1차원) |
| Beam-forming ref (§15.8) | ★★ | 4 |
| Sound zone PM (§15.5) | ★ | 5 |
| Ambisonics | ❌ | — |

**최대 잠재 이득 카드**: §15.4 베개 통합 (>+15dB). 단 HW 작업 큼.
**즉시 시도**: §15.3 ref mic 재배치 (작업 최소, 이득 큼).
