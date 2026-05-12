# §17 비전통적 변환 / 신호 표현 기반 ANC

표준 FxLMS / FFT 외의 신호 표현 — wavelet, EMD, sparse 등. 본 시스템 narrowband 한계 돌파의 보조 카드.

---

## 17.1 Wavelet Packet ANC

**원리**: DWT 또는 wavelet packet으로 신호를 시간-주파수 격자에 분해. 각 sub-band에서 독립 LMS.

**장점**: 코골이처럼 시간적 burst + 주파수 sparse 신호에 효율적. STFT보다 시간 해상도 ↑ (lower freq).

**참고**: Erkelens, J.S. & Hendriks, R.C. — wavelet 도메인 잡음 억제.

**Pi5 비용**: 4-level DWT (Daubechies-4), period=32 → 약 0.05ms.

**적용성**: 
- band별 weight → §10 subband ANC의 wavelet 버전
- 시간적으로 짧은 burst의 onset/decay 잘 분리

**판정**: ★★ (Phase 3, §10 §10.4 subband와 경쟁 카드).

---

## 17.2 EMD / EEMD (Empirical Mode Decomposition)

**원리**: 비선형/비정상 신호를 Intrinsic Mode Functions (IMF)로 분해. data-driven, no basis 가정.

**ANC 응용**: 코골이 → IMF1(고주파 hiss), IMF2~5(main snore harmonics), IMF6+(breath rumble). IMF별 ANC.

**참고**: Huang, N.E. et al. (1998). "The empirical mode decomposition..."

**문제**:
- iterative sifting, 수렴 시간 비결정적 → RT 부적합
- mode mixing 발생
- Pi5에서 비용 ~ms 단위

**판정**: ★ (offline 분석 도구로만, real-time 부적합).

---

## 17.3 Hilbert-Huang Transform (HHT)

**원리**: EMD + Hilbert spectrum. 순간 주파수 / 진폭 추출.

**ANC 응용**: f0 tracking 대안. 단 EMD 한계 동일.

**판정**: ★ (offline 분석용).

---

## 17.4 Sparse Representation / Compressive Sensing

**원리**: 코골이 신호를 dictionary D 위 sparse code α로 표현. y = D·α, α는 sparse (대부분 0).

**Dictionary**: harmonic atoms (e.g., 50~500Hz × 0~5 harmonics × phase) 또는 학습된 K-SVD.

**Pursuit**: OMP (Orthogonal Matching Pursuit) — O(K·N²), N=block, K=non-zero count.

**ANC 응용**: 
- ref mic 입력 sparse decompose → 강한 atom만 anti-noise 합성
- 비코골이 잡음 자동 거부 (atom 매칭 안 됨)

**Pi5 cost**: block 256, K=5 → 약 1~3ms. period=32엔 불가, sub-block 또는 50ms 주기 가능.

**판정**: ★★ (Phase 4 카드, n_harm 확장의 sparse 버전).

**참고**: Aharon, M., Elad, M. (2006). "K-SVD..."

---

## 17.5 Compressive Sensing for 저샘플 ref

**원리**: ref mic 신호가 sparse(harmonic)면 sub-Nyquist 측정 가능. 본 시스템엔 직접 응용 부족.

**판정**: ★ (학술 영역).

---

## 17.6 Phase Vocoder / STFT-domain processing

**원리**: STFT block의 magnitude/phase 직접 조작. magnitude는 noise model, phase는 보존.

**ANC 응용**: post-processing으로 안티노이즈 spectral shape 조정.

**한계**: block latency = block_size/fs. block=1024 @ 96kHz = 10.7ms → 본 system budget 부적합.

**판정**: ★ (block-based 모드에서만).

---

## 17.7 Time-stretching for prediction

**원리**: 코골이가 quasi-periodic이면 직전 burst를 time-stretch 하여 다음 burst 예측 → ref_lead 인위 생성.

**참고**: §07 §7.4 예측 필터의 확장.

**문제**: burst-to-burst 변화 큼 (분석상 std 큼) → 예측 오류 ↑.

**판정**: ★ (변동 큼).

---

## 17.8 Non-stationary harmonic model (Doppler-like)

**원리**: f0(t)가 시변. NB-FxLMS의 oscillator step도 시변 → phase_step(t) chirp.

**구현**: f0_rate를 Kalman으로 추정 (§08 §8.4 확장) → phase_step에 chirp 보정.

**판정**: ★★ (Phase 2~3, KF 확장으로 자연스럽게 들어옴).

---

## 17.9 Modulation domain (AM/FM)

**원리**: 코골이를 carrier × envelope 로 분해. envelope follower로 burst 추적.

**관련**: §09 §9.2 VAD, §09 §9.4 burst onset detection 의 일반화.

**Pi5 cost**: 미미.

**판정**: ★★ (Phase 2, VAD 강화).

---

## 17.10 Cyclostationary analysis

**원리**: 코골이 호흡 주기(3~5s)의 cyclic spectral analysis. envelope의 spectral correlation.

**활용**: 호흡 주기 자체를 활용한 long-term scheduling — inhale phase에서만 adapt 활성화.

**판정**: ★ (대규모 데이터 필요, 효과 검증 안 됨).

---

## 17.11 Chirp Z-Transform (CZT)

**원리**: 임의 곡선상 sampling. f0 후보 영역에서 fine resolution.

**용도**: §08 §8.5 HPS와 결합, f0 finer resolution.

**Pi5 cost**: FFT 보다 약간 비쌈.

**판정**: ★ (HPS로 충분).

---

## 17.12 우선순위

| 카드 | 점수 | Phase |
|---|---|---|
| Non-stationary harmonic chirp (§17.8) | ★★ | KF 확장 (Phase 2~3) |
| Modulation envelope (§17.9) | ★★ | VAD 보강 (Phase 2) |
| Wavelet packet ANC (§17.1) | ★★ | Phase 3 |
| Sparse / OMP (§17.4) | ★★ | Phase 4 |
| EMD/EEMD/HHT (§17.2~3) | ★ | offline 분석만 |
| Phase vocoder, Cyclostationary, CZT, CS, time-stretch | ★ | 학술/특수 |

**즉시 시도**: §17.9 modulation envelope — §09 §9.2 VAD 구현 시 자연스럽게 포함.
