# 03. 코골이 신호 프로파일

"잡을 수 있는 신호 vs 못 잡는 신호". 알고리즘 가정과 실제 신호의 정합 여부를 미리 본다.

---

## 3.1 데이터셋

- **출처**: `archive/ANC_Bed_Speaker/snoring_analysis/analysis_output/*.csv`
- **수**: 29개 코골이 mp3 (`snoring_data/freesound_community-*`, `mollyroselee-...` 등)
- **세그먼트**: 1444 (frame 단위로 분석)
- **샘플레이트**: 48kHz mp3 디코드 (시스템은 96kHz 동작, mpg123로 리샘플 필요)

---

## 3.2 f0 (fundamental) 분포

출처: `dominant_frequency_summary.csv`

| 통계 | 값 (Hz) |
|---|---|
| segment_count | 1444 |
| mean | 96.9 |
| **median** | **70.3** ⚠️ |
| std | 90.3 |
| p10 | 31.25 |
| p90 | 187.5 |

> 절반 이상의 segment에서 f0 < 94Hz. 현재 `BUF_LEN=2048` 검출 floor 94Hz와 충돌 → §01 §1.4, §04 #9.

---

## 3.3 에너지 분포 (전체 평균)

출처: `fft_summary.csv`, `bandpass_summary_80_300hz.csv`

| 항목 | 값 |
|---|---|
| dominant FFT peak | 136.07 Hz |
| dominant amplitude | 3522.28 |
| 80–300Hz 대역 / 전체 (FFT 파워) | **70%** |
| 80–300Hz 대역 / 전체 (bandpass RMS) | 82% |

→ NB-FxLMS 1차 타깃 대역은 80–300Hz로 두는 것이 합리적. 그러나 f0 자체는 더 낮을 수 있음(§3.2).

---

## 3.4 고조파 에너지 (spectrogram)

출처: `spectrogram_summary.csv`

| 항목 | 값 |
|---|---|
| num_frames | 754 |
| dominant_freq_mean | **433.9 Hz** |
| dominant_freq_std | 407.18 Hz |
| mean_db (전체 대역 평균) | −24.0 dB |
| mean_db (80–300Hz 평균) | 2.6 dB |

> burst 평균적 dominant가 fundamental(평균 97Hz)보다 ~4.5× 위 → **에너지 다수가 3~6차 고조파에 존재**. n_harm=1 추적만으로는 잡히지 않는 영역. → §05 Card B.

---

## 3.5 burst-to-burst 변동성

- f0 std 90Hz, dominant std 407Hz → **burst마다 매우 다름**
- 단일 algorithm 파라미터로 일반화 어려움
- `scatter_summary.csv`: 1444 점에서 dominant_freq ↔ snore_index pearson_corr = 0.05 (거의 무상관) → 강한 코골이라고 f0가 일정한 것 아님

---

## 3.6 snore_index 분포 (파일 단위)

출처: `snore_index_summary.csv` (n=29 파일)

| 강도 | 개수 |
|---|---|
| high (> 0.5) | 10 |
| mid (0.2 – 0.5) | 8 |
| low (< 0.2) | 11 |

→ 데이터셋의 1/3이 low intensity. ANC 검증 시 high/mid에서 우선 검증, low는 silent safety(§04 #10) 우선.

---

## 3.7 단일 안정 신호 (실험용 reference)

**`mollyroselee-a-person-snoring-468533.mp3`**

| 항목 | 값 |
|---|---|
| sample_rate | 48000 Hz |
| duration | 8.04 s |
| amplitude RMS | 0.117 |
| FFT top-10 | **134.7 ~ 136.3 Hz** (0.5Hz 폭 안에 집중) |
| 특성 | 매우 안정된 ~135Hz 톤 (사실상 단일 frequency, 변동 작음) |

> NB-FxLMS 알고리즘 자체의 검증용 reference. 이 신호에서 +N dB 안 나오면 알고리즘 문제, 다른 신호에서만 안 나오면 일반화 문제.

raw 변환:
```bash
mpg123 -s --rate 96000 -m \
  archive/ANC_Bed_Speaker/snoring_analysis/snoring_data/mollyroselee-a-person-snoring-468533.mp3 \
  > /tmp/snoring.raw
```
(int16 mono 96kHz, anc_nb.c `--snore-file` 옵션이 요구하는 포맷)

---

## 3.8 알고리즘 가정 ↔ 신호 시사점

| 신호 특성 | 알고리즘 가정 | 정합? | 함의 |
|---|---|---|---|
| median f0 = 70Hz | BUF_LEN=2048 → floor 94Hz | ❌ | 50% segment 검출 불가, §05 Card A |
| burst-to-burst f0 변화 (std 90Hz) | 한 burst에서 weight 수렴 시간 < burst 길이 가정 | ⚠️ | burst 짧고 μ 낮으면 dip, §04 #6 |
| dominant_freq_mean 434Hz | n_harm=1 (fundamental만 추적) | ❌ | n_harm 확장 필요, §05 Card B |
| 80–300Hz에 70% 에너지 | OUTPUT_SAFETY_LPF=800Hz | ✅ | 현재 LPF 적절 |
| mollyroselee 안정성 | 단일 frequency 가정 | ✅ | 알고리즘 검증 base 신호 |
| snore_index low 1/3 | f0 미검출 시 silent | ✅ | silent safety가 핵심, §04 #5 |

---

## Source

- raw 데이터: `archive/ANC_Bed_Speaker/snoring_analysis/snoring_data/*.mp3`
- 분석 CSV: `archive/ANC_Bed_Speaker/snoring_analysis/analysis_output/*.csv`
- 관련 KB: §01 검출 floor, §05 Card A/B
