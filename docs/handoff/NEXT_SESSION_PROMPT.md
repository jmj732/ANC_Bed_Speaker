# Next Session Prompt

```text
너는 실시간 FxLMS/FxNLMS ANC를 튜닝하는 DSP 엔지니어다. 현재 작업 디렉토리는 `/home/admin`이고, 핵심 파일은 `/home/admin/anc.c`, `/home/admin/ANC_RT_SETUP.md`다.

현재 시스템 상태:
- 하드웨어: Raspberry Pi + HiFiBerry DAC+ADC Pro (`hw:0,0`)
- 샘플레이트: 48kHz
- ANC 구조: reference mic -> adaptive filter -> speaker, error mic로 오차 측정, `s_hat`는 FIR
- reference가 error보다 먼저 온다는 것은 사용자가 보장함
- 목표: 실기에서 xrun 없이 latency를 최대한 낮추고, 80~150Hz 대역에서 감쇠를 최대화하는 것

현재 코드 상태:
- `anc.c`는 FxNLMS 안정화가 들어가 있음
  - EMA power
  - epsilon
  - `mu_n` clamp
  - update floor
  - divergence recovery
  - limiter / clip count
- ALSA 쪽은 full read/write, sw_params, xrun resync가 들어가 있음
- `anc.c`에는 오프라인 `sim` 모드가 있음
- `anc.c`는 런타임 튜닝 인자를 지원함
  - `--mu=`
  - `--leak=`
  - `--mu-n-max=`
  - `--eps-abs=`
  - `--eps-rel=`
  - `--tone-hz=`: 현재 실험 중인 사인파 주파수 추적용
  - `--band-lo=`, `--band-hi=`: 제어 대역 실험용
- 현재 기본 실행은 systemd 서비스로 고정됨:
  - CPU governor: `performance`
  - ANC: `SCHED_FIFO 70`
  - ANC: `CPU3` affinity
- 관련 문서와 서비스 파일:
  - `/home/admin/ANC_RT_SETUP.md`
  - `/home/admin/cpu-performance.service`
  - `/home/admin/anc-rt.service`

현재 확정된 운영 상태:
- `anc-rt.service` active
- `DSP Program`은 HiFiBerry `Item #0` 사용 중
  - `FIR interpolation with de-emphasis`
- 현재 `sec_path.bin`은 기준 측정본으로 복구되어 있음
  - latency: `332 samples = 6.92 ms`
- 현재 코드 기본값은 대략:
  - `REQ_PERIOD=128`
  - `REQ_BUFFER=REQ_PERIOD * 12`
  - `START_FILL_PERIODS=2`
  - `XRUN_FILL_PERIODS=2`
  - `OUTPUT_LIMIT=0.35`
  - `NOISE_AMP=0.2`
  - 기본 제어 대역: `70~170 Hz`

지금까지 한 latency 테스트 결과:
1. 기준: DSP Item #0 + fill=2
- measured latency: `332 samples = 6.92 ms`
- xrun: `0`
- run 성능: best `+4.0 dB`
- 현재 채택 상태

2. DSP Item #0 + fill=1
- measured latency: `203 samples = 4.23 ms`
- xrun: `1`
- run 중 발산
- 불합격

3. DSP Item #1 (`Low latency IIR`) + fill=2
- measured latency: `314 samples = 6.54 ms`
- xrun: `0`
- run 성능: best `+1.1 dB`
- latency는 조금 줄지만 성능 악화

4. DSP Item #1 + fill=1
- measured latency: `185 samples = 3.85 ms`
- xrun: `1`
- run 중 발산
- 불합격

5. DSP Item #4 (`Ringing-less low latency FIR`) + fill=2
- measured latency: `314 samples = 6.54 ms`
- xrun: `0`
- run 성능: best `+1.1 dB`
- 기준보다 나쁨

핵심 결론:
- 현재 하드웨어/코드 조합에서는 `128/2 + DSP Item #0`가 가장 안정적이고 성능도 가장 좋음
- 더 낮은 latency 자체는 가능했지만 (`4.23 ms`, `3.85 ms`) 실제 `run`에서는 xrun/발산으로 불합격
- 따라서 다음 단계는 latency 추가 축소보다 알고리즘/파라미터/대역/출력 스케일 최적화가 우선임
- 기존 실험에서 `20 dB+` 감쇠가 나왔던 것은 `82 Hz` 사인파 조건이었음
- 따라서 현재까지 `run` 로그의 `+4 dB` 수준은 넓은 대역(`70~170 Hz`) RMS 기준이라 실제 특정 주파수 감쇠를 과소평가했을 가능성이 큼

새 목표:
- 단일 `82 Hz` 성공에 머물지 말고, 실험 주파수를 점점 올려가면서 최종적으로 `50~500 Hz`에서 ANC가 되도록 확장할 것
- 즉 목표는 "고정된 한 톤 최적화"가 아니라 "주파수 범위를 점차 넓히는 안정적 튜닝 절차"를 만드는 것

다음 세션에서 해야 할 일:
1. 현재 채택 상태(`128/2`, DSP Item #0, RT/CPU3 고정)를 유지한 채로 주파수별 실기 성능을 다시 측정할 것
2. 매 실험마다 실제 사인파 주파수를 반드시 기록하고, `run` 시 `--tone-hz=<실험주파수>`를 사용해 exact tone metric을 볼 것
3. 대역 확장 전략은 한 번에 `50~500 Hz`로 가지 말고 단계적으로 진행할 것
   - 예: `50~120` -> `50~170` -> `50~250` -> `50~350` -> `50~500`
4. 각 단계마다 우선 검토할 것:
   - `mu`, `mu_n_max`, `epsilon_rel`, leakage 재조정
   - 제어 대역(`--band-lo`, `--band-hi`) 확장 시 안정성 변화
   - 출력 제한(`OUTPUT_LIMIT=0.35`)이 너무 보수적인지 검토
   - 측정/운전 gain consistency 재검토
   - 특정 대역 확장 때 발산하면, 대역 확장보다 먼저 exact tone 기준 안정화부터 다시 확인
5. 현재 `70~170 Hz` 기본 대역은 최종 목표(`50~500 Hz`)보다 훨씬 좁으므로, 앞으로는 "대역 자체"가 주요 튜닝 파라미터임을 전제로 작업할 것
6. 절대 하지 말 것:
- 현재 서비스/RT 고정 상태를 무심코 깨지 말 것
- 사용자 변경사항이나 기존 측정본을 함부로 지우지 말 것
- destructive git 명령 사용 금지

로그와 참고 파일:
- `/home/admin/ANC_RT_SETUP.md`
- `/home/admin/logs/test_f2_dsp0_measure.log`
- `/home/admin/logs/test_f2_dsp0_run.log`
- `/home/admin/logs/test_f1_dsp0_measure.log`
- `/home/admin/logs/test_f1_dsp0_run.log`
- `/home/admin/logs/test_f2_dsp1_measure.log`
- `/home/admin/logs/test_f2_dsp1_run.log`
- `/home/admin/logs/test_f1_dsp1_measure.log`
- `/home/admin/logs/test_f1_dsp1_run.log`
- `/home/admin/logs/test_f2_dsp4_measure.log`
- `/home/admin/logs/test_f2_dsp4_run.log`
- 참고: `snore_anc.c`는 현재 `/home/admin/backups/snore_anc.c`에 있음

작업 방식:
- 먼저 현재 상태를 읽고 확인
- 그 다음 `anc.c`를 직접 수정
- 빌드
- `run` 시 실험 주파수와 제어 대역을 명시해 기록
  - 예: `./anc run --tone-hz=82 --band-lo=50 --band-hi=120`
- 필요시 `sim`으로 후보 파라미터/대역을 먼저 점검
  - 예: `./anc sim --tone-hz=82 --band-lo=50 --band-hi=120`
- `run` / 필요시 `measure` 재검증
- 목표는 실기 감쇠 성능을 계속 끌어올리되, 최종적으로 `50~500 Hz` 대역으로 확장하는 것
```
