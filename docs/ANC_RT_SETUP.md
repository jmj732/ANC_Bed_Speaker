# ANC RT Setup

이 문서는 `/home/admin` 루트의 C 기반 ANC 런타임 운영 기준 문서다.

현재 ANC 실행은 다음 조건으로 고정했다.

- CPU governor: `performance`
- ANC scheduler: `SCHED_FIFO 70`
- ANC CPU affinity: `CPU3`
- ANC service: systemd 자동 재시작

관련 파일:

- `README.md`
- `anc.c`
- `anc-rt.service`
- `anc-snore.service`
- `cpu-performance.service`
- `scripts/anc-irq-affinity.sh`

## 생성한 서비스

- `cpu-performance.service`
- `anc-rt.service`
- `anc-snore.service`

서비스 파일 위치:

- `/etc/systemd/system/cpu-performance.service`
- `/etc/systemd/system/anc-rt.service`
- `/etc/systemd/system/anc-snore.service`

## 동작 방식

`cpu-performance.service`

- 부팅 시 모든 CPU governor를 `performance`로 고정

`anc-rt.service`

- `/home/admin/anc run` 실행
- `CPU3`에 고정
- `SCHED_FIFO` 우선순위 `70`
- 종료되면 자동 재시작

`anc-snore.service`

- `/home/admin/anc-rt/build/snore_anc run` 실행
- 코골이 측정본 기준 `70~250Hz`, `mb-bands=2`
- 상향 파라미터 `mu=0.0020`, `mu-n-max=0.006`, `eps-rel=0.02`, `OUTPUT_LIMIT=0.45`
- `CPU3`에 고정
- `SCHED_FIFO` 우선순위 `70`
- 종료되면 자동 재시작
- `anc-rt.service`와 동시 실행 금지

## 적용 명령

```bash
sudo systemctl daemon-reload
sudo systemctl enable cpu-performance.service
sudo systemctl restart cpu-performance.service
```

기본 ANC를 쓸 때:

```bash
sudo systemctl disable --now anc-snore.service
sudo systemctl enable --now anc-rt.service
```

코골이 ANC를 쓸 때:

```bash
sudo systemctl disable --now anc-rt.service
sudo systemctl enable --now anc-snore.service
```

## 상태 확인

```bash
systemctl status cpu-performance.service
systemctl status anc-rt.service
systemctl show anc-rt.service -p CPUAffinity -p CPUSchedulingPolicy -p CPUSchedulingPriority
ps -eo pid,cls,rtprio,psr,cmd | grep '/home/admin/anc run'
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo "$f: $(cat $f)"; done
```

## 수동 실행과 동등한 명령

```bash
sudo taskset -c 3 chrt -f 70 /home/admin/anc run
```

코골이 전용 수동 실행:

```bash
cd /home/admin/anc-rt/runtime
sudo taskset -c 3 chrt -f 70 /home/admin/anc-rt/build/snore_anc run --period=64 --buffer-mult=12 --start-fill=2 --xrun-fill=2 --mb-bands=2 --band-lo=70 --band-hi=250 --mu=0.0020 --mu-n-max=0.006 --eps-rel=0.02 --adapt-delay-ms=400
```

## 현재 측정 메모

- ALSA 설정: `48kHz`, `period=128`, `buffer=1536`
- 새 `sec_path.bin` 기준 latency: `335 samples = 6.98 ms`
- `performance + RT + CPU3` 적용 후 `128` period에서 짧은 테스트 동안 `xrun=0` 확인

## 2026-04-09 Snore ANC 메모

- `snore_sec_path.bin`를 RT 측정으로 다시 생성했다.
- 측정 결과: `peak=0.1716 @ 218 samples = 4.54 ms`, `xrun=1`
- 코골이 서비스 운영값: `period=64`, `buffer=768`, `start-fill=2`, `xrun-fill=2`, `mb-bands=2`, `band=70~250Hz`, `mu=0.0020`, `mu-n-max=0.006`, `eps-rel=0.02`, `OUTPUT_LIMIT=0.45`
- 현재 서비스는 발산 시 best snapshot으로 복귀한 뒤 `fixed` 상태로 유지한다.
- 장시간 관찰에서 `xr=0`, `NRestarts=0` 유지 확인
- 다만 ambient xcorr가 `lag=0`으로 나와 ref mic가 에러 mic보다 충분히 선행하지 않는다.
- 큰 감쇠가 더 필요하면 소프트웨어 튜닝보다 먼저 마이크/스피커 배치를 바꿔 ref mic가 코골이를 더 먼저 듣게 해야 한다.

## 2026-04-02 Latency Test Results

테스트 조건:

- 실행: `sudo taskset -c 3 chrt -f 70 /home/admin/anc ...`
- 측정 시간: `20초`
- 실행 시간: `20초`
- 출력 제한: 현재 코드 기준 `OUTPUT_LIMIT=0.35`

결과 요약:

| 케이스 | DSP Program | Fill | Measured Latency | Xrun | Run 결과 |
|---|---|---:|---:|---:|---|
| 기준 | `Item #0 FIR interpolation with de-emphasis` | `2` | `332 samples = 6.92 ms` | `0` | best `+4.0 dB`, 가장 안정적 |
| Fill 감소 | `Item #0 FIR interpolation with de-emphasis` | `1` | `203 samples = 4.23 ms` | `1` | 발산, 불합격 |
| DSP 저지연 | `Item #1 Low latency IIR with de-emphasis` | `2` | `314 samples = 6.54 ms` | `0` | best `+1.1 dB`, 성능 저하 |
| Fill+DSP 저지연 | `Item #1 Low latency IIR with de-emphasis` | `1` | `185 samples = 3.85 ms` | `1` | 발산, 불합격 |
| DSP 대안 | `Item #4 Ringing-less low latency FIR` | `2` | `314 samples = 6.54 ms` | `0` | best `+1.1 dB`, 성능 저하 |

현재 채택:

- DSP Program: `Item #0`
- Fill: `2`
- `sec_path.bin`: 기준 측정본(`332 samples = 6.92 ms`)

로그 파일:

- `/home/admin/test_f2_dsp0_measure.log`
- `/home/admin/test_f2_dsp0_run.log`
- `/home/admin/test_f1_dsp0_measure.log`
- `/home/admin/test_f1_dsp0_run.log`
- `/home/admin/test_f2_dsp1_measure.log`
- `/home/admin/test_f2_dsp1_run.log`
- `/home/admin/test_f1_dsp1_measure.log`
- `/home/admin/test_f1_dsp1_run.log`
- `/home/admin/test_f2_dsp4_measure.log`
- `/home/admin/test_f2_dsp4_run.log`

## 주의

- 재부팅 후 ANC가 자동 시작된다.
- 자동 시작을 끄려면 아래를 실행한다.

```bash
sudo systemctl disable --now anc-rt.service
```
