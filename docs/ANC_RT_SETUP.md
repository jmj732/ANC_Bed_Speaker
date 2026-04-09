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
- `cpu-performance.service`
- `scripts/anc-irq-affinity.sh`

## 생성한 서비스

- `cpu-performance.service`
- `anc-rt.service`

서비스 파일 위치:

- `/etc/systemd/system/cpu-performance.service`
- `/etc/systemd/system/anc-rt.service`

## 동작 방식

`cpu-performance.service`

- 부팅 시 모든 CPU governor를 `performance`로 고정

`anc-rt.service`

- `/home/admin/anc run` 실행
- `CPU3`에 고정
- `SCHED_FIFO` 우선순위 `70`
- 종료되면 자동 재시작

## 적용 명령

```bash
sudo systemctl daemon-reload
sudo systemctl enable cpu-performance.service
sudo systemctl enable anc-rt.service
sudo systemctl restart cpu-performance.service
sudo systemctl restart anc-rt.service
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

## 현재 측정 메모

- ALSA 설정: `48kHz`, `period=128`, `buffer=1536`
- 새 `sec_path.bin` 기준 latency: `335 samples = 6.98 ms`
- `performance + RT + CPU3` 적용 후 `128` period에서 짧은 테스트 동안 `xrun=0` 확인

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
