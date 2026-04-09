# Real-Time ANC Workspace

현재 활성 개발 기준은 `/home/admin` 루트의 C 구현입니다.

- 실시간 실행 코드: `anc.c`
- 빌드 결과물: `anc`
- 현재 운영 문서: `ANC_RT_SETUP.md`
- 서비스 설정: `anc-rt.service`, `cpu-performance.service`, `anc-irq-affinity.service`

## 현재 기준 문서

아래 세 파일을 최신 기준 문서로 봅니다.

- `README.md`: 작업 공간과 문서 분류
- `ANC_RT_SETUP.md`: Raspberry Pi 실시간 운영 설정과 측정 메모
- `anc.c`: 실제 런타임 동작과 기본 파라미터

`NEXT_SESSION_PROMPT.md`는 세션 인수인계 메모입니다. 참고용으로는 유용하지만 장기 문서의 기준점으로 쓰지는 않는 것이 좋습니다.

## 디렉터리 안내

- `anc.c`: 단일 파일 C 기반 FxLMS/FxNLMS ANC 구현
- `anc`: 현재 빌드된 실행 파일
- `sec_path.bin`: 현재 사용 중인 secondary path 측정본
- `scripts/`: 운영 보조 스크립트
- `logs/`: 측정 및 실행 로그
- `backups/`: 실험 백업과 이전 측정본
- `images/`: 하드웨어 연결 사진
- `ANC_Bed_Speaker/`: 초기 조사 자료와 Python 프로토타입 아카이브

## 빠른 시작

빌드:

```bash
gcc -O2 -o anc anc.c -lasound -lm
```

수동 실행:

```bash
./anc measure
./anc run
```

RT 조건으로 수동 실행:

```bash
sudo taskset -c 3 chrt -f 70 /home/admin/anc run
```

서비스 상태 확인:

```bash
systemctl status anc-rt.service
systemctl show anc-rt.service -p CPUAffinity -p CPUSchedulingPolicy -p CPUSchedulingPriority
```

## 문서 분류 기준

정리 기준은 아래처럼 잡는 것이 안전합니다.

- 최신 운영 문서: 루트 `/home/admin`의 C 코드와 RT 설정 문서
- 참고 문서: `ANC_Bed_Speaker/` 내부의 조사 보고서와 초기 사양서
- 세션 메모: `NEXT_SESSION_PROMPT.md`
- 실험 증적: `logs/`, `backups/`

즉, 지금 Git 쪽을 뜯어고칠 계획이라면 `ANC_Bed_Speaker/`는 "현재 제품 코드"보다 "아이디어, 조사, 초기 Python 프로토타입 기록"으로 보는 편이 맞습니다.
