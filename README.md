# anc-rt

정리된 C 기반 실시간 ANC 프로젝트 트리다.

현재 `/home/admin` 루트에 흩어져 있던 활성 파일을, Git에 올리기 쉬운 구조로 다시 모아둔 작업용 디렉터리다. 기존 `/home/admin` 실행 환경은 그대로 두었고, 이 디렉터리는 정리된 기준 트리 역할을 한다.

## 구조

- `src/`: C 소스
- `build/`: 빌드 산출물
- `runtime/`: 실행 시 필요한 런타임 파일과 임시 산출물
- `docs/`: 운영 문서, 개발 메모, 인수인계
- `systemd/`: 서비스 파일
- `config/`: sysctl, systemd, 오버레이 관련 설정
- `scripts/`: 설치 및 측정 보조 스크립트
- `assets/`: 연결 사진 등 참고 자산
- `archive/`: 과거 Python 프로토타입과 조사 자료

## 빠른 시작

빌드:

```bash
cd /home/admin/anc-rt
make
```

수동 실행:

```bash
cd /home/admin/anc-rt/runtime
../build/anc measure
../build/anc run
```

서비스 파일을 새 구조 기준으로 보려면 아래를 참고한다.

- `systemd/anc-rt.service`
- `scripts/apply-anc-kernel-tuning.sh`

## 현재 운영 환경과의 관계

- 현재 실제 실행 환경: `/home/admin`
- 정리된 프로젝트 기준: `/home/admin/anc-rt`

즉, 지금 당장은 새 디렉터리를 Git 기준점으로 쓰고, 운영 전환은 나중에 별도로 해도 된다.

## Git에 올릴 때 권장 범위

보통 아래만 추적하면 된다.

- `src/`
- `docs/`
- `systemd/`
- `config/`
- `scripts/`
- `assets/`
- `Makefile`
- `README.md`

반대로 아래는 기본적으로 추적하지 않는 편이 낫다.

- `build/`
- `runtime/*.bin`
- `runtime/*.log`
- `runtime/backups/`
- 실험 중 생성되는 측정 결과물
