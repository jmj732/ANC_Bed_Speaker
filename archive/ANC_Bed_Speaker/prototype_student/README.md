# Python Prototype (Stage 1, Legacy)

이 디렉터리는 초기 PC 기반 ANC 프로토타입입니다.

- 언어/런타임: Python
- 오디오 I/O: `sounddevice`
- 목적: FxLMS 흐름 검증, secondary path 학습, baseline/ANC 로그 비교

현재 활성 런타임은 `/home/admin/anc.c` 기반 C 구현입니다. 이 폴더는 초기 알고리즘 검증 기록으로 보는 편이 맞습니다.

## 언제 참고하면 좋은가

- FxLMS 처리 흐름을 빠르게 다시 읽고 싶을 때
- Python에서 실험하던 CLI와 로그 포맷을 확인할 때
- stage 1 요구사항과 stage 2 제품 요구사항을 비교할 때

## 빠른 시작

```bash
cd /home/admin/ANC_Bed_Speaker/prototype_student
python3 -m venv .venv
source .venv/bin/activate
pip install -e .[dev,plot]
```

장치 확인:

```bash
python main.py --list-devices
```

secondary path 학습:

```bash
python main.py learn-secondary --duration 10 --mu 0.02
```

baseline 기록:

```bash
python main.py baseline --duration 30
```

ANC 실행:

```bash
python main.py run --secondary secondary_path.npz
```

PSD 비교:

```bash
python tools/measure_psd.py --baseline baseline.npz --anc anc_run.npz
```

## 주요 파일

- `main.py`: CLI 진입점
- `anc/`: DSP, FxLMS, secondary path, baseline 로직
- `tools/measure_psd.py`: baseline/ANC 로그 비교
- `STUDENT_PROJECT_SPEC.md`: 1단계 사양서
- `docs/`: 일반인 설명서, 용어집, 문서 인덱스
- `tests/`: 단위 테스트

## 생성되는 산출물

- `secondary_path.npz`
- `baseline.npz`
- `anc_run.npz`

위 파일들은 실험 로그이므로, 현재 C 구현으로 넘어갈 때는 포맷 참고용으로만 보는 편이 좋습니다.
