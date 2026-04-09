# ANC_Bed_Speaker Archive

이 디렉터리는 수면용 ANC 제품을 검토하던 초기 자료를 모아둔 아카이브입니다.

- 시장/제품 조사
- 1단계 PC 프로토타입 사양
- 2단계 임베디드 제품 사양
- 초기 Python 기반 FxLMS 프로토타입

현재 활성 구현은 이 폴더 밖의 `/home/admin/anc.c`입니다. Raspberry Pi 실시간 운영 기준은 `/home/admin/ANC_RT_SETUP.md`를 보는 것이 맞습니다.

## 이 폴더를 읽어야 할 때

- 제품 방향성과 UX 가설을 다시 확인할 때
- 초기에 정의했던 성공 기준과 비목표를 재검토할 때
- Python 프로토타입 설계나 실험 절차를 참고할 때

## 문서 안내

- `deep-research-report.md`: 제품화 가능성, 시장, UX, 기술 리스크 조사 보고서
- `PROTOTYPE_PRODUCT_SPEC.md`: 2단계 임베디드 제품 사양
- `prototype_student/STUDENT_PROJECT_SPEC.md`: 1단계 PC 프로토타입 사양
- `prototype_student/README.md`: 초기 Python 프로토타입 실행 가이드
- `prototype_student/docs/README.md`: Python 프로토타입 관련 문서 인덱스

## 현재 문맥에서의 위치

이 폴더는 "활성 코드 저장소"라기보다 아래 역할에 가깝습니다.

- 제품/연구 히스토리 보관
- 초기 알고리즘 실험 기록
- Git 리포지토리를 새로 정리할 때 가져갈 요구사항의 출처

즉, 지금 기준으로는 여기 문서를 그대로 실행 기준으로 믿기보다, 현재 C 구현과 비교하면서 필요한 내용만 가져가는 편이 맞습니다.
