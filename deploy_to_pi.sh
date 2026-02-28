#!/bin/bash
# ============================================================
# deploy_to_pi.sh — Bedside ANC 라즈베리파이 배포 스크립트
# 사용법: bash deploy_to_pi.sh
# ============================================================

PI_USER="baro"
PI_HOST="192.168.219.115"
PI_PORT="22"
PI_DIR="~/anc-project/prototype_student"
LOCAL_PROJECT_DIR="$(cd "$(dirname "$0")/prototype_student" && pwd)"

echo "======================================"
echo "  Bedside ANC → 라즈베리파이 배포"
echo "======================================"
echo "대상: ${PI_USER}@${PI_HOST}:${PI_DIR}"
echo "소스: ${LOCAL_PROJECT_DIR}"
echo ""

# 1) 연결 테스트
echo "[1/4] SSH 연결 확인 중..."
ssh -o ConnectTimeout=5 -p "$PI_PORT" "${PI_USER}@${PI_HOST}" "echo 'SSH 연결 성공'" || {
    echo "ERROR: SSH 연결 실패. IP/포트/비밀번호를 확인하세요."
    exit 1
}

# 2) 파일 전송
echo ""
echo "[2/4] 프로젝트 파일 전송 중..."
rsync -avz --progress \
    --exclude '__pycache__' \
    --exclude '*.egg-info' \
    --exclude 'venv' \
    --exclude '.git' \
    -e "ssh -p ${PI_PORT}" \
    "${LOCAL_PROJECT_DIR}/" \
    "${PI_USER}@${PI_HOST}:${PI_DIR}/"

echo ""
echo "[3/4] 라즈베리파이에서 환경 설정 중..."
ssh -p "$PI_PORT" "${PI_USER}@${PI_HOST}" bash << 'REMOTE'
set -e

echo "--- 시스템 패키지 설치 (portaudio, libatlas) ---"
sudo apt-get update -qq
sudo apt-get install -y -qq portaudio19-dev libatlas-base-dev python3-venv python3-pip

mkdir -p ~/anc-project
cd ~/anc-project/prototype_student

echo "--- venv 생성 ---"
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

echo "--- venv 활성화 및 패키지 설치 ---"
source venv/bin/activate
pip install --upgrade pip -q
pip install -e ".[dev]" -q

echo ""
echo "--- 설치된 패키지 확인 ---"
pip list | grep -E "numpy|scipy|sounddevice|pytest"

echo ""
echo "--- 오디오 장치 목록 ---"
PYTHONPATH=~/anc-project:~/anc-project/prototype_student \
    python main.py --list-devices 2>/dev/null || echo "(오디오 장치 없음 또는 오류)"

REMOTE

echo ""
echo "[4/4] 완료!"
echo "======================================"
echo "접속 방법:"
echo "  ssh baro@192.168.219.115"
echo ""
echo "테스트 실행:"
echo "  ~/run_tests.sh"
echo ""
echo "실행 순서:"
echo "  1) 2차 경로 학습:"
echo "     ~/run_anc.sh learn-secondary --duration 10 --mu 0.02"
echo ""
echo "  2) ANC 실행:"
echo "     ~/run_anc.sh run --secondary secondary_path.npz"
echo ""
echo "  3) 기준 신호 녹음:"
echo "     ~/run_anc.sh baseline --duration 30"
echo "======================================"
