"""
Pytest configuration and shared fixtures.
KR: Pytest 설정 및 공유 픽스처.
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Generator

import numpy as np
import pytest

from prototype_student.anc.dsp import DspConfig


@pytest.fixture
def default_config() -> DspConfig:
    """Create default DSP configuration for tests.
    KR: 테스트용 기본 DSP 설정 생성."""
    return DspConfig()


@pytest.fixture
def small_config() -> DspConfig:
    """Create smaller DSP configuration for faster tests.
    KR: 빠른 테스트를 위한 작은 DSP 설정."""
    return DspConfig(
        w_len=32,
        s_len=32,
        bandpass_taps=33,
        max_log_samples=1000,
    )


@pytest.fixture
def temp_directory() -> Generator[Path, None, None]:
    """Create temporary directory for test files.
    KR: 테스트 파일용 임시 디렉토리 생성."""
    with TemporaryDirectory() as temp_dir:
        yield Path(temp_dir)


@pytest.fixture
def sample_secondary_path(small_config: DspConfig) -> np.ndarray:
    """Create sample secondary path impulse response.
    KR: 샘플 2차 경로 임펄스 응답 생성."""
    s_hat = np.zeros(small_config.s_len, dtype=np.float32)
    # Simple exponential decay
    decay = np.exp(-np.arange(small_config.s_len) / 10)
    s_hat[:] = decay.astype(np.float32)
    s_hat /= np.sum(s_hat)  # Normalize
    return s_hat


@pytest.fixture
def sample_secondary_path_file(
    temp_directory: Path, sample_secondary_path: np.ndarray
) -> Path:
    """Create temporary secondary path npz file.
    KR: 임시 2차 경로 npz 파일 생성."""
    path = temp_directory / "secondary_path.npz"
    np.savez(
        path,
        s_hat=sample_secondary_path,
        sample_rate=48000,
        block_size=64,
        duration_s=10.0,
        mu=0.02,
    )
    return path


@pytest.fixture
def sample_audio_block(default_config: DspConfig) -> np.ndarray:
    """Create sample audio block (2 channels: 1 ref + 1 error).
    KR: 샘플 오디오 블록 생성 (2채널: 레퍼런스 1 + 에러 1)."""
    rng = np.random.default_rng(42)
    frames = default_config.block_size
    return rng.standard_normal((frames, 2)).astype(np.float32) * 0.1


@pytest.fixture
def sample_tone_200hz(default_config: DspConfig) -> np.ndarray:
    """Create 200 Hz tone (1 second).
    KR: 200 Hz 톤 생성 (1초)."""
    t = np.arange(default_config.sample_rate) / default_config.sample_rate
    return np.sin(2 * np.pi * 200 * t).astype(np.float32)


@pytest.fixture
def sample_baseline_file(
    temp_directory: Path, default_config: DspConfig, sample_tone_200hz: np.ndarray
) -> Path:
    """Create temporary baseline npz file.
    KR: 임시 baseline npz 파일 생성."""
    path = temp_directory / "baseline.npz"
    np.savez(
        path,
        error=sample_tone_200hz,
        mic_in=np.column_stack([sample_tone_200hz] * 2),
        x_ref=sample_tone_200hz,
        sample_rate=default_config.sample_rate,
        block_size=default_config.block_size,
    )
    return path


@pytest.fixture
def sample_anc_file(
    temp_directory: Path, default_config: DspConfig, sample_tone_200hz: np.ndarray
) -> Path:
    """Create temporary ANC npz file with reduced signal.
    KR: 감소된 신호가 있는 임시 ANC npz 파일 생성."""
    path = temp_directory / "anc.npz"
    reduced_signal = sample_tone_200hz * 0.1  # 10x reduction
    np.savez(
        path,
        error=reduced_signal,
        mic_in=np.column_stack([reduced_signal] * 2),
        x_ref=reduced_signal,
        y_out=np.zeros_like(reduced_signal),
        w_history=np.zeros((10, 256), dtype=np.float32),
        s_hat=np.zeros(256, dtype=np.float32),
        sample_rate=default_config.sample_rate,
        block_size=default_config.block_size,
        mu=1e-4,
        use_nlms=True,
        w_log_every=10,
    )
    return path
