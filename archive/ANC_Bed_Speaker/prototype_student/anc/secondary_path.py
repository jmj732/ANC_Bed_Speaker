"""
Secondary-path identification.
KR: 2차 경로 추정 모듈.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import numpy as np
import sounddevice as sd
from numpy.typing import NDArray

from .dsp import DspConfig, RingBuffer, db_to_linear

# Constants
MIN_INPUT_CHANNELS: int = 1  # 마이크 1개
OUTPUT_CHANNELS: int = 2
ERROR_MIC_CHANNEL: int = 0

# Type aliases
Float32Array = NDArray[np.float32]

# Configure logging
logger = logging.getLogger(__name__)


def estimate_secondary_path(
    cfg: DspConfig,
    device: int | str | tuple[int | None, int | None] | None,
    log_path: str | Path,
) -> Float32Array:
    """Estimate secondary-path impulse response S_hat using white noise and NLMS.
    KR: 백색잡음과 NLMS로 2차 경로 임펄스 응답 S_hat 추정.

    Args:
        cfg: DSP configuration
        device: Audio device identifier
        log_path: Path to save results

    Returns:
        Estimated secondary path impulse response
    """
    # Validate inputs
    cfg.validate()
    log_path = Path(log_path)

    if cfg.secondary_duration_s <= 0:
        raise ValueError(
            f"secondary_duration_s must be positive, got {cfg.secondary_duration_s}"
        )

    taps_len = cfg.s_len
    s_hat: Float32Array = np.zeros(taps_len, dtype=np.float32)
    x_hist = RingBuffer(taps_len, dtype=np.float32)
    noise_amp = db_to_linear(cfg.noise_dbfs)
    rng = np.random.default_rng()

    # Logging with memory limit
    noise_log: list[Float32Array] = []
    err_log: list[Float32Array] = []
    total_samples: int = 0
    max_samples: int = cfg.max_log_samples
    status_count: int = 0

    def callback(
        indata: Float32Array,
        outdata: Float32Array,
        frames: int,
        time_info: Any,
        status: sd.CallbackFlags,
    ) -> None:
        nonlocal s_hat, total_samples, status_count

        if status:
            status_count += 1
            logger.warning("Stream status: %s (count: %d)", status, status_count)

        # Excite the secondary path with white noise
        # 백색잡음으로 2차 경로를 여기(Excitation)
        noise: Float32Array = rng.standard_normal(frames).astype(np.float32) * noise_amp
        outdata[:] = np.column_stack([noise, noise])

        # Error mic is channel 2
        # 에러 마이크는 채널 2
        err: Float32Array = indata[:, ERROR_MIC_CHANNEL].astype(np.float32, copy=False)

        for i in range(frames):
            # Push sample into ring buffer and compute filter output
            # 링버퍼에 샘플 추가 후 필터 출력 계산
            x_hist.push(noise[i])
            seg1, seg2, seg1_len = x_hist.segments()
            y_hat = np.dot(s_hat[:seg1_len], seg1) + np.dot(s_hat[seg1_len:], seg2)
            e = err[i] - y_hat

            # NLMS update using ring-buffered regressor
            # 링버퍼 기반 회귀 벡터로 NLMS 갱신
            norm = cfg.eps + x_hist.sumsq()
            gain = (cfg.secondary_mu * e) / norm
            s_hat[:seg1_len] += gain * seg1
            s_hat[seg1_len:] += gain * seg2

        # Log with memory limit
        if total_samples < max_samples:
            noise_log.append(noise.copy())
            err_log.append(err.copy())
            total_samples += frames

    logger.info(
        "Starting secondary path estimation (duration=%.1fs, mu=%.4f)",
        cfg.secondary_duration_s,
        cfg.secondary_mu,
    )

    with sd.Stream(
        samplerate=cfg.sample_rate,
        blocksize=cfg.block_size,
        dtype="float32",
        channels=(1, OUTPUT_CHANNELS),
        device=device,
        callback=callback,
    ):
        sd.sleep(int(cfg.secondary_duration_s * 1000))

    logger.info("Secondary path estimation completed (status_warnings=%d)", status_count)

    # Save results
    noise_arr = np.concatenate(noise_log) if noise_log else np.array([])
    err_arr = np.concatenate(err_log) if err_log else np.array([])

    np.savez(
        log_path,
        s_hat=s_hat,
        noise=noise_arr,
        error=err_arr,
        sample_rate=cfg.sample_rate,
        block_size=cfg.block_size,
        duration_s=cfg.secondary_duration_s,
        mu=cfg.secondary_mu,
    )
    logger.info("Saved secondary path to %s", log_path)

    return s_hat
