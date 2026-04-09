"""
Baseline recording.
KR: 기준 신호(ANC off) 기록.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import numpy as np
import sounddevice as sd
from numpy.typing import NDArray

from .dsp import DspConfig

# Constants
MIN_INPUT_CHANNELS: int = 2  # 1 ref + 1 error
OUTPUT_CHANNELS: int = 2

# Type aliases
Float32Array = NDArray[np.float32]

# Configure logging
logger = logging.getLogger(__name__)


def record_baseline(
    cfg: DspConfig,
    device: int | str | tuple[int | None, int | None] | None,
    log_path: str | Path,
    duration_s: float,
    *,
    latency: str | float = "low",
) -> None:
    """Record baseline (ANC off) for comparison.
    KR: 비교를 위한 기준 신호(ANC off) 기록.

    Args:
        cfg: DSP configuration
        device: Audio device identifier
        log_path: Path to save results
        duration_s: Recording duration in seconds (None for indefinite)
        latency: Stream latency hint ('low', 'high', or seconds)
    """
    # Validate inputs
    cfg.validate()
    log_path = Path(log_path)

    if duration_s is not None and duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s}")

    # Logging with memory limit
    mic_log: list[Float32Array] = []
    ref_log: list[Float32Array] = []
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
        nonlocal total_samples, status_count

        if status:
            status_count += 1
            logger.warning("Stream status: %s (count: %d)", status, status_count)

        # Silence output
        outdata[:] = 0.0

        n_ch = indata.shape[1]
        mic_in = indata[:, :min(n_ch, MIN_INPUT_CHANNELS)].astype(np.float32, copy=False)
        x_raw = mic_in[:, 0]

        # 마이크 1개: ch0을 ref+error 둘 다 사용
        err_sig = x_raw
        mic_pair = np.column_stack([x_raw, x_raw])

        # Log with memory limit
        if total_samples < max_samples:
            mic_log.append(mic_pair.copy())
            ref_log.append(x_raw.copy())
            err_log.append(err_sig.copy())
            total_samples += frames

    # Detect available input channels for the device
    try:
        dev_info = sd.query_devices(device if not isinstance(device, tuple) else device[0])
        available_in = int(dev_info["max_input_channels"])
    except Exception:
        available_in = MIN_INPUT_CHANNELS
    in_channels = min(available_in, MIN_INPUT_CHANNELS)
    if in_channels < MIN_INPUT_CHANNELS:
        logger.warning(
            "Mono test mode: device has %d input channel(s), need %d. "
            "Error mic will mirror reference mic.",
            in_channels, MIN_INPUT_CHANNELS,
        )

    logger.info(
        "Starting baseline recording (duration=%.1fs, block_size=%d, latency=%s, in_ch=%d)",
        duration_s or -1, cfg.block_size, latency, in_channels,
    )

    with sd.Stream(
        samplerate=cfg.sample_rate,
        blocksize=cfg.block_size,
        dtype="float32",
        channels=(in_channels, OUTPUT_CHANNELS),
        device=device,
        latency=latency,
        callback=callback,
    ):
        if duration_s and duration_s > 0:
            sd.sleep(int(duration_s * 1000))
        else:
            print("Baseline recording... Press Ctrl+C to stop.")
            print("기준 신호 기록 중... Ctrl+C로 중지하세요.")
            try:
                while True:
                    sd.sleep(500)
            except KeyboardInterrupt:
                pass

    logger.info("Baseline recording completed (status_warnings=%d)", status_count)

    # Save results
    mic_arr = np.concatenate(mic_log) if mic_log else np.array([])
    ref_arr = np.concatenate(ref_log) if ref_log else np.array([])
    err_arr = np.concatenate(err_log) if err_log else np.array([])

    np.savez(
        log_path,
        mic_in=mic_arr,
        x_ref=ref_arr,
        error=err_arr,
        sample_rate=cfg.sample_rate,
        block_size=cfg.block_size,
    )
    logger.info("Saved baseline to %s", log_path)
