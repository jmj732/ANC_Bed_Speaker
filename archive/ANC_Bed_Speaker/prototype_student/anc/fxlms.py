"""
FxLMS controller and runtime.
KR: FxLMS 컨트롤러 및 실행 루틴.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

import numpy as np
import sounddevice as sd
from numpy.typing import NDArray

from .dsp import DspConfig, FIRFilter, RingBuffer, build_bandpass_taps

# Constants
MIN_INPUT_CHANNELS: int = 1  # Feedback ANC: 마이크 1개
OUTPUT_CHANNELS: int = 2  # Stereo output

# Type aliases
Float32Array = NDArray[np.float32]

# Configure logging
logger = logging.getLogger(__name__)


class FxLMSController:
    """FxLMS controller with optional NLMS and output limiter.
    KR: 선택적 NLMS 및 출력 리미터를 갖는 FxLMS 컨트롤러."""

    def __init__(
        self,
        cfg: DspConfig,
        s_hat: Float32Array,
        use_nlms: bool = True,
        w_log_every: int = 10,
    ) -> None:
        self.cfg = cfg
        self.use_nlms = use_nlms
        self.w_log_every = max(1, int(w_log_every))

        self.w: Float32Array = np.zeros(cfg.w_len, dtype=np.float32)
        self.x_hist = RingBuffer(cfg.w_len, dtype=np.float32)
        self.xf_hist = RingBuffer(cfg.w_len, dtype=np.float32)
        self.y_hist = RingBuffer(cfg.s_len, dtype=np.float32)  # feedback ANC: 출력 이력
        self.s_hat: Float32Array = s_hat.astype(np.float32, copy=False)
        self.bandpass = FIRFilter(build_bandpass_taps(cfg))

        self._block_index: int = 0
        self._total_samples: int = 0
        self._max_log_samples: int = cfg.max_log_samples

        # Pre-allocate log lists with capacity hint
        self.mic_log: list[Float32Array] = []
        self.ref_log: list[Float32Array] = []
        self.err_log: list[Float32Array] = []
        self.y_log: list[Float32Array] = []
        self.w_log: list[Float32Array] = []

        # Status tracking
        self._status_count: int = 0

    def callback(
        self,
        indata: Float32Array,
        outdata: Float32Array,
        frames: int,
        time_info: Any,
        status: sd.CallbackFlags,
    ) -> None:
        """Audio stream callback for real-time processing.
        KR: 실시간 처리를 위한 오디오 스트림 콜백."""
        if status:
            self._status_count += 1
            logger.warning("Stream status: %s (count: %d)", status, self._status_count)

        # Feedback ANC: 마이크 1개 (CH0)
        mic_ch0: Float32Array = indata[:, 0].astype(np.float32, copy=False)

        y_out: Float32Array = np.zeros(frames, dtype=np.float32)
        e_hat_block: Float32Array = np.zeros(frames, dtype=np.float32)

        for i in range(frames):
            # 과거 출력으로 스피커→마이크 기여 예측
            y_seg1, y_seg2, y_seg1_len = self.y_hist.segments()
            predicted = (np.dot(self.s_hat[:y_seg1_len], y_seg1) +
                         np.dot(self.s_hat[y_seg1_len:], y_seg2))

            # 추정 소음 = 마이크 - 예측된 스피커 기여
            e_hat = mic_ch0[i] - predicted
            e_hat_block[i] = e_hat

            # 추정 소음을 레퍼런스로 사용
            self.x_hist.push(e_hat)
            seg1, seg2, seg1_len = self.x_hist.segments()

            y = np.dot(self.w[:seg1_len], seg1) + np.dot(self.w[seg1_len:], seg2)
            y = float(np.tanh(self.cfg.limiter_drive * y))
            y_out[i] = y

            # 출력 이력 갱신
            self.y_hist.push(y)

            # 2차 경로 필터링된 레퍼런스
            xf_sample = (np.dot(self.s_hat[:seg1_len], seg1) +
                         np.dot(self.s_hat[seg1_len:], seg2))
            self.xf_hist.push(xf_sample)

            # 적응 필터 업데이트
            xf_seg1, xf_seg2, xf_seg1_len = self.xf_hist.segments()
            norm = self.cfg.eps + self.xf_hist.sumsq()
            gain = self.cfg.mu * e_hat
            if self.use_nlms:
                gain /= norm
            self.w[:xf_seg1_len] -= gain * xf_seg1
            self.w[xf_seg1_len:] -= gain * xf_seg2

        outdata[:] = np.column_stack([y_out, y_out])

        if self._total_samples < self._max_log_samples:
            self.mic_log.append(mic_ch0.copy())
            self.ref_log.append(e_hat_block.copy())
            self.err_log.append(e_hat_block.copy())
            self.y_log.append(y_out.copy())
            self._total_samples += frames

        if self._block_index % self.w_log_every == 0:
            self.w_log.append(self.w.copy())
        self._block_index += 1

    def get_stats(self) -> dict[str, Any]:
        """Get controller statistics.
        KR: 컨트롤러 통계 반환."""
        return {
            "blocks_processed": self._block_index,
            "total_samples": self._total_samples,
            "status_warnings": self._status_count,
            "log_full": self._total_samples >= self._max_log_samples,
        }


def run_fx_lms(
    cfg: DspConfig,
    device: int | str | tuple[int | None, int | None] | None,
    s_hat_path: str | Path,
    log_path: str | Path,
    duration_s: float,
    use_nlms: bool,
    w_log_every: int,
) -> None:
    """Run ANC controller and log signals/weights.
    KR: ANC 컨트롤러 실행 및 신호/가중치 로그 저장."""
    # Validate inputs
    cfg.validate()
    s_hat_path = Path(s_hat_path)
    log_path = Path(log_path)

    if not s_hat_path.exists():
        raise FileNotFoundError(f"Secondary path file not found: {s_hat_path}")

    if duration_s is not None and duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s}")

    with np.load(s_hat_path) as data:
        s_hat: Float32Array = data["s_hat"].astype(np.float32)

    controller = FxLMSController(
        cfg=cfg, s_hat=s_hat, use_nlms=use_nlms, w_log_every=w_log_every
    )

    logger.info("Starting FxLMS ANC (NLMS=%s, duration=%.1fs)", use_nlms, duration_s or -1)

    with sd.Stream(
        samplerate=cfg.sample_rate,
        blocksize=cfg.block_size,
        dtype="float32",
        channels=(1, OUTPUT_CHANNELS),
        device=device,
        callback=controller.callback,
    ):
        if duration_s and duration_s > 0:
            sd.sleep(int(duration_s * 1000))
        else:
            print("ANC running... Press Ctrl+C to stop.")
            print("ANC 실행 중... Ctrl+C로 중지하세요.")
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass

    # Log statistics
    stats = controller.get_stats()
    logger.info("FxLMS completed: %s", stats)

    # Concatenate logs
    mic_log = np.concatenate(controller.mic_log) if controller.mic_log else np.array([])
    ref_log = np.concatenate(controller.ref_log) if controller.ref_log else np.array([])
    err_log = np.concatenate(controller.err_log) if controller.err_log else np.array([])
    y_log = np.concatenate(controller.y_log) if controller.y_log else np.array([])
    w_log = np.stack(controller.w_log) if controller.w_log else np.array([])

    np.savez(
        log_path,
        mic_in=mic_log,
        x_ref=ref_log,
        error=err_log,
        y_out=y_log,
        w_history=w_log,
        w_log_every=controller.w_log_every,
        s_hat=s_hat,
        sample_rate=cfg.sample_rate,
        block_size=cfg.block_size,
        mu=cfg.mu,
        use_nlms=use_nlms,
    )
    logger.info("Saved log to %s", log_path)
