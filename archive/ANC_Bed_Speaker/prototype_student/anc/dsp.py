"""
Core DSP utilities for ANC.
KR: ANC용 DSP 유틸리티 모음.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from scipy.signal import firwin, lfilter

if TYPE_CHECKING:
    from typing import Tuple

# Type aliases for clarity
Float32Array = NDArray[np.float32]
Float64Array = NDArray[np.float64]


def db_to_linear(db: float) -> float:
    """Convert dBFS value to linear amplitude.
    KR: dBFS 값을 선형 진폭으로 변환."""
    return 10 ** (db / 20.0)


@dataclass
class DspConfig:
    """Configuration for ANC DSP pipeline and learning.
    KR: ANC DSP 파이프라인/학습 설정."""

    sample_rate: int = 48000
    block_size: int = 64
    w_len: int = 256
    s_len: int = 256
    mu: float = 1e-4
    secondary_mu: float = 0.02
    secondary_duration_s: float = 10.0
    bandpass_low: float = 100.0
    bandpass_high: float = 300.0
    bandpass_taps: int = 129
    limiter_drive: float = 1.0
    noise_dbfs: float = -12.0
    eps: float = 1e-8
    # Memory limits for logging (max samples to store)
    max_log_samples: int = 48000 * 60 * 10  # 10 minutes max

    def validate(self) -> None:
        """Validate configuration parameters.
        KR: 설정 파라미터 검증."""
        if self.sample_rate <= 0:
            raise ValueError(f"sample_rate must be positive, got {self.sample_rate}")
        if self.block_size <= 0:
            raise ValueError(f"block_size must be positive, got {self.block_size}")
        if self.w_len <= 0:
            raise ValueError(f"w_len must be positive, got {self.w_len}")
        if self.s_len <= 0:
            raise ValueError(f"s_len must be positive, got {self.s_len}")
        if self.mu <= 0:
            raise ValueError(f"mu must be positive, got {self.mu}")
        if self.bandpass_low >= self.bandpass_high:
            raise ValueError(
                f"bandpass_low ({self.bandpass_low}) must be less than "
                f"bandpass_high ({self.bandpass_high})"
            )
        if self.bandpass_low < 0 or self.bandpass_high > self.sample_rate / 2:
            raise ValueError(
                f"bandpass frequencies must be in [0, {self.sample_rate / 2}]"
            )


class FIRFilter:
    """Stateful FIR filter wrapper using scipy.signal.lfilter.
    KR: scipy.signal.lfilter 기반 상태 유지 FIR 필터."""

    def __init__(self, taps: Float32Array | Float64Array) -> None:
        self.taps: Float32Array = np.asarray(taps, dtype=np.float32)
        self.zi: Float32Array = np.zeros(len(self.taps) - 1, dtype=np.float32)

    def process(self, x_block: Float32Array) -> Float32Array:
        """Filter block with internal state for continuity.
        KR: 블록 간 연속성을 위해 내부 상태를 유지하며 필터링."""
        y, self.zi = lfilter(self.taps, [1.0], x_block, zi=self.zi)
        return y.astype(np.float32, copy=False)

    def reset(self) -> None:
        """Reset filter state.
        KR: 필터 상태 초기화."""
        self.zi[:] = 0.0


class RingBuffer:
    """Optimized ring buffer for most-recent-first vector operations.
    KR: 최신 샘플 우선 벡터 연산을 위한 최적화된 링버퍼."""

    def __init__(self, size: int, dtype: type = np.float32) -> None:
        self.buf: Float32Array = np.zeros(int(size), dtype=dtype)
        self.size: int = int(size)
        self.index: int = 0
        # Pre-allocated arrays for segments to avoid repeated allocation
        self._seg1: Float32Array = np.zeros(size, dtype=dtype)
        self._seg2: Float32Array = np.zeros(size, dtype=dtype)

    def push(self, sample: float) -> None:
        """Insert newest sample at current index.
        KR: 최신 샘플을 현재 인덱스에 삽입."""
        self.buf[self.index] = sample
        self.index = (self.index + 1) % self.size

    def push_block(self, samples: Float32Array) -> None:
        """Push multiple samples efficiently.
        KR: 여러 샘플을 효율적으로 추가."""
        n = len(samples)
        if n >= self.size:
            # If more samples than buffer size, just take the last size samples
            self.buf[:] = samples[-self.size:]
            self.index = 0
        else:
            # Calculate how many samples fit before wrap
            space_before_wrap = self.size - self.index
            if n <= space_before_wrap:
                self.buf[self.index:self.index + n] = samples
                self.index = (self.index + n) % self.size
            else:
                # Split into two parts
                self.buf[self.index:] = samples[:space_before_wrap]
                remaining = n - space_before_wrap
                self.buf[:remaining] = samples[space_before_wrap:]
                self.index = remaining

    def segments(self) -> Tuple[Float32Array, Float32Array, int]:
        """Return two segments representing newest->oldest ordering.
        KR: 최신->과거 순서를 표현하는 두 구간 반환.

        Returns pre-allocated views to avoid memory allocation in hot path.
        """
        pos = (self.index - 1) % self.size
        seg1_len = pos + 1
        # Copy to pre-allocated arrays (reversed order)
        self._seg1[:seg1_len] = self.buf[pos::-1]
        seg2_len = self.size - seg1_len
        if seg2_len > 0:
            self._seg2[:seg2_len] = self.buf[:pos:-1]
        return self._seg1[:seg1_len], self._seg2[:seg2_len], seg1_len

    def get_vector(self) -> Float32Array:
        """Get full buffer as newest-first contiguous array.
        KR: 최신 우선 연속 배열로 전체 버퍼 반환."""
        seg1, seg2, seg1_len = self.segments()
        result = np.empty(self.size, dtype=np.float32)
        result[:seg1_len] = seg1
        result[seg1_len:] = seg2
        return result

    def sumsq(self) -> float:
        """Energy of buffer contents (for NLMS normalization).
        KR: 버퍼 에너지(NLMS 정규화용)."""
        return float(np.dot(self.buf, self.buf))

    def reset(self) -> None:
        """Reset buffer to zeros.
        KR: 버퍼를 0으로 초기화."""
        self.buf[:] = 0.0
        self.index = 0


def build_bandpass_taps(cfg: DspConfig) -> Float32Array:
    """Design bandpass FIR taps for the reference mic.
    KR: 레퍼런스 마이크용 대역통과 FIR 탭 설계."""
    numtaps = int(cfg.bandpass_taps)
    fs = float(cfg.sample_rate)
    low = float(cfg.bandpass_low)
    high = float(cfg.bandpass_high)

    # Use a weighted least-squares design for better stopband rejection
    # in the low-frequency regime where windowed methods struggle.
    if numtaps % 2 == 1:
        half = (numtaps - 1) // 2
        bw = high - low

        pass_freqs = np.linspace(low, high, 20)
        low_stop_max = max(0.0, low * 0.8)
        stop_low = (
            np.linspace(0.0, low_stop_max, 10)
            if low_stop_max > 0.0
            else np.array([0.0])
        )
        high_stop_start = min(fs / 2.0, high + 0.1 * bw)
        high_stop_end = min(fs / 2.0, high + 9.0 * bw)
        stop_high = (
            np.linspace(high_stop_start, high_stop_end, 20)
            if high_stop_end > high_stop_start
            else np.array([high_stop_start])
        )
        stop_crit = np.array(
            [
                max(0.0, low * 0.5),
                min(fs / 2.0, high + bw),
            ]
        )
        pass_crit = np.array([(low + high) / 2.0])

        freqs = np.concatenate([pass_freqs, stop_low, stop_high, stop_crit, pass_crit])
        desired = np.concatenate(
            [
                np.ones_like(pass_freqs),
                np.zeros_like(stop_low),
                np.zeros_like(stop_high),
                np.zeros_like(stop_crit),
                np.ones_like(pass_crit),
            ]
        )
        weights = np.concatenate(
            [
                np.ones_like(pass_freqs) * 1.0,
                np.ones_like(stop_low) * 5.0,
                np.ones_like(stop_high) * 5.0,
                np.ones_like(stop_crit) * 20.0,
                np.ones_like(pass_crit) * 1.0,
            ]
        )

        w = 2.0 * np.pi * freqs / fs
        A = np.zeros((len(freqs), half + 1), dtype=np.float64)
        A[:, half] = 1.0
        k = np.arange(half, dtype=np.float64)
        A[:, :half] = 2.0 * np.cos(w[:, None] * (half - k))

        Aw = A * weights[:, None]
        bw_vec = desired * weights
        # Ridge regularization keeps coefficients well-scaled for float32 output.
        lam = 1e-6
        coeffs = np.linalg.solve(
            Aw.T @ Aw + lam * np.eye(half + 1),
            Aw.T @ bw_vec,
        )

        taps = np.zeros(numtaps, dtype=np.float64)
        taps[:half] = coeffs[:half]
        taps[half] = coeffs[half]
        taps[half + 1 :] = coeffs[:half][::-1]

        # Normalize to unity gain at band center.
        w0 = 2.0 * np.pi * (low + high) / (2.0 * fs)
        center_gain = coeffs[half] + 2.0 * np.sum(
            coeffs[:half] * np.cos(w0 * (half - k))
        )
        if center_gain != 0.0:
            taps /= abs(center_gain)
        return taps.astype(np.float32, copy=False)

    # Fallback for even-length filters.
    return firwin(
        numtaps=numtaps,
        cutoff=[low, high],
        fs=fs,
        pass_zero=False,
    ).astype(np.float32)
