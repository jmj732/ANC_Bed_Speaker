"""
Tests for anc/dsp.py module.
KR: anc/dsp.py 모듈 테스트.
"""

from __future__ import annotations

import numpy as np
import pytest

from prototype_student.anc.dsp import (
    DspConfig,
    FIRFilter,
    RingBuffer,
    build_bandpass_taps,
    db_to_linear,
)


class TestDbToLinear:
    """Tests for db_to_linear function."""

    def test_zero_db(self) -> None:
        """0 dB should return 1.0."""
        assert db_to_linear(0.0) == pytest.approx(1.0)

    def test_minus_6db(self) -> None:
        """-6 dB should return approximately 0.5."""
        assert db_to_linear(-6.0) == pytest.approx(0.5011872, rel=1e-4)

    def test_minus_20db(self) -> None:
        """-20 dB should return 0.1."""
        assert db_to_linear(-20.0) == pytest.approx(0.1)

    def test_positive_6db(self) -> None:
        """+6 dB should return approximately 2.0."""
        assert db_to_linear(6.0) == pytest.approx(1.9952623, rel=1e-4)


class TestDspConfig:
    """Tests for DspConfig dataclass."""

    def test_default_values(self) -> None:
        """Test default configuration values."""
        cfg = DspConfig()
        assert cfg.sample_rate == 48000
        assert cfg.block_size == 64
        assert cfg.w_len == 256
        assert cfg.s_len == 256
        assert cfg.bandpass_low == 100.0
        assert cfg.bandpass_high == 300.0

    def test_validate_success(self) -> None:
        """Test validation passes for valid config."""
        cfg = DspConfig()
        cfg.validate()  # Should not raise

    def test_validate_negative_sample_rate(self) -> None:
        """Test validation fails for negative sample rate."""
        cfg = DspConfig(sample_rate=-1)
        with pytest.raises(ValueError, match="sample_rate must be positive"):
            cfg.validate()

    def test_validate_zero_block_size(self) -> None:
        """Test validation fails for zero block size."""
        cfg = DspConfig(block_size=0)
        with pytest.raises(ValueError, match="block_size must be positive"):
            cfg.validate()

    def test_validate_invalid_bandpass(self) -> None:
        """Test validation fails when bandpass_low >= bandpass_high."""
        cfg = DspConfig(bandpass_low=300.0, bandpass_high=100.0)
        with pytest.raises(ValueError, match="bandpass_low"):
            cfg.validate()

    def test_validate_bandpass_exceeds_nyquist(self) -> None:
        """Test validation fails when bandpass exceeds Nyquist frequency."""
        cfg = DspConfig(bandpass_high=30000.0)  # > 48000/2
        with pytest.raises(ValueError, match="bandpass frequencies"):
            cfg.validate()


class TestFIRFilter:
    """Tests for FIRFilter class."""

    def test_creation(self) -> None:
        """Test FIR filter creation."""
        taps = np.array([0.25, 0.5, 0.25], dtype=np.float32)
        fir = FIRFilter(taps)
        assert len(fir.taps) == 3
        assert fir.zi.shape == (2,)

    def test_process_identity(self) -> None:
        """Test processing with identity-like filter."""
        # Simple delay filter: [0, 1, 0]
        taps = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        fir = FIRFilter(taps)

        x = np.array([1.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        y = fir.process(x)

        # Output should be delayed by 1 sample
        assert y.shape == x.shape
        assert y[0] == pytest.approx(0.0)
        assert y[1] == pytest.approx(1.0)

    def test_process_continuity(self) -> None:
        """Test that filter maintains state across blocks."""
        taps = np.array([0.5, 0.5], dtype=np.float32)
        fir = FIRFilter(taps)

        # Process two blocks
        x1 = np.array([1.0, 0.0], dtype=np.float32)
        x2 = np.array([0.0, 0.0], dtype=np.float32)

        y1 = fir.process(x1)
        y2 = fir.process(x2)

        # Second block should see effect of first block's last sample
        assert y2[0] == pytest.approx(0.0)  # 0.5 * 0 + 0.5 * 0

    def test_reset(self) -> None:
        """Test filter reset."""
        taps = np.array([0.5, 0.5], dtype=np.float32)
        fir = FIRFilter(taps)

        # Process some data
        fir.process(np.array([1.0, 1.0], dtype=np.float32))
        assert np.any(fir.zi != 0)

        # Reset and verify
        fir.reset()
        assert np.all(fir.zi == 0)


class TestRingBuffer:
    """Tests for RingBuffer class."""

    def test_creation(self) -> None:
        """Test ring buffer creation."""
        rb = RingBuffer(size=10)
        assert rb.size == 10
        assert len(rb.buf) == 10
        assert rb.index == 0

    def test_push_single(self) -> None:
        """Test pushing single samples."""
        rb = RingBuffer(size=4)
        rb.push(1.0)
        rb.push(2.0)
        rb.push(3.0)

        assert rb.index == 3
        assert rb.buf[0] == pytest.approx(1.0)
        assert rb.buf[1] == pytest.approx(2.0)
        assert rb.buf[2] == pytest.approx(3.0)

    def test_push_wrap_around(self) -> None:
        """Test wrap-around behavior."""
        rb = RingBuffer(size=3)
        for i in range(5):
            rb.push(float(i))

        # After 5 pushes in size-3 buffer: [3, 4, 2]
        assert rb.buf[0] == pytest.approx(3.0)
        assert rb.buf[1] == pytest.approx(4.0)
        assert rb.buf[2] == pytest.approx(2.0)

    def test_push_block(self) -> None:
        """Test pushing multiple samples at once."""
        rb = RingBuffer(size=5)
        samples = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        rb.push_block(samples)

        assert rb.buf[0] == pytest.approx(1.0)
        assert rb.buf[1] == pytest.approx(2.0)
        assert rb.buf[2] == pytest.approx(3.0)

    def test_push_block_larger_than_buffer(self) -> None:
        """Test pushing block larger than buffer size."""
        rb = RingBuffer(size=3)
        samples = np.array([1.0, 2.0, 3.0, 4.0, 5.0], dtype=np.float32)
        rb.push_block(samples)

        # Should contain last 3 samples
        assert set(rb.buf.tolist()) == {3.0, 4.0, 5.0}

    def test_segments(self) -> None:
        """Test segment retrieval."""
        rb = RingBuffer(size=4)
        rb.push(1.0)
        rb.push(2.0)
        rb.push(3.0)

        seg1, seg2, seg1_len = rb.segments()

        # seg1 should be [3, 2, 1] (newest to oldest up to index)
        assert seg1_len == 3
        assert seg1[0] == pytest.approx(3.0)
        assert seg1[1] == pytest.approx(2.0)
        assert seg1[2] == pytest.approx(1.0)

    def test_sumsq(self) -> None:
        """Test sum of squares calculation."""
        rb = RingBuffer(size=3)
        rb.push(1.0)
        rb.push(2.0)
        rb.push(3.0)

        # 1^2 + 2^2 + 3^2 = 14
        assert rb.sumsq() == pytest.approx(14.0)

    def test_reset(self) -> None:
        """Test buffer reset."""
        rb = RingBuffer(size=3)
        rb.push(1.0)
        rb.push(2.0)

        rb.reset()

        assert rb.index == 0
        assert np.all(rb.buf == 0)

    def test_get_vector(self) -> None:
        """Test getting full buffer as contiguous array."""
        rb = RingBuffer(size=4)
        rb.push(1.0)
        rb.push(2.0)
        rb.push(3.0)

        vec = rb.get_vector()

        # Should be newest-first: [3, 2, 1, 0]
        assert vec[0] == pytest.approx(3.0)
        assert vec[1] == pytest.approx(2.0)
        assert vec[2] == pytest.approx(1.0)
        assert vec[3] == pytest.approx(0.0)


class TestBuildBandpassTaps:
    """Tests for build_bandpass_taps function."""

    def test_taps_length(self) -> None:
        """Test that taps have correct length."""
        cfg = DspConfig(bandpass_taps=129)
        taps = build_bandpass_taps(cfg)
        assert len(taps) == 129

    def test_taps_dtype(self) -> None:
        """Test that taps are float32."""
        cfg = DspConfig()
        taps = build_bandpass_taps(cfg)
        assert taps.dtype == np.float32

    def test_taps_symmetry(self) -> None:
        """Test that taps are symmetric (linear phase)."""
        cfg = DspConfig(bandpass_taps=129)
        taps = build_bandpass_taps(cfg)
        assert np.allclose(taps, taps[::-1], atol=1e-6)

    def test_frequency_response(self) -> None:
        """Test that frequency response passes target band."""
        cfg = DspConfig(
            sample_rate=48000,
            bandpass_low=100.0,
            bandpass_high=300.0,
            bandpass_taps=129,
        )
        taps = build_bandpass_taps(cfg)

        # Compute frequency response
        from scipy.signal import freqz

        w, h = freqz(taps, worN=4096, fs=cfg.sample_rate)

        # Find response at center frequency (200 Hz)
        center_idx = np.argmin(np.abs(w - 200.0))
        center_response = np.abs(h[center_idx])

        # Find response outside band (50 Hz and 500 Hz)
        low_idx = np.argmin(np.abs(w - 50.0))
        high_idx = np.argmin(np.abs(w - 500.0))
        low_response = np.abs(h[low_idx])
        high_response = np.abs(h[high_idx])

        # Center should be significantly higher than out-of-band
        assert center_response > low_response * 10
        assert center_response > high_response * 10
