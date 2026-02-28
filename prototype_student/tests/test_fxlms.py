"""
Tests for anc/fxlms.py module.
KR: anc/fxlms.py 모듈 테스트.
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest

from prototype_student.anc.dsp import DspConfig
from prototype_student.anc.fxlms import FxLMSController, run_fx_lms


class TestFxLMSController:
    """Tests for FxLMSController class."""

    @pytest.fixture
    def cfg(self) -> DspConfig:
        """Create test configuration."""
        return DspConfig(
            sample_rate=48000,
            block_size=64,
            w_len=32,  # Smaller for testing
            s_len=32,
            mu=1e-4,
        )

    @pytest.fixture
    def s_hat(self) -> np.ndarray:
        """Create test secondary path."""
        # Simple impulse response (delayed delta)
        s = np.zeros(32, dtype=np.float32)
        s[5] = 1.0  # 5-sample delay
        return s

    def test_creation(self, cfg: DspConfig, s_hat: np.ndarray) -> None:
        """Test controller creation."""
        controller = FxLMSController(cfg, s_hat)

        assert controller.w.shape == (32,)
        assert controller.s_hat.shape == (32,)
        assert controller.use_nlms is True
        assert len(controller.mic_log) == 0

    def test_creation_no_nlms(self, cfg: DspConfig, s_hat: np.ndarray) -> None:
        """Test controller creation without NLMS."""
        controller = FxLMSController(cfg, s_hat, use_nlms=False)
        assert controller.use_nlms is False

    def test_callback_shape(self, cfg: DspConfig, s_hat: np.ndarray) -> None:
        """Test callback with correct input/output shapes."""
        controller = FxLMSController(cfg, s_hat)

        # Simulate input: 3 channels (2 ref + 1 error)
        frames = cfg.block_size
        indata = np.zeros((frames, 3), dtype=np.float32)
        outdata = np.zeros((frames, 2), dtype=np.float32)

        # Run callback
        controller.callback(indata, outdata, frames, None, None)

        # Check output is populated
        assert outdata.shape == (frames, 2)
        # Logs should be populated
        assert len(controller.mic_log) == 1
        assert len(controller.ref_log) == 1
        assert len(controller.err_log) == 1
        assert len(controller.y_log) == 1

    def test_callback_insufficient_channels(
        self, cfg: DspConfig, s_hat: np.ndarray
    ) -> None:
        """Test callback handles insufficient input channels."""
        controller = FxLMSController(cfg, s_hat)

        # Only 2 channels instead of required 3
        frames = cfg.block_size
        indata = np.zeros((frames, 2), dtype=np.float32)
        outdata = np.zeros((frames, 2), dtype=np.float32)

        # Should not raise, but output should be zero
        controller.callback(indata, outdata, frames, None, None)
        assert np.all(outdata == 0.0)

    def test_weight_update(self, cfg: DspConfig, s_hat: np.ndarray) -> None:
        """Test that weights are updated during callback."""
        controller = FxLMSController(cfg, s_hat)

        frames = cfg.block_size
        # Create non-zero input
        indata = np.random.randn(frames, 3).astype(np.float32) * 0.1
        outdata = np.zeros((frames, 2), dtype=np.float32)

        # Initial weights are zero
        assert np.all(controller.w == 0)

        # Run callback
        controller.callback(indata, outdata, frames, None, None)

        # Weights should be updated (non-zero)
        assert not np.all(controller.w == 0)

    def test_get_stats(self, cfg: DspConfig, s_hat: np.ndarray) -> None:
        """Test statistics retrieval."""
        controller = FxLMSController(cfg, s_hat)

        # Initial stats
        stats = controller.get_stats()
        assert stats["blocks_processed"] == 0
        assert stats["total_samples"] == 0
        assert stats["status_warnings"] == 0
        assert stats["log_full"] is False

        # Process one block
        frames = cfg.block_size
        indata = np.zeros((frames, 3), dtype=np.float32)
        outdata = np.zeros((frames, 2), dtype=np.float32)
        controller.callback(indata, outdata, frames, None, None)

        # Updated stats
        stats = controller.get_stats()
        assert stats["blocks_processed"] == 1
        assert stats["total_samples"] == frames

    def test_memory_limit(self, cfg: DspConfig, s_hat: np.ndarray) -> None:
        """Test that logging respects memory limit."""
        cfg.max_log_samples = 100  # Very small limit
        controller = FxLMSController(cfg, s_hat)

        frames = cfg.block_size
        indata = np.zeros((frames, 3), dtype=np.float32)
        outdata = np.zeros((frames, 2), dtype=np.float32)

        # Process multiple blocks
        for _ in range(10):
            controller.callback(indata, outdata, frames, None, None)

        # Check that logging stopped at limit
        stats = controller.get_stats()
        assert stats["total_samples"] <= cfg.max_log_samples + cfg.block_size
        assert stats["log_full"] is True


class TestRunFxLms:
    """Integration tests for run_fx_lms function."""

    @pytest.fixture
    def cfg(self) -> DspConfig:
        """Create test configuration."""
        return DspConfig()

    @pytest.fixture
    def temp_dir(self) -> TemporaryDirectory:
        """Create temporary directory."""
        return TemporaryDirectory()

    def test_file_not_found(self, cfg: DspConfig, temp_dir: TemporaryDirectory) -> None:
        """Test error when secondary path file doesn't exist."""
        with pytest.raises(FileNotFoundError):
            run_fx_lms(
                cfg,
                device=None,
                s_hat_path=Path(temp_dir.name) / "nonexistent.npz",
                log_path=Path(temp_dir.name) / "output.npz",
                duration_s=0.1,
                use_nlms=True,
                w_log_every=10,
            )

    def test_negative_duration(self, cfg: DspConfig, temp_dir: TemporaryDirectory) -> None:
        """Test error with negative duration."""
        # Create dummy secondary path file
        s_hat = np.zeros(256, dtype=np.float32)
        s_hat_path = Path(temp_dir.name) / "s_hat.npz"
        np.savez(s_hat_path, s_hat=s_hat)

        with pytest.raises(ValueError, match="duration_s must be non-negative"):
            run_fx_lms(
                cfg,
                device=None,
                s_hat_path=s_hat_path,
                log_path=Path(temp_dir.name) / "output.npz",
                duration_s=-1.0,
                use_nlms=True,
                w_log_every=10,
            )
