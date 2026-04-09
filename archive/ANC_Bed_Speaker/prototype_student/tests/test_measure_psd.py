"""
Tests for tools/measure_psd.py module.
KR: tools/measure_psd.py 모듈 테스트.
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest

from prototype_student.tools.measure_psd import (
    band_power_db,
    compute_attenuation,
    load_error_signal,
)


class TestBandPowerDb:
    """Tests for band_power_db function."""

    def test_simple_band(self) -> None:
        """Test band power calculation with simple data."""
        freqs = np.array([0, 100, 200, 300, 400, 500], dtype=np.float64)
        psd = np.array([1, 1, 1, 1, 1, 1], dtype=np.float64)

        # Band 100-300 Hz should integrate to 200 (area under curve)
        result = band_power_db(freqs, psd, (100.0, 300.0))

        # 10 * log10(200) ≈ 23.01 dB
        assert result == pytest.approx(23.01, rel=0.01)

    def test_empty_band(self) -> None:
        """Test with band outside frequency range."""
        freqs = np.array([0, 100, 200], dtype=np.float64)
        psd = np.array([1, 1, 1], dtype=np.float64)

        result = band_power_db(freqs, psd, (500.0, 600.0))

        assert result == float("-inf")

    def test_narrow_band(self) -> None:
        """Test with very narrow band."""
        freqs = np.linspace(0, 1000, 1001, dtype=np.float64)
        psd = np.ones_like(freqs)

        result = band_power_db(freqs, psd, (199.0, 201.0))

        # Should get approximately 2 Hz bandwidth
        assert result == pytest.approx(10 * np.log10(2), rel=0.1)


class TestLoadErrorSignal:
    """Tests for load_error_signal function."""

    def test_load_valid_file(self) -> None:
        """Test loading valid npz file."""
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "test.npz"
            error = np.array([1.0, 2.0, 3.0], dtype=np.float32)
            np.savez(path, error=error, sample_rate=48000)

            loaded_error, fs = load_error_signal(path)

            assert np.allclose(loaded_error, error)
            assert fs == 48000

    def test_file_not_found(self) -> None:
        """Test error when file doesn't exist."""
        with pytest.raises(FileNotFoundError):
            load_error_signal("/nonexistent/path.npz")

    def test_missing_error_key(self) -> None:
        """Test error when 'error' key is missing."""
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "test.npz"
            np.savez(path, other_data=[1, 2, 3], sample_rate=48000)

            with pytest.raises(KeyError, match="error"):
                load_error_signal(path)

    def test_missing_sample_rate_key(self) -> None:
        """Test error when 'sample_rate' key is missing."""
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "test.npz"
            np.savez(path, error=[1, 2, 3])

            with pytest.raises(KeyError, match="sample_rate"):
                load_error_signal(path)


class TestComputeAttenuation:
    """Tests for compute_attenuation function."""

    @pytest.fixture
    def create_test_files(self) -> callable:
        """Factory to create test npz files."""

        def _create(
            temp_dir: str,
            baseline_signal: np.ndarray,
            anc_signal: np.ndarray,
            sample_rate: int = 48000,
        ) -> tuple[Path, Path]:
            baseline_path = Path(temp_dir) / "baseline.npz"
            anc_path = Path(temp_dir) / "anc.npz"

            np.savez(baseline_path, error=baseline_signal, sample_rate=sample_rate)
            np.savez(anc_path, error=anc_signal, sample_rate=sample_rate)

            return baseline_path, anc_path

        return _create

    def test_compute_attenuation_identical(self, create_test_files: callable) -> None:
        """Test attenuation is 0 dB for identical signals."""
        with TemporaryDirectory() as temp_dir:
            # Create identical signals
            signal = np.random.randn(48000).astype(np.float32)
            baseline_path, anc_path = create_test_files(temp_dir, signal, signal)

            result = compute_attenuation(baseline_path, anc_path)

            assert result["reduction_db"] == pytest.approx(0.0, abs=0.1)

    def test_compute_attenuation_reduced(self, create_test_files: callable) -> None:
        """Test positive attenuation when ANC signal is reduced."""
        with TemporaryDirectory() as temp_dir:
            # Create test signals (48kHz, 1 second)
            fs = 48000
            t = np.arange(fs) / fs
            # 200 Hz tone in target band
            baseline = (np.sin(2 * np.pi * 200 * t)).astype(np.float32)
            # Reduced amplitude for ANC
            anc = (0.1 * np.sin(2 * np.pi * 200 * t)).astype(np.float32)

            baseline_path, anc_path = create_test_files(temp_dir, baseline, anc)

            result = compute_attenuation(baseline_path, anc_path)

            # 10x reduction = 20 dB
            assert result["reduction_db"] == pytest.approx(20.0, abs=1.0)

    def test_sample_rate_mismatch(self, create_test_files: callable) -> None:
        """Test error when sample rates don't match."""
        with TemporaryDirectory() as temp_dir:
            baseline_path = Path(temp_dir) / "baseline.npz"
            anc_path = Path(temp_dir) / "anc.npz"

            np.savez(baseline_path, error=np.zeros(1000), sample_rate=48000)
            np.savez(anc_path, error=np.zeros(1000), sample_rate=44100)

            with pytest.raises(ValueError, match="Sample rates differ"):
                compute_attenuation(baseline_path, anc_path)

    def test_custom_band(self, create_test_files: callable) -> None:
        """Test with custom frequency band."""
        with TemporaryDirectory() as temp_dir:
            signal = np.random.randn(48000).astype(np.float32)
            baseline_path, anc_path = create_test_files(temp_dir, signal, signal)

            result = compute_attenuation(
                baseline_path, anc_path, band=(200.0, 400.0)
            )

            assert result["band"] == (200.0, 400.0)
            assert result["reduction_db"] == pytest.approx(0.0, abs=0.1)
