"""
Compute band-limited attenuation (100-300 Hz) between baseline and ANC logs.
KR: baseline/ANC 로그의 100-300 Hz 대역 감쇠량을 계산.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from numpy.typing import NDArray

import numpy as np
from scipy.signal import welch

# Type aliases
Float32Array = NDArray[np.float32]
Float64Array = NDArray[np.float64]

# Constants
DEFAULT_BAND: tuple[float, float] = (100.0, 300.0)
DEFAULT_NPERSEG: int = 4096

# Configure logging
logger = logging.getLogger(__name__)


def band_power_db(
    freqs: Float64Array,
    psd: Float64Array,
    band: tuple[float, float],
) -> float:
    """Integrate PSD over band and return dB power.
    KR: 대역 PSD 적분 후 dB 파워 반환.

    Args:
        freqs: Frequency array from Welch
        psd: Power spectral density array
        band: (low_freq, high_freq) tuple

    Returns:
        Band power in dB
    """
    idx = (freqs >= band[0]) & (freqs <= band[1])
    if not np.any(idx):
        return float("-inf")
    # Use np.trapezoid (numpy 2.0+) with fallback to np.trapz
    try:
        power = np.trapezoid(psd[idx], freqs[idx])
    except AttributeError:
        # Fallback for numpy < 2.0
        power = np.trapz(psd[idx], freqs[idx])
    return 10.0 * np.log10(power + 1e-12)


def load_error_signal(path: str | Path) -> tuple[Float32Array, int]:
    """Load error mic signal and sample rate from npz.
    KR: npz에서 에러 마이크 신호와 샘플레이트 로드.

    Args:
        path: Path to npz file

    Returns:
        Tuple of (error_signal, sample_rate)

    Raises:
        FileNotFoundError: If file doesn't exist
        KeyError: If required keys missing in npz
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    with np.load(path) as data:
        if "error" not in data:
            raise KeyError(f"'error' key not found in {path}")
        if "sample_rate" not in data:
            raise KeyError(f"'sample_rate' key not found in {path}")
        err = data["error"].astype(np.float32)
        fs = int(data["sample_rate"])
    return err, fs


def compute_attenuation(
    baseline_path: str | Path,
    anc_path: str | Path,
    band: tuple[float, float] = DEFAULT_BAND,
    nperseg: int = DEFAULT_NPERSEG,
) -> dict[str, float]:
    """Compute band-limited attenuation between baseline and ANC.
    KR: baseline과 ANC 간 대역 제한 감쇠량 계산.

    Args:
        baseline_path: Path to baseline npz
        anc_path: Path to ANC npz
        band: Frequency band (low, high) in Hz
        nperseg: Samples per segment for Welch

    Returns:
        Dict with baseline_db, anc_db, reduction_db
    """
    baseline_err, fs = load_error_signal(baseline_path)
    anc_err, fs_anc = load_error_signal(anc_path)

    if fs != fs_anc:
        raise ValueError(
            f"Sample rates differ: baseline={fs}, anc={fs_anc}"
        )

    # Compute PSD estimates
    freqs, psd_base = welch(baseline_err, fs=fs, nperseg=nperseg, scaling="density")
    _, psd_anc = welch(anc_err, fs=fs, nperseg=nperseg, scaling="density")

    base_db = band_power_db(freqs, psd_base, band)
    anc_db = band_power_db(freqs, psd_anc, band)
    reduction = base_db - anc_db

    return {
        "baseline_db": base_db,
        "anc_db": anc_db,
        "reduction_db": reduction,
        "sample_rate": fs,
        "band": band,
    }


def plot_psd_comparison(
    baseline_path: str | Path,
    anc_path: str | Path,
    output_path: str | Path,
    band: tuple[float, float] = DEFAULT_BAND,
    nperseg: int = DEFAULT_NPERSEG,
) -> None:
    """Plot PSD comparison and save to file.
    KR: PSD 비교 플롯을 생성하고 파일로 저장.

    Args:
        baseline_path: Path to baseline npz
        anc_path: Path to ANC npz
        output_path: Path to save plot
        band: Frequency band to highlight
        nperseg: Samples per segment for Welch
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available; skipping plot.")
        return

    baseline_err, fs = load_error_signal(baseline_path)
    anc_err, _ = load_error_signal(anc_path)

    freqs, psd_base = welch(baseline_err, fs=fs, nperseg=nperseg, scaling="density")
    _, psd_anc = welch(anc_err, fs=fs, nperseg=nperseg, scaling="density")

    # Plot PSD curves on log scale
    # 로그 스케일로 PSD 곡선을 플롯
    plt.figure(figsize=(10, 5))

    plt.subplot(1, 2, 1)
    plt.semilogy(freqs, psd_base, label="Baseline", alpha=0.8)
    plt.semilogy(freqs, psd_anc, label="ANC", alpha=0.8)
    plt.axvspan(band[0], band[1], alpha=0.2, color="red", label=f"Target band {band}")
    plt.xlim(0, 1000)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("PSD (V²/Hz)")
    plt.title("Error Mic PSD Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 2, 2)
    # Show attenuation ratio
    with np.errstate(divide="ignore", invalid="ignore"):
        attenuation = 10 * np.log10(psd_base / (psd_anc + 1e-12))
    plt.plot(freqs, attenuation)
    plt.axvspan(band[0], band[1], alpha=0.2, color="red")
    plt.axhline(y=0, color="k", linestyle="--", alpha=0.5)
    plt.xlim(0, 1000)
    plt.ylim(-20, 30)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Attenuation (dB)")
    plt.title("Frequency-wise Attenuation")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    logger.info("Saved plot to %s", output_path)


def main() -> None:
    """CLI entry point for PSD comparison.
    KR: PSD 비교용 CLI 엔트리 포인트."""
    parser = argparse.ArgumentParser(
        description="Compute 100-300 Hz attenuation from baseline and ANC logs."
    )
    parser.add_argument(
        "--baseline",
        required=True,
        type=Path,
        help="Baseline .npz log file.",
    )
    parser.add_argument(
        "--anc",
        required=True,
        type=Path,
        help="ANC .npz log file.",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=Path("psd_comparison.png"),
        help="Output plot filename (default: psd_comparison.png).",
    )
    parser.add_argument(
        "--nperseg",
        type=int,
        default=DEFAULT_NPERSEG,
        help=f"Samples per segment for Welch (default: {DEFAULT_NPERSEG}).",
    )
    parser.add_argument(
        "--band-low",
        type=float,
        default=DEFAULT_BAND[0],
        help=f"Lower frequency bound (default: {DEFAULT_BAND[0]}).",
    )
    parser.add_argument(
        "--band-high",
        type=float,
        default=DEFAULT_BAND[1],
        help=f"Upper frequency bound (default: {DEFAULT_BAND[1]}).",
    )
    args = parser.parse_args()

    # Configure logging
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    band = (args.band_low, args.band_high)

    # Validate band
    if band[0] >= band[1]:
        parser.error(f"band-low ({band[0]}) must be less than band-high ({band[1]})")

    try:
        results = compute_attenuation(
            args.baseline,
            args.anc,
            band=band,
            nperseg=args.nperseg,
        )

        print(f"Baseline band power ({band[0]}-{band[1]} Hz): {results['baseline_db']:.2f} dB")
        print(f"ANC band power ({band[0]}-{band[1]} Hz): {results['anc_db']:.2f} dB")
        print(f"Reduction: {results['reduction_db']:.2f} dB")

        if args.plot:
            plot_psd_comparison(
                args.baseline,
                args.anc,
                args.plot,
                band=band,
                nperseg=args.nperseg,
            )

    except FileNotFoundError as e:
        logger.error("File error: %s", e)
        raise SystemExit(1) from e
    except KeyError as e:
        logger.error("Data error: %s", e)
        raise SystemExit(1) from e
    except ValueError as e:
        logger.error("Value error: %s", e)
        raise SystemExit(1) from e


if __name__ == "__main__":
    main()
