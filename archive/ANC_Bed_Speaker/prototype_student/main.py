"""
Bedside ANC prototype (FxLMS) with secondary-path identification.
침대용 ANC 프로토타입(FxLMS) 및 2차 경로 추정.

Beginner quickstart (EN):
1) Learn secondary path:   python main.py learn-secondary --duration 10 --mu 0.02
2) Run ANC control:        python main.py run --secondary secondary_path.npz
3) Record baseline (off):  python main.py baseline --duration 30

초보자 빠른 시작 (KR):
1) 2차 경로 학습:          python main.py learn-secondary --duration 10 --mu 0.02
2) ANC 실행:               python main.py run --secondary secondary_path.npz
3) 기준 신호 기록(OFF):    python main.py baseline --duration 30

Project layout (EN/KR):
- anc/dsp.py: core DSP utilities, FIR, ring buffer / DSP 유틸리티, FIR, 링버퍼
- anc/secondary_path.py: S_hat estimation / 2차 경로 추정
- anc/fxlms.py: FxLMS controller + runtime / FxLMS 컨트롤러 + 실행
- anc/baseline.py: baseline recorder / 기준 신호 기록
- tools/measure_psd.py: PSD comparison / PSD 비교
- docs/GLOSSARY.md: glossary / 용어집
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import sounddevice as sd

from anc.baseline import record_baseline
from anc.dsp import DspConfig
from anc.fxlms import run_fx_lms
from anc.secondary_path import estimate_secondary_path

if TYPE_CHECKING:
    pass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Type for device specification
DeviceType = int | str | tuple[int | None, int | None] | None


def _resolve_device(args: argparse.Namespace) -> DeviceType:
    """Resolve combined device tuple when input/output are specified.
    KR: 입력/출력 장치가 분리된 경우 결합 장치 튜플로 구성."""
    if args.device is not None:
        # Try to parse as int, otherwise use as string
        try:
            return int(args.device)
        except ValueError:
            return args.device
    if args.input_device is not None or args.output_device is not None:
        return (args.input_device, args.output_device)
    return None


def list_devices() -> None:
    """Print available audio devices.
    KR: 사용 가능한 오디오 장치 출력."""
    print("Available audio devices / 사용 가능한 오디오 장치:")
    print("-" * 60)
    print(sd.query_devices())
    print("-" * 60)
    print("\nDefault input device:", sd.default.device[0])
    print("Default output device:", sd.default.device[1])


def validate_duration(value: float | None, name: str) -> None:
    """Validate duration parameter.
    KR: duration 파라미터 검증."""
    if value is not None and value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")


def validate_mu(value: float, name: str) -> None:
    """Validate learning rate parameter.
    KR: 학습률 파라미터 검증."""
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")
    if value > 1.0:
        logger.warning("%s (%.4f) is unusually large, may cause instability", name, value)


def validate_path(path: str, must_exist: bool = False) -> Path:
    """Validate file path.
    KR: 파일 경로 검증."""
    p = Path(path)
    if must_exist and not p.exists():
        raise FileNotFoundError(f"File not found: {p}")
    # Check parent directory exists for output files
    if not must_exist and not p.parent.exists():
        raise FileNotFoundError(f"Parent directory does not exist: {p.parent}")
    return p


def build_parser() -> argparse.ArgumentParser:
    """CLI parser for learn-secondary / run / baseline modes.
    KR: learn-secondary / run / baseline용 CLI 파서."""
    parser = argparse.ArgumentParser(
        description="Bedside ANC prototype (FxLMS) with secondary path learning.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py learn-secondary --duration 10 --mu 0.02
  python main.py run --secondary secondary_path.npz --duration 30
  python main.py baseline --duration 30

예시:
  python main.py learn-secondary --duration 10 --mu 0.02
  python main.py run --secondary secondary_path.npz --duration 30
  python main.py baseline --duration 30
        """,
    )

    # Global options
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List audio devices and exit.",
    )
    parser.add_argument(
        "--device",
        help="Sounddevice device id or name.",
    )
    parser.add_argument(
        "--input-device",
        type=int,
        help="Input device id (overrides --device for input).",
    )
    parser.add_argument(
        "--output-device",
        type=int,
        help="Output device id (overrides --device for output).",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging.",
    )
    parser.add_argument(
        "--block-size",
        type=int,
        default=128,
        help="Audio block size in samples (default: 128, safer start).",
    )
    parser.add_argument(
        "--latency",
        default="low",
        help="Stream latency hint: 'low', 'high', or float seconds (default: 'low').",
    )

    sub = parser.add_subparsers(dest="command")

    # learn-secondary subcommand
    learn = sub.add_parser(
        "learn-secondary",
        help="Estimate S_hat with white noise.",
        description="Estimate secondary path using white noise excitation and NLMS.",
    )
    learn.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Duration in seconds (default: 10.0).",
    )
    learn.add_argument(
        "--mu",
        type=float,
        default=0.02,
        help="NLMS step size (default: 0.02).",
    )
    learn.add_argument(
        "--log",
        default="secondary_path.npz",
        help="Output file path (default: secondary_path.npz).",
    )

    # run subcommand
    run = sub.add_parser(
        "run",
        help="Run FxLMS ANC.",
        description="Run FxLMS active noise control with pre-learned secondary path.",
    )
    run.add_argument(
        "--secondary",
        default="secondary_path.npz",
        help="Secondary path .npz file (default: secondary_path.npz).",
    )
    run.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Duration in seconds (default: indefinite, Ctrl+C to stop).",
    )
    run.add_argument(
        "--mu",
        type=float,
        default=1e-4,
        help="FxLMS step size (default: 1e-4).",
    )
    run.add_argument(
        "--limiter-drive",
        type=float,
        default=1.0,
        help="Soft limiter drive factor (default: 1.0).",
    )
    run.add_argument(
        "--nlms",
        action="store_true",
        default=True,
        help="Use NLMS normalization (default: True).",
    )
    run.add_argument(
        "--no-nlms",
        action="store_false",
        dest="nlms",
        help="Disable NLMS normalization.",
    )
    run.add_argument(
        "--w-log-every",
        type=int,
        default=10,
        help="Log filter weights every N blocks (default: 10).",
    )
    run.add_argument(
        "--log",
        default="anc_run.npz",
        help="Output log file (default: anc_run.npz).",
    )

    # baseline subcommand
    base = sub.add_parser(
        "baseline",
        help="Record baseline with ANC off.",
        description="Record baseline (ANC off) signal for comparison.",
    )
    base.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Duration in seconds (default: indefinite, Ctrl+C to stop).",
    )
    base.add_argument(
        "--log",
        default="baseline.npz",
        help="Output file path (default: baseline.npz).",
    )

    return parser


def main() -> int:
    """Entry point.
    KR: 엔트리 포인트.

    Returns:
        Exit code (0 for success, non-zero for error)
    """
    parser = build_parser()
    args = parser.parse_args()

    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Handle --list-devices
    if args.list_devices:
        list_devices()
        return 0

    # Require a command
    if args.command is None:
        parser.print_help()
        return 1

    try:
        device = _resolve_device(args)
        cfg = DspConfig(block_size=args.block_size)

        # Parse latency hint
        try:
            latency = float(args.latency)
        except ValueError:
            latency = args.latency  # 'low' or 'high'

        if args.command == "learn-secondary":
            validate_duration(args.duration, "duration")
            validate_mu(args.mu, "mu")
            log_path = validate_path(args.log)

            cfg.secondary_duration_s = args.duration
            cfg.secondary_mu = args.mu

            logger.info("Starting secondary path learning...")
            estimate_secondary_path(cfg, device, log_path)
            logger.info("Secondary path saved to %s", log_path)
            return 0

        if args.command == "baseline":
            validate_duration(args.duration, "duration")
            log_path = validate_path(args.log)

            logger.info("Starting baseline recording...")
            record_baseline(cfg, device, log_path, args.duration, latency=latency)
            logger.info("Baseline saved to %s", log_path)
            return 0

        if args.command == "run":
            validate_duration(args.duration, "duration")
            validate_mu(args.mu, "mu")
            secondary_path = validate_path(args.secondary, must_exist=True)
            log_path = validate_path(args.log)

            cfg.mu = args.mu
            cfg.limiter_drive = args.limiter_drive

            logger.info("Starting FxLMS ANC...")
            run_fx_lms(
                cfg,
                device,
                s_hat_path=secondary_path,
                log_path=log_path,
                duration_s=args.duration,
                use_nlms=args.nlms,
                w_log_every=args.w_log_every,
            )
            logger.info("ANC log saved to %s", log_path)
            return 0

    except FileNotFoundError as e:
        logger.error("File error: %s", e)
        return 1
    except ValueError as e:
        logger.error("Validation error: %s", e)
        return 1
    except KeyboardInterrupt:
        logger.info("Operation cancelled by user.")
        return 0
    except Exception as e:
        logger.exception("Unexpected error: %s", e)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
