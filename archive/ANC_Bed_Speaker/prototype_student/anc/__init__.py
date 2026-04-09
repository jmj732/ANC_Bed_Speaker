"""
ANC package: core DSP, secondary-path learning, and FxLMS control.
KR: DSP 코어, 2차 경로 학습, FxLMS 제어 패키지.

Main components:
- DspConfig: Configuration for all DSP operations
- FxLMSController: Real-time FxLMS adaptive filter controller
- estimate_secondary_path: Secondary path identification
- record_baseline: Baseline signal recording
"""

from __future__ import annotations

from .baseline import record_baseline
from .dsp import DspConfig, FIRFilter, RingBuffer, build_bandpass_taps, db_to_linear
from .fxlms import FxLMSController, run_fx_lms
from .secondary_path import estimate_secondary_path

__all__ = [
    # Configuration
    "DspConfig",
    # DSP utilities
    "FIRFilter",
    "RingBuffer",
    "build_bandpass_taps",
    "db_to_linear",
    # Main functions
    "FxLMSController",
    "run_fx_lms",
    "estimate_secondary_path",
    "record_baseline",
]

__version__ = "0.1.0"
