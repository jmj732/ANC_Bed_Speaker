"""
Tests for main.py CLI module.
KR: main.py CLI 모듈 테스트.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest

from prototype_student.main import (
    build_parser,
    validate_duration,
    validate_mu,
    validate_path,
)


class TestValidateDuration:
    """Tests for validate_duration function."""

    def test_valid_positive(self) -> None:
        """Test valid positive duration."""
        validate_duration(10.0, "duration")  # Should not raise

    def test_valid_zero(self) -> None:
        """Test valid zero duration."""
        validate_duration(0.0, "duration")  # Should not raise

    def test_valid_none(self) -> None:
        """Test valid None duration."""
        validate_duration(None, "duration")  # Should not raise

    def test_invalid_negative(self) -> None:
        """Test invalid negative duration."""
        with pytest.raises(ValueError, match="must be non-negative"):
            validate_duration(-1.0, "duration")


class TestValidateMu:
    """Tests for validate_mu function."""

    def test_valid_small(self) -> None:
        """Test valid small mu."""
        validate_mu(1e-4, "mu")  # Should not raise

    def test_valid_large(self) -> None:
        """Test valid but large mu (should warn)."""
        # This should log a warning but not raise
        validate_mu(0.5, "mu")

    def test_invalid_zero(self) -> None:
        """Test invalid zero mu."""
        with pytest.raises(ValueError, match="must be positive"):
            validate_mu(0.0, "mu")

    def test_invalid_negative(self) -> None:
        """Test invalid negative mu."""
        with pytest.raises(ValueError, match="must be positive"):
            validate_mu(-0.01, "mu")


class TestValidatePath:
    """Tests for validate_path function."""

    def test_valid_existing_file(self, temp_directory: Path) -> None:
        """Test validation of existing file."""
        test_file = temp_directory / "test.txt"
        test_file.write_text("test")

        result = validate_path(str(test_file), must_exist=True)
        assert result == test_file

    def test_nonexistent_file_must_exist(self, temp_directory: Path) -> None:
        """Test error when file must exist but doesn't."""
        with pytest.raises(FileNotFoundError):
            validate_path(str(temp_directory / "nonexistent.txt"), must_exist=True)

    def test_nonexistent_file_output(self, temp_directory: Path) -> None:
        """Test validation of output file path."""
        # Should not raise if parent exists
        result = validate_path(str(temp_directory / "output.txt"), must_exist=False)
        assert result == temp_directory / "output.txt"

    def test_nonexistent_parent_output(self) -> None:
        """Test error when parent directory doesn't exist for output."""
        with pytest.raises(FileNotFoundError, match="Parent directory"):
            validate_path("/nonexistent/parent/output.txt", must_exist=False)


class TestBuildParser:
    """Tests for build_parser function."""

    def test_parser_creation(self) -> None:
        """Test parser is created successfully."""
        parser = build_parser()
        assert isinstance(parser, argparse.ArgumentParser)

    def test_list_devices_flag(self) -> None:
        """Test --list-devices flag."""
        parser = build_parser()
        args = parser.parse_args(["--list-devices"])
        assert args.list_devices is True

    def test_learn_secondary_defaults(self) -> None:
        """Test learn-secondary subcommand defaults."""
        parser = build_parser()
        args = parser.parse_args(["learn-secondary"])

        assert args.command == "learn-secondary"
        assert args.duration == 10.0
        assert args.mu == 0.02
        assert args.log == "secondary_path.npz"

    def test_learn_secondary_custom(self) -> None:
        """Test learn-secondary with custom values."""
        parser = build_parser()
        args = parser.parse_args([
            "learn-secondary",
            "--duration", "20",
            "--mu", "0.05",
            "--log", "custom.npz",
        ])

        assert args.duration == 20.0
        assert args.mu == 0.05
        assert args.log == "custom.npz"

    def test_run_defaults(self) -> None:
        """Test run subcommand defaults."""
        parser = build_parser()
        args = parser.parse_args(["run"])

        assert args.command == "run"
        assert args.secondary == "secondary_path.npz"
        assert args.duration is None
        assert args.mu == 1e-4
        assert args.limiter_drive == 1.0
        assert args.nlms is True
        assert args.w_log_every == 10
        assert args.log == "anc_run.npz"

    def test_run_no_nlms(self) -> None:
        """Test run with --no-nlms flag."""
        parser = build_parser()
        args = parser.parse_args(["run", "--no-nlms"])

        assert args.nlms is False

    def test_baseline_defaults(self) -> None:
        """Test baseline subcommand defaults."""
        parser = build_parser()
        args = parser.parse_args(["baseline"])

        assert args.command == "baseline"
        assert args.duration is None
        assert args.log == "baseline.npz"

    def test_device_options(self) -> None:
        """Test device specification options."""
        parser = build_parser()

        # Test --device
        args = parser.parse_args(["--device", "hw:0", "baseline"])
        assert args.device == "hw:0"

        # Test --input-device and --output-device
        args = parser.parse_args([
            "--input-device", "1",
            "--output-device", "2",
            "baseline",
        ])
        assert args.input_device == 1
        assert args.output_device == 2

    def test_verbose_flag(self) -> None:
        """Test -v/--verbose flag."""
        parser = build_parser()

        args = parser.parse_args(["-v", "baseline"])
        assert args.verbose is True

        args = parser.parse_args(["--verbose", "baseline"])
        assert args.verbose is True
