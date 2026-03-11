# Whole File vs Segment-Based Fair Comparison

## Comparison Rule
- `raw`: analyze the original waveform as-is.
- `snore_only`: remove silence with `librosa.effects.split()` and concatenate only detected snoring segments.
- Both methods use the exact same metric functions for `spectral centroid`, `spectral bandwidth`, `spectral rolloff`, `RMS`, `dominant frequency`, and `Snore Index`.
- `Snore Index` is defined consistently as the FFT power ratio in the 80-300 Hz band over all positive-frequency power.
- All files are standardized to 8000 Hz before analysis so both methods share the same time-frequency resolution.

## Aggregate Results
- File count: 29
- Total detected segments: 927
- Mean segment coverage ratio: 0.472
- Type changed after silence removal: 0 / 29 files
- Mean Snore Index: raw 0.414 -> snore_only 0.417
- Median dominant frequency: raw 101.30 Hz -> snore_only 88.54 Hz

## Type Distribution
- raw: high_low_freq=12, mid=9, broadband_high_freq=8
- snore_only: high_low_freq=12, mid=9, broadband_high_freq=8

## Largest Snore Index Shifts
- freesound_community-soft-female-snoring-17325.mp3: delta=0.022, raw=0.953 (high_low_freq), snore_only=0.976 (high_low_freq), segments=85, coverage=0.188
- geriart-snoring-8486.mp3: delta=0.019, raw=0.538 (high_low_freq), snore_only=0.557 (high_low_freq), segments=6, coverage=0.273
- freesound_community-snoring-74587.mp3: delta=0.017, raw=0.676 (high_low_freq), snore_only=0.692 (high_low_freq), segments=8, coverage=0.215
- freesound_community-snore-69841.mp3: delta=0.010, raw=0.752 (high_low_freq), snore_only=0.762 (high_low_freq), segments=16, coverage=0.331
- freesound_community-023789_man-snoring-74637.mp3: delta=0.008, raw=0.527 (high_low_freq), snore_only=0.534 (high_low_freq), segments=23, coverage=0.425

## Type Transition Matrix
| raw_type | broadband_high_freq | high_low_freq | mid |
| --- | --- | --- | --- |
| broadband_high_freq | 8 | 0 | 0 |
| high_low_freq | 0 | 12 | 0 |
| mid | 0 | 0 | 9 |

## Notes
- This comparison is methodologically fairer than the original notebook comparison because only the presence or absence of silence removal changes.
- Segment imbalance still matters for event-level analyses, but the default comparison unit here is the file, not the segment.
