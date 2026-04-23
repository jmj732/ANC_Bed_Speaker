#!/usr/bin/env python3
"""Plot FFT of ref and error mic recordings from nb ANC run."""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
import os

SAMPLE_RATE = 96000
REF_FILE = "/tmp/rec_ref.raw"
ERR_FILE = "/tmp/rec_err.raw"
OUT_FILE = "/tmp/fft_plot.png"

def load_raw(path):
    data = np.frombuffer(open(path, 'rb').read(), dtype=np.int16)
    return data.astype(np.float32) / 32768.0

def plot_fft(sig, rate, label, color, ax):
    n = len(sig)
    win = np.hanning(n)
    fft = np.abs(np.fft.rfft(sig * win)) / n
    fft[1:-1] *= 2  # single-sided
    freqs = np.fft.rfftfreq(n, 1.0 / rate)
    fft_db = 20 * np.log10(fft + 1e-9)
    ax.plot(freqs, fft_db, label=label, color=color, linewidth=0.8)

if not os.path.exists(REF_FILE) or not os.path.exists(ERR_FILE):
    print(f"ERROR: recording files not found. Run: ../build/anc nb --record-secs=5")
    sys.exit(1)

ref = load_raw(REF_FILE)
err = load_raw(ERR_FILE)

print(f"ref: {len(ref)} samples ({len(ref)/SAMPLE_RATE:.1f}s)  rms={np.sqrt(np.mean(ref**2)):.5f}")
print(f"err: {len(err)} samples ({len(err)/SAMPLE_RATE:.1f}s)  rms={np.sqrt(np.mean(err**2)):.5f}")

fig, axes = plt.subplots(2, 1, figsize=(12, 8))

for sig, label, color, ax in [(ref, 'REF mic (R)', 'steelblue', axes[0]),
                                (err, 'ERR mic (L)', 'tomato',    axes[1])]:
    plot_fft(sig, SAMPLE_RATE, label, color, ax)
    ax.set_xlim(0, 1000)
    ax.set_ylim(-80, 0)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('dBFS')
    ax.set_title(label)
    ax.axvline(150, color='gray', linestyle='--', linewidth=0.8, label='150 Hz')
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(OUT_FILE, dpi=150)
print(f"Saved: {OUT_FILE}")
