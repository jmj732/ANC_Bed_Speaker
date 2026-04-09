#!/usr/bin/env python3
"""
R 마이크 입력 → L+R 스피커 출력 (HiFiBerry DAC2 ADC Pro, device=0)
bandpass 80~8000Hz + noise gate
"""
import numpy as np
import sounddevice as sd
from scipy.signal import butter, sosfilt, sosfilt_zi

DEVICE = 0
SAMPLERATE = 48000
BLOCKSIZE = 512
CHANNELS = 2
GAIN = 8.0
NOISE_GATE = 0.005  # 이 이하 신호는 묵음 처리

# bandpass 80~8000Hz (4차 Butterworth)
sos = butter(4, [80, 8000], btype="bandpass", fs=SAMPLERATE, output="sos")
zi = sosfilt_zi(sos) * 0.0  # 초기 상태

def callback(indata, outdata, frames, time, status):
    global zi
    if status:
        print(status)

    r_mic = indata[:, 1].astype(np.float64) * GAIN

    # bandpass 필터 (블록 경계 연속)
    filtered, zi = sosfilt(sos, r_mic, zi=zi)

    # noise gate
    rms = np.sqrt(np.mean(filtered ** 2))
    if rms < NOISE_GATE:
        filtered[:] = 0.0

    np.clip(filtered, -1.0, 1.0, out=filtered)
    out = filtered.astype(np.float32)
    outdata[:, 0] = out
    outdata[:, 1] = out

print(f"R mic → L+R  |  bandpass 80~8000Hz  |  gate={NOISE_GATE}  |  gain={GAIN}x")
print("Ctrl+C to stop")

with sd.Stream(
    device=(DEVICE, DEVICE),
    samplerate=SAMPLERATE,
    blocksize=BLOCKSIZE,
    channels=CHANNELS,
    dtype="float32",
    callback=callback,
):
    try:
        while True:
            sd.sleep(1000)
    except KeyboardInterrupt:
        print("\nStopped.")
