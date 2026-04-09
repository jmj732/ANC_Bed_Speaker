import sounddevice as sd
import numpy as np

DEVICE = 'hw:0,0'
SAMPLERATE = 44100
BLOCKSIZE = 1024
CHANNELS = 2

def callback(indata, outdata, frames, time, status):
    if status:
        print(status)
    outdata[:] = indata

with sd.Stream(device=(DEVICE, DEVICE),
               samplerate=SAMPLERATE,
               blocksize=BLOCKSIZE,
               channels=CHANNELS,
               dtype='int16',
               callback=callback):
    print('루프백 시작 — Ctrl+C로 종료')
    while True:
        sd.sleep(1000)
