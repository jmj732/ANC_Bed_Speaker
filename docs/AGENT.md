# AGENTS.md

## 🎯 Project Overview

This project implements a **real-time Adaptive Active Noise Control (ANC)** system on Raspberry Pi.

Core algorithm:

* FxLMS (Filtered-x Least Mean Square)
* NLMS-based adaptation
* Secondary path modeling (s_hat)

Audio pipeline:

* ALSA (arecord / aplay)
* Real-time processing (low latency required)

Target:

* 80~150Hz noise reduction
* Goal: **≥10dB attenuation**

---

## ⚙️ System Constraints (VERY IMPORTANT)

* Real-time system (latency < 5ms)
* CPU limited (Raspberry Pi)
* No dynamic memory allocation in processing loop
* No blocking operations in audio callback
* Avoid heavy libraries (numpy, scipy in runtime)

---

## 🧠 Core Concepts

### 1. Secondary Path (s_hat)

* FIR filter modeling: speaker → air → error mic
* Used to compute filtered-x signal
* Accuracy is CRITICAL for convergence

### 2. Filtered-x Signal

x_filt = S_hat * x_ref

* Used in weight update
* If incorrect → divergence

### 3. NLMS Update

* Normalize by signal power
* Prevent scale dependency

---

## 🚨 Critical Rules (DO NOT VIOLATE)

### 🔴 Real-time safety

* NO malloc/free in processing loop
* NO printf/log inside audio loop
* NO file I/O in real-time path

### 🔴 Stability

* Always protect division:
  mu_n = mu / (power + epsilon)

* epsilon MUST NOT be too small (≥ 1e-4)

* Clamp mu_n:
  if (mu_n > 0.1f) mu_n = 0.1f;

### 🔴 Buffer safety

* Fixed-size arrays only
* No overflow
* Use circular buffer carefully

---

## 🧩 Coding Rules

### ✔ Style

* Plain C (C99)
* No unnecessary abstraction
* Optimize for readability + performance

### ✔ Naming

* w[] → adaptive filter weights
* x[] → reference signal
* fx[] → filtered-x
* s_hat[] → secondary path
* e → error signal
* y → anti-noise output

---

## ⚡ FxLMS Implementation (REFERENCE)

```c
// compute output
y = 0.0f;
for (int k = 0; k < W_LEN; k++)
    y += w[k] * x[k];

// compute filtered-x power
float fx_pow = 0.0f;
for (int k = 0; k < W_LEN; k++)
    fx_pow += fx[k] * fx[k];

// NLMS step size
float mu_n = mu / (fx_pow + 1e-4f);
if (mu_n > 0.1f) mu_n = 0.1f;

// update weights
for (int k = 0; k < W_LEN; k++)
    w[k] = leakage * w[k] - mu_n * e * fx[k];
```

---

## 🔧 Secondary Path Measurement

* Use white noise input
* NLMS adaptation
* Long enough duration (≥ 30 sec)

Update rule:

```c
float u_pow = 0.0f;
for (int k = 0; k < S_LEN; k++)
    u_pow += u[k] * u[k];

float mu_n = mu_s / (u_pow + 1e-4f);

for (int k = 0; k < S_LEN; k++)
    s_hat[k] += mu_n * pe * u[k];
```

---

## 📊 Debugging Strategy

### ✔ Always check:

* fx_pow value (too small → unstable)
* mu_n value (explosion 여부)
* error RMS
* anti-noise RMS

### ✔ Expected behavior:

* 초기: noise 증가 가능
* 이후: 점진적 감소
* steady: 안정된 attenuation

---

## 🚫 Common Failure Cases

1. fx_pow ≈ 0 → mu explosion → divergence
2. s_hat inaccurate → wrong phase → no cancellation
3. leakage too small → weight saturation
4. mu too large → instability
5. audio delay mismatch → phase error

---

## 🎯 Performance Targets

| Metric          | Target        |
| --------------- | ------------- |
| Latency         | < 5ms         |
| Stability       | No divergence |
| Noise reduction | ≥ 10dB        |
| CPU usage       | < 50%         |

---

## 🧪 Testing Workflow

1. Run secondary path measurement
2. Verify convergence (pred_err ↓)
3. Run ANC with sine wave (100Hz)
4. Measure dB reduction
5. Tune mu / leakage if needed

---

## 🤖 Instructions for AI (IMPORTANT)

When modifying code:

* NEVER break real-time constraints
* ALWAYS protect NLMS normalization
* ALWAYS assume low-power CPU
* Prefer stability over speed
* If unsure → choose safer (smaller mu)

When debugging:

* First check s_hat
* Then check fx_pow
* Then check mu

---

## 🚀 Goal

Stable real-time ANC with:

* No divergence
* Fast convergence
* Audible noise reduction

---

END OF FILE
