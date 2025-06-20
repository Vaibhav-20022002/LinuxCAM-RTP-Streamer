# 🎥 C++ RTP/MJPEG Streamer

![Build Status](https://img.shields.io/badge/status-experimental-red)
![License](https://img.shields.io/badge/license-MIT-blue)
![Language](https://img.shields.io/badge/language-C++17-blue)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey)

> **Blazing-fast, single-process MJPEG streaming over UDP**
> Capture, convert, packetize, and broadcast live video from any V4L2 or libcamera-compatible device — all in real time.

---

## ⚠️ Project Status


> ⚠️ Heads up: This version is a total dumpster fire - doesn’t work at all. But you’ve still got 99% (.99?) of the units up and running. Will fix in the next version hopefully, with more optimisations. ¯\\(ツ)/¯


---

## 🌟 Features
✨ __Modular pipeline__ – Four cleanly separated stages:
  - `Driver` → `FrameForger` → `TurboSnap` → `PacketPulse`. All interconnected with blazing-fast, lock-free SPSC queues ("`RingMaster`").

🌀 __Zero-copy MJPEG__ – If your camera provides MJPEG natively, it bypasses encoding for ultra-low latency streaming.

🏎️ __SIMD-accelerated JPEG encoding__ – For non-MJPEG formats, TurboSnap leverages TurboJPEG (with SIMD) to squeeze every CPU cycle.

🧵 __Single-process, multi-threaded__ – Max efficiency. All four stages live harmoniously inside one process.

⚙️ __Configurable defaults__ – Autodetects best camera format. Binds to __0.0.0.0:5004__ if nothing is provided.

📚 __Well-documented__ – Javadoc-style inline comments and intuitive naming for seamless exploration.

---

## 🏗 Architecture Overview

```text
┌─────────┐      ┌─────────────┐      ┌───────────┐      ┌────────────┐
│ Driver  │ ──▶  │ FrameForger │ ──▶  │ TurboSnap │ ──▶  │ PacketPulse│
└─────────┘      └─────────────┘      └───────────┘      └────────────┘
         \____________________ RingMaster _________________/

```

  > Modularity meets concurrency. Each stage is dedicated, concurrent, and connected via a lock-free ring buffer.

---

## 🎞 Supported Formats

- __Input Pixel Formats (priority order):__

   - 🟢 MJPEG (preferable: skips re-encoding)
   - 🟡 YUV420
   - 🟠 YUV422
   - 🔵 RGB24

- __Output:__

  - RTP/MJPEG over UDP

- __Codec Support:__

  - ✅ MJPEG
  - ⛔ No H.264/H.265/AV1 — planned for future versions

---

## 🛠 Prerequisites

- __Linux__ machine with:

  - ✅ V4L2 or LibCamera-compatible device

- 🧰 __Build tools__:

  - C++17 compiler (GCC ≥ 9, Clang ≥ 11)
  - TurboJPEG development libraries

---


## 🤝 Contributing

1. 🍴 Fork this repo

2. 🔧 Create your feature branch: git checkout -b feature/<name>

3. 💬 Commit your changes: git commit -m "Add cool new thing"

4. 🚀 Push to your branch: git push origin feature/<name>

5. 📬 Open a pull request

> Contributions, ideas, bug reports, and improvements are welcome!

---

## 📄 License

This project is licensed under the __MIT__ License.

 > Do what you want

---

## 💬 Final Note

```
⚠️ This thing's half-baked and totally experimental.
```

> Tinker with it if you want, or just scrap it and build something way cooler using whatever you can learn/take from here. Up to you—no pressure (and no guarantees ¯\\(ツ)/¯).
