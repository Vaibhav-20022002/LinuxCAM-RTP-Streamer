# C++ RTP/MJPEG Streamer

![Build Status](https://img.shields.io/badge/status-experimental-red)
![License](https://img.shields.io/badge/license-MIT-blue)
![Language](https://img.shields.io/badge/language-C++17-blue)
![Platform](https://img.shields.io/badge/platform-Linux-lightgrey)

Single-process MJPEG streaming over UDP. Captures video from V4L2 or libcamera devices, converts it, packetizes it into RTP/MJPEG, and broadcasts it.

---

## Project Status

This is experimental and currently broken. Will fix in the next version.

---

## Features

- **Modular pipeline** -- Four stages: `Driver` -> `FrameForger` -> `TurboSnap` -> `PacketPulse`. Connected via lock-free SPSC queues (`RingMaster`).
- **Zero-copy MJPEG** -- If your camera outputs MJPEG natively, encoding is bypassed.
- **SIMD-accelerated JPEG encoding** -- TurboSnap uses TurboJPEG with SIMD for non-MJPEG formats.
- **Single-process, multi-threaded** -- All four stages run in one process.
- **Configurable defaults** -- Autodetects the best camera format. Binds to 0.0.0.0:5004 if nothing is provided.
- **Documented** -- Javadoc-style inline comments.

---

## Architecture

```
+---------+      +-------------+      +-----------+      +------------+
| Driver  | ---> | FrameForger | ---> | TurboSnap | ---> | PacketPulse|
+---------+      +-------------+      +-----------+      +------------+
         \____________________ RingMaster _________________/
```

Each stage runs concurrently and communicates through a lock-free ring buffer.

---

## Supported Formats

### Input (priority order)
- MJPEG (preferred -- skips re-encoding)
- YUV420
- YUV422
- RGB24

### Output
- RTP/MJPEG over UDP

### Codec Support
- MJPEG
- H.264/H.265/AV1 -- planned

---

## Prerequisites

- Linux with a V4L2 or LibCamera-compatible device
- C++17 compiler (GCC >= 9, Clang >= 11)
- TurboJPEG development libraries

---

## Contributing

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/<name>`
3. Commit your changes: `git commit -m "Add cool new thing"`
4. Push: `git push origin feature/<name>`
5. Open a pull request

---

## License

MIT. Do what you want.

---

## Final Note

This is half-baked and experimental. Tinker with it or scrap it and build something better.
