#!/bin/bash

WIDTH=640
HEIGHT=480

export WIDTH HEIGHT

encode_frame() {
    rgb_file="$1"
    frame_number="${rgb_file#frame_}"
    frame_number="${frame_number%.rgb}"
    jpeg_file="jpeg_${frame_number}.jpg"

    echo "[INFO] Starting $rgb_file → $jpeg_file (PID $$)"

    ffmpeg -hide_banner -loglevel error -threads 1 \
        -f rawvideo -pix_fmt rgb24 -s ${WIDTH}x${HEIGHT} \
        -i "$rgb_file" -vframes 1 -c:v mjpeg "$jpeg_file"

    if [[ $? -eq 0 ]]; then
        echo "[OK] Finished $jpeg_file (PID $$)"
    else
        echo "[ERROR] Failed $rgb_file (PID $$)"
    fi
}

export -f encode_frame

# Process frames in parallel, showing progress
parallel -j"$(nproc)" encode_frame ::: frame_*.rgb


