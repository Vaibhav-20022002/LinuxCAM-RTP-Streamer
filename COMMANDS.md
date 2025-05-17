# Commands

## `v4l2` :

### List available devices
```
$ v4l2-ctl --list-devices
```

### List device supported formats :
```
$ v4l2-ctl -d {device_name} --list-formats
```

- Example : `v4l2-ctl -d /dev/video0 --list-formats`

- Expected Output :
    ```
    Integrated_Webcam_HD: Integrate (usb-0000:00:14.0-5):
        /dev/video0
        /dev/video1
        /dev/media0
    ```

### List device supported formats with resolution and frame-rate :
```
$ v4l2-ctl -d {device_name} --list-formats-ext
```

- Example : `v4l2-ctl -d /dev/video0 --list-formats-ext`

- Expected Output : 
    ```
    ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 1280x720
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 848x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 960x540
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x360
                        Interval: Discrete 0.033s (30.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 160x120
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 320x180
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 320x240
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 424x240
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x360
                        Interval: Discrete 0.033s (30.000 fps)
    ```

---

