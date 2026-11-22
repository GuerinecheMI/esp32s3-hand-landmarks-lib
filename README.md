# ESP32-S3 Hand Landmark & Gesture Tracker Library

A C++ library for real-time 21-point hand landmark tracking and gesture recognition on ESP32-S3 microcontrollers using ESP-DL hardware acceleration.

Works seamlessly with **ESP-IDF** (as a managed component) and **Arduino-ESP32 / PlatformIO**.

---

## Features

- **21 MediaPipe Landmarks:** Predicts full skeletal coordinates normalized to $[0.0, 1.0]$.
- **Adaptive Wide-View Tracking:** Follows the hand across $160 \times 120$ QQVGA or $320 \times 240$ QVGA camera views with automatic centroid acquisition and smooth temporal EMA tracking.
- **Hardware Accelerated:** Uses ESP-DL INT8 quantized inference with DSP vector instructions on the Xtensa LX7 dual-core.
- **Ultra-Fast Preprocessing:** Built-in 256-byte LUT eliminates all floating-point math during input quantization; integer scaling crops frames in $<0.2\text{ ms}$.
- **Built-in Gesture Engine:** Real-time geometric classification for Open Palm, Fist, Pinch, Peace Sign, Pointing, and Thumbs Up.
- **High Performance:** Runs at **18–24 FPS** on ESP32-S3 Core 1.

---

## Installation

### ESP-IDF Project
Clone or add this repository into your project's `components/` directory:

```bash
cd your_esp_idf_project/components
git clone https://github.com/GuerinecheMI/esp32s3-hand-landmarks-lib.git esp_hand_tracker
```

In your component's `CMakeLists.txt`, add `esp_hand_tracker` to `REQUIRES`:
```cmake
idf_component_register(
    SRCS "main.cpp"
    REQUIRES esp_hand_tracker esp32-camera
)
```

### PlatformIO / Arduino-ESP32
Add the Git repository URL to your `platformio.ini`:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_deps =
    https://github.com/GuerinecheMI/esp32s3-hand-landmarks-lib.git
```

---

## Quick Usage

```cpp
#include "esp_hand_tracker.hpp"

esp_hand_tracker::HandTracker tracker;

void app_main() {
    // 1. Initialize with your compiled .espdl model binary
    tracker.begin(model_data_ptr, model_size);

    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            // 2. Process wide-view camera frame (e.g. 160x120 grayscale)
            auto result = tracker.process(fb->buf, fb->width, fb->height);

            if (result.detected) {
                // Access individual joint positions [0.0, 1.0]
                auto wrist = result.joint(esp_hand_tracker::Joint::Wrist);
                auto index = result.joint(esp_hand_tracker::Joint::IndexTip);

                // Access recognized gesture
                if (result.gesture == esp_hand_tracker::Gesture::PeaceSign) {
                    // Trigger peace sign action
                }
            }
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## Recognized Joints (21 Points)

| Landmark Index | Joint Name |
| :--- | :--- |
| **0** | `Wrist` |
| **1 – 4** | `ThumbCMC`, `ThumbMCP`, `ThumbIP`, `ThumbTip` |
| **5 – 8** | `IndexMCP`, `IndexPIP`, `IndexDIP`, `IndexTip` |
| **9 – 12** | `MiddleMCP`, `MiddlePIP`, `MiddleDIP`, `MiddleTip` |
| **13 – 16** | `RingMCP`, `RingPIP`, `RingDIP`, `RingTip` |
| **17 – 20** | `PinkyMCP`, `PinkyPIP`, `PinkyDIP`, `PinkyTip` |

---

## Gesture Classes

- `Gesture::OpenPalm` — All 5 fingers extended
- `Gesture::Fist` — All fingers curled inward
- `Gesture::Pinch` — Thumb tip touching index tip
- `Gesture::PeaceSign` — Index and Middle extended
- `Gesture::Pointing` — Index extended only
- `Gesture::ThumbsUp` — Thumb extended upwards, other fingers curled

---

## Hardware Requirements

- **MCU:** ESP32-S3 (with 8MB Flash and Octal/Quad PSRAM recommended)
- **Camera Module:** OV2640, OV3660, or OV5640 (ESP-S3-EYE, Freenove S3 CAM, Xiao ESP32S3 Sense)
- **Power:** Stable 5V / 2A supply recommended for camera stability
