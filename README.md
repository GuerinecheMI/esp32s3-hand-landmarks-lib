# ESP32-S3 Hand Landmark & Gesture Tracker Library

A lightweight C++ library for real-time 21-point hand landmark tracking and gesture recognition on ESP32-S3 microcontrollers using Espressif's [ESP-DL](https://github.com/espressif/esp-dl) hardware acceleration.

Compatible with **ESP-IDF** (as an official component) and **Arduino-ESP32 / PlatformIO**.

---

## Features

- **21 MediaPipe Landmarks:** Predicts complete anatomical joint coordinates normalized to $[0.0, 1.0]$.
- **Adaptive Wide-View Tracking:** Follows the hand across $160 \times 120$ QQVGA or $320 \times 240$ QVGA camera views with automatic centroid acquisition and smooth temporal EMA tracking.
- **Hardware Accelerated:** Uses ESP-DL INT8 quantized inference with DSP vector instructions on the Xtensa LX7 dual-core.
- **Ultra-Fast Preprocessing:** Built-in 256-byte LUT eliminates all floating-point math during input quantization; integer scaling crops frames in $<0.2\text{ ms}$.
- **Built-in Gesture Engine:** Real-time geometric classification for Open Palm, Fist, Pinch, Peace Sign, Pointing, and Thumbs Up.
- **High Performance:** Runs at **18–24 FPS** on ESP32-S3 Core 1.

---

## Models & Weights

This library accepts models compiled in the **ESP-DL FlatBuffers (`.espdl`) INT8** format:

- **Input Dimension:** $96 \times 96$ single-channel grayscale
- **Output Dimension:** 42 elements (21 normalized keypoint pairs)
- **Model Footprint:** **~150 KB** in INT8 (small enough to reside entirely in internal SRAM / cache)

### Generating Custom `.espdl` Models
To train your own model weights or fine-tune on custom hand datasets, use the companion training pipeline repository:
👉 **[esp32s3-hand-landmarks](https://github.com/GuerinecheMI/esp32s3-hand-landmarks)**

### Embedding the Model in Flash
In your application component's `CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "main.cpp"
    EMBED_FILES "models/hand_landmarks.espdl"
)
```

In your C++ code, reference the linker symbols:
```cpp
extern const uint8_t model_start[] asm("_binary_hand_landmarks_espdl_start");
extern const uint8_t model_end[]   asm("_binary_hand_landmarks_espdl_end");

size_t model_size = model_end - model_start;
tracker.begin(model_start, model_size);
```

### Loading from Filesystem (SPIFFS / LittleFS / SD)
If you prefer not to embed the model into the app binary:
```cpp
File file = LittleFS.open("/hand_landmarks.espdl", "r");
size_t size = file.size();
uint8_t* buffer = (uint8_t*)malloc(size);
file.read(buffer, size);
file.close();

tracker.begin(buffer, size);
```

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
    // 1. Initialize tracker with your .espdl model pointer
    tracker.begin(model_start, model_size);

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

| Landmark Index | Joint Name | Anatomical Location |
| :---: | :--- | :--- |
| **0** | `Wrist` | Wrist center |
| **1 – 4** | `Thumb` | `ThumbCMC` $\to$ `ThumbMCP` $\to$ `ThumbIP` $\to$ `ThumbTip` |
| **5 – 8** | `Index Finger` | `IndexMCP` $\to$ `IndexPIP` $\to$ `IndexDIP` $\to$ `IndexTip` |
| **9 – 12** | `Middle Finger` | `MiddleMCP` $\to$ `MiddlePIP` $\to$ `MiddleDIP` $\to$ `MiddleTip` |
| **13 – 16** | `Ring Finger` | `RingMCP` $\to$ `RingPIP` $\to$ `RingDIP` $\to$ `RingTip` |
| **17 – 20** | `Pinky Finger` | `PinkyMCP` $\to$ `PinkyPIP` $\to$ `PinkyDIP` $\to$ `PinkyTip` |

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

---

## License

MIT
