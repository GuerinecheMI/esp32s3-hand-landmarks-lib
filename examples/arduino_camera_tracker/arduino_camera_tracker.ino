/**
 * @file arduino_camera_tracker.ino
 * @brief Arduino-ESP32 example for the ESP32 Hand Tracker library.
 */

#include "esp_camera.h"
#include "esp_hand_tracker.hpp"

esp_hand_tracker::HandTracker tracker;

// Embedded model symbols if embedded, or passed from SPIFFS/LittleFS
extern const uint8_t model_start[] asm("_binary_hand_landmarks_espdl_start") __attribute__((weak));
extern const uint8_t model_end[]   asm("_binary_hand_landmarks_espdl_end") __attribute__((weak));

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\nInitializing ESP32-S3 Hand Tracker...");

    // Standard ESP-S3-EYE pin configuration
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 11;
    config.pin_d1 = 9;
    config.pin_d2 = 8;
    config.pin_d3 = 10;
    config.pin_d4 = 12;
    config.pin_d5 = 18;
    config.pin_d6 = 17;
    config.pin_d7 = 16;
    config.pin_xclk = 15;
    config.pin_pclk = 13;
    config.pin_vsync = 6;
    config.pin_href = 7;
    config.pin_sccb_sda = 4;
    config.pin_sccb_scl = 5;
    config.pin_reset = -1;
    config.pin_pwdn = -1;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QQVGA; // 160x120 wide view
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera init failed!");
        while (true) delay(1000);
    }

    size_t model_size = (model_end > model_start) ? (model_end - model_start) : 0;
    tracker.begin(model_start, model_size);
    Serial.println("Hand Tracker initialized!");
}

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return;

    auto result = tracker.process(fb->buf, fb->width, fb->height);
    if (result.detected) {
        Serial.printf("Hand Detected! Gesture ID: %d | Wrist: (%.2f, %.2f)\n",
                      static_cast<int>(result.gesture),
                      result.joint(esp_hand_tracker::Joint::Wrist).x,
                      result.joint(esp_hand_tracker::Joint::Wrist).y);
    }

    esp_camera_fb_return(fb);
    delay(5);
}
