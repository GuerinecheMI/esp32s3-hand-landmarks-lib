/**
 * @file main.cpp
 * @brief Minimal example demonstrating the esp_hand_tracker library.
 */

#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"

#include "esp_hand_tracker.hpp"

static const char* TAG = "example";

extern const uint8_t model_start[] asm("_binary_hand_landmarks_espdl_start") __attribute__((weak));
extern const uint8_t model_end[]   asm("_binary_hand_landmarks_espdl_end") __attribute__((weak));

static const camera_config_t camera_config = {
    .pin_pwdn = -1, .pin_reset = -1, .pin_xclk = 15,
    .pin_sccb_sda = 4, .pin_sccb_scl = 5,
    .pin_d7 = 16, .pin_d6 = 17, .pin_d5 = 18, .pin_d4 = 12,
    .pin_d3 = 10, .pin_d2 = 8, .pin_d1 = 9, .pin_d0 = 11,
    .pin_vsync = 6, .pin_href = 7, .pin_pclk = 13,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_QQVGA, // 160x120
    .jpeg_quality = 12, .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

static const char* gesture_to_string(esp_hand_tracker::Gesture g) {
    switch (g) {
        case esp_hand_tracker::Gesture::OpenPalm:  return "Open Palm";
        case esp_hand_tracker::Gesture::Fist:      return "Fist";
        case esp_hand_tracker::Gesture::Pinch:     return "Pinch";
        case esp_hand_tracker::Gesture::PeaceSign: return "Peace Sign";
        case esp_hand_tracker::Gesture::Pointing:  return "Pointing";
        case esp_hand_tracker::Gesture::ThumbsUp:  return "Thumbs Up";
        default: return "Unknown";
    }
}

static void tracker_task(void*) {
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    esp_hand_tracker::HandTracker tracker;
    size_t model_size = (model_end > model_start) ? (model_end - model_start) : 0;
    tracker.begin(model_start, model_size);

    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            auto result = tracker.process(fb->buf, fb->width, fb->height);
            if (result.detected) {
                printf("[Hand Detected] Gesture: %s | Wrist=(%.2f, %.2f) IndexTip=(%.2f, %.2f)\n",
                       gesture_to_string(result.gesture),
                       result.joint(esp_hand_tracker::Joint::Wrist).x,
                       result.joint(esp_hand_tracker::Joint::Wrist).y,
                       result.joint(esp_hand_tracker::Joint::IndexTip).x,
                       result.joint(esp_hand_tracker::Joint::IndexTip).y);
            }
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main() {
    xTaskCreatePinnedToCore(tracker_task, "tracker", 8192, nullptr, 5, nullptr, 1);
}
