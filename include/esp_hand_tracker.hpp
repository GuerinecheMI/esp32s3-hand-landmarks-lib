#pragma once
/**
 * @file esp_hand_tracker.hpp
 * @brief High-performance 21-point Hand Landmark & Gesture Tracking Library for ESP32-S3.
 *
 * Designed for ESP-IDF and Arduino-ESP32 using ESP-DL hardware acceleration.
 */

#include <array>
#include <cstdint>
#include <cstddef>
#include <memory>

namespace esp_hand_tracker {

/// Number of anatomical landmarks tracked per hand
constexpr int kNumLandmarks = 21;

/// Spatial resolution of the underlying CNN model
constexpr int kModelInputSize = 96;

/// Landmark joint index enumeration (MediaPipe compatible)
enum class Joint : uint8_t {
    Wrist = 0,
    ThumbCMC = 1,
    ThumbMCP = 2,
    ThumbIP = 3,
    ThumbTip = 4,
    IndexMCP = 5,
    IndexPIP = 6,
    IndexDIP = 7,
    IndexTip = 8,
    MiddleMCP = 9,
    MiddlePIP = 10,
    MiddleDIP = 11,
    MiddleTip = 12,
    RingMCP = 13,
    RingPIP = 14,
    RingDIP = 15,
    RingTip = 16,
    PinkyMCP = 17,
    PinkyPIP = 18,
    PinkyDIP = 19,
    PinkyTip = 20,
};

/// High-level recognized gestures
enum class Gesture : uint8_t {
    Unknown = 0,
    OpenPalm,
    Fist,
    Pinch,       // Thumb tip close to index tip
    PeaceSign,   // Index + Middle extended
    Pointing,    // Only Index extended
    ThumbsUp,    // Thumb extended, others curled
};

/// 2D normalized coordinate point [0.0, 1.0]
struct Point2f {
    float x = 0.0f;
    float y = 0.0f;
};

/// Bounding region of interest in camera pixel coordinates
struct BoundingBox {
    float x = 0.0f;
    float y = 0.0f;
    float size = 0.0f;
};

/// Full tracking result
struct TrackResult {
    bool detected = false;
    BoundingBox roi{};
    std::array<Point2f, kNumLandmarks> landmarks{};       // Full-frame mapped coordinates [0, 1]
    std::array<Point2f, kNumLandmarks> crop_landmarks{};  // Local 96x96 crop coordinates [0, 1]
    Gesture gesture = Gesture::Unknown;

    /// Helper to get a specific joint coordinate
    const Point2f& joint(Joint j) const {
        return landmarks[static_cast<size_t>(j)];
    }

    /// Check if a finger is currently extended
    bool is_finger_extended(Joint tip, Joint mcp) const;
};

/// Configuration options for the tracker
struct TrackerConfig {
    int camera_width = 160;
    int camera_height = 120;
    float smoothing_factor = 0.35f;  // EMA smoothing for bounding box tracking (0.1 to 0.9)
    bool auto_scan_on_lost = true;   // Automatically scan scene when hand leaves view
    int lost_threshold = 6;          // Consecutive empty frames before resetting tracking
};

/**
 * @class HandTracker
 * @brief Main engine managing model loading, quantization LUT, adaptive tracking, and inference.
 */
class HandTracker {
public:
    HandTracker();
    ~HandTracker();

    HandTracker(const HandTracker&) = delete;
    HandTracker& operator=(const HandTracker&) = delete;

    /**
     * @brief Initialize the tracker with an embedded or external ESP-DL model.
     * @param model_data Pointer to .espdl binary data (nullptr to use built-in default if available).
     * @param model_size Size of model data in bytes.
     * @param config Optional configuration struct.
     * @return true if initialization succeeded.
     */
    bool begin(
        const uint8_t* model_data = nullptr,
        size_t model_size = 0,
        const TrackerConfig& config = TrackerConfig()
    );

    /**
     * @brief Process a camera frame in wide-view mode with adaptive ROI tracking.
     * @param gray_frame Pointer to 8-bit grayscale pixel buffer.
     * @param width Frame width (e.g. 160 for QQVGA, 320 for QVGA).
     * @param height Frame height (e.g. 120 for QQVGA, 240 for QVGA).
     * @return TrackResult with normalized landmarks and detected gesture.
     */
    TrackResult process(const uint8_t* gray_frame, int width, int height);

    /**
     * @brief Run landmark inference directly on a pre-cropped 96x96 grayscale image.
     * @param gray_96x96 Pointer to 96*96 row-major uint8 pixel array.
     * @return TrackResult containing landmarks in crop coordinate space.
     */
    TrackResult process_crop(const uint8_t* gray_96x96);

    /**
     * @brief Check if the model is loaded and ready for inference.
     */
    bool is_ready() const;

    /**
     * @brief Reset tracking state (forces scene re-scan on next frame).
     */
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace esp_hand_tracker
