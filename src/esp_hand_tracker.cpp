/**
 * @file esp_hand_tracker.cpp
 * @brief Implementation of ESP32-S3 Hand Landmark & Gesture Tracker Library.
 */

#include "esp_hand_tracker.hpp"

#include <cstring>
#include <cmath>
#include <algorithm>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dl_model_base.hpp"
#include "dl_tool.hpp"

static const char* TAG = "esp_hand_tracker";

namespace esp_hand_tracker {

// ---------------------------------------------------------------------------
// Gesture recognition helper implementation
// ---------------------------------------------------------------------------
bool TrackResult::is_finger_extended(Joint tip, Joint mcp) const {
    const Point2f& pt_tip = joint(tip);
    const Point2f& pt_mcp = joint(mcp);
    const Point2f& pt_wrist = joint(Joint::Wrist);

    // Euclidean distance squared from wrist
    float dist_tip = (pt_tip.x - pt_wrist.x) * (pt_tip.x - pt_wrist.x) +
                     (pt_tip.y - pt_wrist.y) * (pt_tip.y - pt_wrist.y);
    float dist_mcp = (pt_mcp.x - pt_wrist.x) * (pt_mcp.x - pt_wrist.x) +
                     (pt_mcp.y - pt_wrist.y) * (pt_mcp.y - pt_wrist.y);

    return dist_tip > dist_mcp * 1.35f;
}

static Gesture classify_gesture(const TrackResult& res) {
    if (!res.detected) return Gesture::Unknown;

    bool thumb_ext  = res.is_finger_extended(Joint::ThumbTip, Joint::ThumbMCP);
    bool index_ext  = res.is_finger_extended(Joint::IndexTip, Joint::IndexMCP);
    bool middle_ext = res.is_finger_extended(Joint::MiddleTip, Joint::MiddleMCP);
    bool ring_ext   = res.is_finger_extended(Joint::RingTip, Joint::RingMCP);
    bool pinky_ext  = res.is_finger_extended(Joint::PinkyTip, Joint::PinkyMCP);

    // Pinch check (Thumb tip distance to Index tip)
    const Point2f& t = res.joint(Joint::ThumbTip);
    const Point2f& i = res.joint(Joint::IndexTip);
    float pinch_dist = (t.x - i.x) * (t.x - i.x) + (t.y - i.y) * (t.y - i.y);
    if (pinch_dist < 0.005f) {
        return Gesture::Pinch;
    }

    // Open Palm (all 5 fingers extended)
    if (index_ext && middle_ext && ring_ext && pinky_ext && thumb_ext) {
        return Gesture::OpenPalm;
    }

    // Fist (all curled)
    if (!index_ext && !middle_ext && !ring_ext && !pinky_ext && !thumb_ext) {
        return Gesture::Fist;
    }

    // Peace sign (Index + Middle extended, others curled)
    if (index_ext && middle_ext && !ring_ext && !pinky_ext) {
        return Gesture::PeaceSign;
    }

    // Pointing (Only Index extended)
    if (index_ext && !middle_ext && !ring_ext && !pinky_ext) {
        return Gesture::Pointing;
    }

    // Thumbs Up (Thumb extended, others curled)
    if (thumb_ext && !index_ext && !middle_ext && !ring_ext && !pinky_ext) {
        return Gesture::ThumbsUp;
    }

    return Gesture::Unknown;
}

// ---------------------------------------------------------------------------
// Internal Tracker State
// ---------------------------------------------------------------------------
struct HandTracker::Impl {
    dl::Model* model = nullptr;
    int8_t* input_buf = nullptr;
    uint8_t* crop_buf = nullptr;

    float input_scale = 1.0f;
    int input_zero_point = 0;
    int8_t quant_lut[256]{};

    TrackerConfig config{};
    BoundingBox current_roi{32.0f, 12.0f, 96.0f};
    bool is_tracking = false;
    int lost_frames = 0;
    bool ready = false;
};

// ---------------------------------------------------------------------------
// Construction & Destruction
// ---------------------------------------------------------------------------
HandTracker::HandTracker() : impl_(std::make_unique<Impl>()) {}

HandTracker::~HandTracker() {
    if (impl_) {
        if (impl_->model) {
            delete impl_->model;
            impl_->model = nullptr;
        }
        if (impl_->input_buf) {
            free(impl_->input_buf);
            impl_->input_buf = nullptr;
        }
        if (impl_->crop_buf) {
            free(impl_->crop_buf);
            impl_->crop_buf = nullptr;
        }
    }
}

bool HandTracker::is_ready() const {
    return impl_ && impl_->ready;
}

void HandTracker::reset() {
    if (impl_) {
        impl_->is_tracking = false;
        impl_->lost_frames = 0;
        impl_->current_roi = {32.0f, 12.0f, 96.0f};
    }
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
bool HandTracker::begin(const uint8_t* model_data, size_t model_size, const TrackerConfig& config) {
    if (!impl_) return false;
    impl_->config = config;

    if (!model_data || model_size < 128) {
        ESP_LOGE(TAG, "Invalid model data (size=%u bytes)", (unsigned)model_size);
        return false;
    }

    // Load ESP-DL model from memory blob
    impl_->model = new dl::Model();
    if (!impl_->model->load(model_data, model_size)) {
        ESP_LOGE(TAG, "Failed to load model into ESP-DL");
        delete impl_->model;
        impl_->model = nullptr;
        return false;
    }

    const size_t pixel_count = kModelInputSize * kModelInputSize;

    // Allocate quantized input buffer (prefer internal SRAM for maximum memory speed)
    impl_->input_buf = static_cast<int8_t*>(
        heap_caps_malloc(pixel_count, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL)
    );
    if (!impl_->input_buf) {
        impl_->input_buf = static_cast<int8_t*>(heap_caps_malloc(pixel_count, MALLOC_CAP_SPIRAM));
    }
    if (!impl_->input_buf) {
        ESP_LOGE(TAG, "Failed to allocate model input buffer (%u bytes)", (unsigned)pixel_count);
        return false;
    }

    // Allocate crop buffer
    impl_->crop_buf = static_cast<uint8_t*>(
        heap_caps_malloc(pixel_count, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL)
    );
    if (!impl_->crop_buf) {
        impl_->crop_buf = static_cast<uint8_t*>(malloc(pixel_count));
    }

    // Read quantization scale & zero point
    auto* in_tensor = impl_->model->get_input(0);
    if (in_tensor) {
        impl_->input_scale = in_tensor->get_scale();
        impl_->input_zero_point = in_tensor->get_zero_point();
    }

    // Precompute 256-entry quantization LUT
    const float inv_scale = (impl_->input_scale > 0.0f) ? (1.0f / impl_->input_scale) : 1.0f;
    const int zp = impl_->input_zero_point;
    for (int i = 0; i < 256; ++i) {
        float norm = static_cast<float>(i) / 255.0f;
        int32_t q = static_cast<int32_t>(roundf(norm * inv_scale)) + zp;
        impl_->quant_lut[i] = static_cast<int8_t>(q < -128 ? -128 : (q > 127 ? 127 : q));
    }

    impl_->ready = true;
    reset();
    ESP_LOGI(TAG, "ESP Hand Tracker initialized (scale=%.4f, zp=%d)",
             impl_->input_scale, impl_->input_zero_point);
    return true;
}

// ---------------------------------------------------------------------------
// Crop & Scaling Utility (<0.2ms)
// ---------------------------------------------------------------------------
static void fast_crop_rescale(
    const uint8_t* src, int src_w, int src_h,
    int rx, int ry, int rsize,
    uint8_t* dst, int dst_size
) {
    int step = (rsize << 16) / dst_size;

    for (int dy = 0; dy < dst_size; ++dy) {
        int sy = ry + ((dy * step) >> 16);
        if (sy < 0) sy = 0;
        if (sy >= src_h) sy = src_h - 1;

        const uint8_t* s_row = src + sy * src_w;
        uint8_t* d_row = dst + dy * dst_size;

        for (int dx = 0; dx < dst_size; ++dx) {
            int sx = rx + ((dx * step) >> 16);
            if (sx < 0) sx = 0;
            if (sx >= src_w) sx = src_w - 1;
            d_row[dx] = s_row[sx];
        }
    }
}

// Fast full-scene acquisition scanner
static bool scan_scene_for_hand(const uint8_t* frame, int w, int h, BoundingBox& roi) {
    int sum_x = 0;
    int sum_y = 0;
    int count = 0;

    for (int y = 6; y < h - 6; y += 3) {
        for (int x = 6; x < w - 6; x += 3) {
            uint8_t px = frame[y * w + x];
            if (px > 65 && px < 215) {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    if (count > 50) {
        float cx = static_cast<float>(sum_x) / count;
        float cy = static_cast<float>(sum_y) / count;
        float box_size = std::min(w, h) * 0.75f;

        roi.x = std::clamp(cx - box_size * 0.5f, 0.0f, static_cast<float>(w - box_size));
        roi.y = std::clamp(cy - box_size * 0.5f, 0.0f, static_cast<float>(h - box_size));
        roi.size = box_size;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Processing: Crop Only
// ---------------------------------------------------------------------------
TrackResult HandTracker::process_crop(const uint8_t* gray_96x96) {
    TrackResult result;
    if (!impl_ || !impl_->ready || !gray_96x96) return result;

    const size_t pixel_count = kModelInputSize * kModelInputSize;

    // Fast LUT input quantization
    for (size_t i = 0; i < pixel_count; ++i) {
        impl_->input_buf[i] = impl_->quant_lut[gray_96x96[i]];
    }

    auto* in_tensor = impl_->model->get_input(0);
    if (in_tensor) {
        std::memcpy(in_tensor->get_data(), impl_->input_buf, pixel_count);
    }

    // Forward pass
    impl_->model->run();

    auto* out_tensor = impl_->model->get_output(0);
    if (!out_tensor) return result;

    const float out_scale = out_tensor->get_scale();
    const int out_zp = out_tensor->get_zero_point();
    const int8_t* out_data = static_cast<const int8_t*>(out_tensor->get_data());

    result.detected = true;
    for (int i = 0; i < kNumLandmarks; ++i) {
        float x = (out_data[i * 2 + 0] - out_zp) * out_scale;
        float y = (out_data[i * 2 + 1] - out_zp) * out_scale;
        x = std::clamp(x, 0.0f, 1.0f);
        y = std::clamp(y, 0.0f, 1.0f);
        result.crop_landmarks[i] = {x, y};
        result.landmarks[i] = {x, y};
    }

    result.gesture = classify_gesture(result);
    return result;
}

// ---------------------------------------------------------------------------
// Processing: Wide-View Frame with Adaptive ROI
// ---------------------------------------------------------------------------
TrackResult HandTracker::process(const uint8_t* gray_frame, int width, int height) {
    TrackResult result;
    if (!impl_ || !impl_->ready || !gray_frame) return result;

    // Scan scene if tracking was lost
    if (!impl_->is_tracking && impl_->config.auto_scan_on_lost) {
        if (scan_scene_for_hand(gray_frame, width, height, impl_->current_roi)) {
            impl_->is_tracking = true;
            impl_->lost_frames = 0;
        }
    }

    // Clamp ROI inside frame
    int rx = std::clamp(static_cast<int>(impl_->current_roi.x), 0, width - kModelInputSize);
    int ry = std::clamp(static_cast<int>(impl_->current_roi.y), 0, height - kModelInputSize);
    int rsize = std::clamp(static_cast<int>(impl_->current_roi.size), 48, std::min(width, height));
    if (rx + rsize > width)  rx = width - rsize;
    if (ry + rsize > height) ry = height - rsize;

    // Fast crop and scale to 96x96
    fast_crop_rescale(gray_frame, width, height, rx, ry, rsize, impl_->crop_buf, kModelInputSize);

    // Run inference on cropped patch
    result = process_crop(impl_->crop_buf);
    result.roi = {static_cast<float>(rx), static_cast<float>(ry), static_cast<float>(rsize)};

    if (result.detected) {
        // Map local crop coordinates [0, 1] back to full frame space [0, 1]
        for (int i = 0; i < kNumLandmarks; ++i) {
            float fx = (rx + result.crop_landmarks[i].x * rsize) / static_cast<float>(width);
            float fy = (ry + result.crop_landmarks[i].y * rsize) / static_cast<float>(height);
            result.landmarks[i] = {std::clamp(fx, 0.0f, 1.0f), std::clamp(fy, 0.0f, 1.0f)};
        }

        // Calculate landmark extent
        float min_x = 1.0f, max_x = 0.0f;
        float min_y = 1.0f, max_y = 0.0f;
        for (int i = 0; i < kNumLandmarks; ++i) {
            min_x = std::min(min_x, result.crop_landmarks[i].x);
            max_x = std::max(max_x, result.crop_landmarks[i].x);
            min_y = std::min(min_y, result.crop_landmarks[i].y);
            max_y = std::max(max_y, result.crop_landmarks[i].y);
        }

        float span_w = max_x - min_x;
        float span_h = max_y - min_y;

        // If hand is clearly visible, update tracking box via Exponential Moving Average (EMA)
        if (span_w > 0.12f && span_h > 0.12f && span_w < 0.95f && span_h < 0.95f) {
            float cx_crop = (min_x + max_x) * 0.5f;
            float cy_crop = (min_y + max_y) * 0.5f;

            float new_center_x = rx + cx_crop * rsize;
            float new_center_y = ry + cy_crop * rsize;
            float target_size = std::max(span_w, span_h) * rsize * 1.35f;
            target_size = std::clamp(target_size, 56.0f, static_cast<float>(std::min(width, height)));

            float alpha = std::clamp(impl_->config.smoothing_factor, 0.1f, 0.9f);
            impl_->current_roi.size = (1.0f - alpha) * impl_->current_roi.size + alpha * target_size;

            float target_x = new_center_x - impl_->current_roi.size * 0.5f;
            float target_y = new_center_y - impl_->current_roi.size * 0.5f;

            impl_->current_roi.x = (1.0f - alpha) * impl_->current_roi.x + alpha * target_x;
            impl_->current_roi.y = (1.0f - alpha) * impl_->current_roi.y + alpha * target_y;

            impl_->current_roi.x = std::clamp(impl_->current_roi.x, 0.0f, static_cast<float>(width - impl_->current_roi.size));
            impl_->current_roi.y = std::clamp(impl_->current_roi.y, 0.0f, static_cast<float>(height - impl_->current_roi.size));

            impl_->is_tracking = true;
            impl_->lost_frames = 0;
        } else {
            impl_->lost_frames++;
            if (impl_->lost_frames > impl_->config.lost_threshold) {
                impl_->is_tracking = false;
            }
        }

        // Re-classify gesture with full-frame context
        result.gesture = classify_gesture(result);
    }

    return result;
}

}  // namespace esp_hand_tracker
