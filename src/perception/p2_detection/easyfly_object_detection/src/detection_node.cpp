// EasyFly P2: 目标检测 ROS2 组件节点
//
// 数据流:
//   /camera/color/image_raw (rgb8)
//     → OnnxRuntime 推理 (CUDA EP 优先)
//     → /detections              (easyfly_msgs/DetectionInfoArray, 归一化 bbox)
//     → /detections_debug_image  (画框可视化, 可配置关闭)
//
// 设计:
//   - 图像 rgb8 → cv::Mat BGR 手动转换 (不依赖 cv_bridge)
//   - 推理在专用线程执行, 不阻塞 executor spin
//   - max_fps 节流: 丢帧策略 (推理中收到新帧直接替换 pending)
#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include <easyfly_msgs/msg/detection_info_array.hpp>

#include "easyfly_object_detection/backend_factory.hpp"

namespace easyfly_object_detection
{

namespace
{
using SteadyClock = std::chrono::steady_clock;

/// rgb8 sensor_msgs → BGR cv::Mat (逐字节交换 R/B)
cv::Mat rgb8_to_bgr_mat(const sensor_msgs::msg::Image & msg)
{
    cv::Mat rgb(msg.height, msg.width, CV_8UC3,
                const_cast<uint8_t *>(msg.data.data()), msg.step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}

/// BGR cv::Mat → rgb8 sensor_msgs
sensor_msgs::msg::Image bgr_mat_to_rgb8(const cv::Mat & bgr, const std_msgs::msg::Header & header)
{
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = rgb.rows;
    msg.width = rgb.cols;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = rgb.cols * 3;
    msg.data.assign(rgb.data, rgb.data + rgb.total() * rgb.elemSize());
    return msg;
}

bool parse_bool_override(const std::string & value, const char * parameter_name)
{
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off") {
        return false;
    }
    throw std::runtime_error(
        std::string(parameter_name) + " must be auto, true/false, 1/0, yes/no, or on/off");
}

std::string normalize_reliability(const std::string & value)
{
    if (value == "reliable") {
        return value;
    }
    if (value == "best_effort" || value == "best-effort") {
        return "best_effort";
    }
    throw std::runtime_error(
        "input_qos_reliability must be reliable or best_effort");
}

int parse_positive_int(const std::string & value, const char * parameter_name)
{
    try {
        size_t consumed = 0;
        const int parsed = std::stoi(value, &consumed);
        if (consumed != value.size() || parsed <= 0) {
            throw std::invalid_argument("must be a positive integer");
        }
        return parsed;
    } catch (const std::exception & e) {
        throw std::runtime_error(
            std::string(parameter_name) + " must be a positive integer: " + e.what());
    }
}

struct TimingAccumulator
{
    void add(double value)
    {
        ++count;
        sum += value;
        maximum = std::max(maximum, value);
        samples.push_back(value);
    }

    double mean() const
    {
        return count == 0 ? 0.0 : sum / static_cast<double>(count);
    }

    double p95() const
    {
        if (samples.empty()) {
            return 0.0;
        }
        std::vector<double> sorted = samples;
        const size_t index = (sorted.size() * 95 + 99) / 100 - 1;
        std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
        return sorted[index];
    }

    void clear()
    {
        count = 0;
        sum = 0.0;
        maximum = 0.0;
        samples.clear();
    }

    uint64_t count{0};
    double sum{0.0};
    double maximum{0.0};
    std::vector<double> samples;
};

struct InputCounterSnapshot
{
    uint64_t received{0};
    uint64_t overwritten{0};
    bool pending{false};
};

// These counters are always accessed while pending_mtx_ is held.  Keeping
// them with the pending slot makes the callback count and replacement count
// one coherent state, rather than two independently sampled atomics.
struct CallbackCounters
{
    uint64_t received{0};
    uint64_t overwritten{0};
};
}  // namespace

class DetectionNode : public rclcpp::Node
{
public:
    explicit DetectionNode(const rclcpp::NodeOptions & options)
        : Node("easyfly_object_detection", options)
    {
        loadParams();

        // 后端初始化 + 预热 (阻塞 ~ 秒级, 放在构造函数保证就绪后才开始订阅)
        detector_ = createBackend(params_);
        detector_->warmUp();

        detections_pub_ = create_publisher<easyfly_msgs::msg::DetectionInfoArray>(
            params_.output_topic, 10);
        if (params_.publish_debug_image) {
            debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
                params_.output_topic + "_debug_image", 10);
        }

        rclcpp::QoS input_qos(rclcpp::KeepLast(
            static_cast<size_t>(params_.input_qos_depth)));
        input_qos.durability_volatile();
        if (params_.input_qos_reliability == "reliable") {
            input_qos.reliable();
        } else {
            input_qos.best_effort();
        }

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            params_.input_topic, input_qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                const auto received_at = SteadyClock::now();
                std::lock_guard<std::mutex> lk(pending_mtx_);
                ++callback_counters_.received;
                if (pending_ != nullptr) {
                    ++callback_counters_.overwritten;
                }
                pending_ = msg;               // 最新帧覆盖旧帧 (丢帧节流)
                pending_received_at_ = received_at;
                pending_cv_.notify_one();
            });

        resetPerformanceWindow();

        // 推理线程
        worker_ = std::thread([this] { workerLoop(); });

        RCLCPP_INFO(get_logger(),
                    "检测节点启动: %s → %s (%s, imgsz=%d, conf=%.2f, perf_period=%.1fs, "
                    "input_qos=%s/volatile/keep_last(%d))",
                    params_.input_topic.c_str(), params_.output_topic.c_str(),
                    params_.backend.c_str(), params_.imgsz, params_.conf_threshold,
                    params_.performance_log_period_s, params_.input_qos_reliability.c_str(),
                    params_.input_qos_depth);
    }

    ~DetectionNode() override
    {
        stop_ = true;
        {
            std::lock_guard<std::mutex> lk(pending_mtx_);
            pending_cv_.notify_all();
        }
        if (worker_.joinable()) worker_.join();
    }

private:
    void loadParams()
    {
        // config 文件路径 (launch 传入) 或默认包内 detection.yaml
        declare_parameter<std::string>("config_file", "");
        // Launch-facing overrides use the value "auto" to preserve the YAML
        // setting. This keeps the default CUDA path unchanged while allowing
        // an acceptance run to request CPU explicitly without copying YAML.
        declare_parameter<std::string>("use_gpu_override", "auto");
        declare_parameter<std::string>("publish_debug_image_override", "auto");
        declare_parameter<std::string>("performance_log_period_s_override", "auto");
        declare_parameter<std::string>("input_qos_reliability_override", "auto");
        declare_parameter<std::string>("input_qos_depth_override", "auto");
        std::string cfg = get_parameter("config_file").as_string();
        if (cfg.empty()) {
            cfg = ament_index_cpp::get_package_share_directory("easyfly_object_detection")
                  + "/config/detection.yaml";
        }

        YAML::Node root = YAML::LoadFile(cfg);
        const auto & d = root["detection"];

        params_.backend = d["backend"].as<std::string>("onnx");
        params_.model_path = d["model_path"].as<std::string>();
        const std::string packaged_model = "package://easyfly_object_detection/";
        if (params_.model_path.rfind(packaged_model, 0) == 0) {
            params_.model_path = ament_index_cpp::get_package_share_directory(
                "easyfly_object_detection") + "/" + params_.model_path.substr(packaged_model.size());
        }
        params_.imgsz = d["imgsz"].as<int>(640);
        params_.conf_threshold = d["conf_threshold"].as<double>(0.35);
        params_.nms_threshold = d["nms_threshold"].as<double>(0.45);
        params_.use_gpu = d["use_gpu"].as<bool>(true);
        params_.gpu_device_id = d["gpu_device_id"].as<int>(0);
        params_.max_detections = d["max_detections"].as<int>(100);
        params_.input_topic = d["input_topic"].as<std::string>("/camera/color/image_raw");
        params_.output_topic = d["output_topic"].as<std::string>("/detections");
        params_.input_qos_reliability = normalize_reliability(
            d["input_qos_reliability"].as<std::string>("reliable"));
        params_.input_qos_depth = d["input_qos_depth"].as<int>(5);
        params_.publish_debug_image = d["publish_debug_image"].as<bool>(true);
        params_.performance_log_period_s =
            d["performance_log_period_s"].as<double>(10.0);

        const auto use_gpu_override = get_parameter("use_gpu_override").as_string();
        if (use_gpu_override != "auto" && !use_gpu_override.empty()) {
            params_.use_gpu = parse_bool_override(use_gpu_override, "use_gpu_override");
        }
        const auto debug_image_override =
            get_parameter("publish_debug_image_override").as_string();
        if (debug_image_override != "auto" && !debug_image_override.empty()) {
            params_.publish_debug_image = parse_bool_override(
                debug_image_override, "publish_debug_image_override");
        }
        const auto perf_period_override =
            get_parameter("performance_log_period_s_override").as_string();
        if (perf_period_override != "auto" && !perf_period_override.empty()) {
            try {
                size_t consumed = 0;
                params_.performance_log_period_s = std::stod(perf_period_override, &consumed);
                if (consumed != perf_period_override.size()) {
                    throw std::invalid_argument("trailing characters");
                }
            } catch (const std::exception & e) {
                throw std::runtime_error(
                    "performance_log_period_s_override must be a number: " +
                    std::string(e.what()));
            }
        }
        if (!std::isfinite(params_.performance_log_period_s) ||
            params_.performance_log_period_s < 0.0) {
            throw std::runtime_error(
                "performance_log_period_s must be finite and >= 0 (0 disables metrics)");
        }

        const auto input_qos_reliability_override =
            get_parameter("input_qos_reliability_override").as_string();
        if (input_qos_reliability_override != "auto" &&
            !input_qos_reliability_override.empty()) {
            params_.input_qos_reliability = normalize_reliability(
                input_qos_reliability_override);
        }
        const auto input_qos_depth_override =
            get_parameter("input_qos_depth_override").as_string();
        if (input_qos_depth_override != "auto" && !input_qos_depth_override.empty()) {
            params_.input_qos_depth = parse_positive_int(
                input_qos_depth_override, "input_qos_depth_override");
        }
        if (params_.input_qos_depth <= 0 || params_.input_qos_depth > 100) {
            throw std::runtime_error("input_qos_depth must be in range [1, 100]");
        }

        if (d["class_names"]) {
            params_.class_names = d["class_names"].as<std::vector<std::string>>();
        }
        if (params_.class_names.empty()) {
            RCLCPP_FATAL(get_logger(), "detection.yaml 缺少 class_names");
            throw std::runtime_error("class_names empty");
        }

        max_interval_ = std::chrono::duration<double>(
            1.0 / d["max_fps"].as<double>(30.0));
    }

    void clearTimingAccumulators()
    {
        queue_stats_.clear();
        color_conversion_stats_.clear();
        preprocess_stats_.clear();
        inference_stats_.clear();
        postprocess_stats_.clear();
        debug_image_stats_.clear();
        end_to_end_stats_.clear();
    }

    // Must be called while pending_mtx_ is held.  The caller's lock scope
    // covers both the counter snapshot and the callback-window baseline.
    InputCounterSnapshot snapshotInputCountersLocked() const
    {
        return InputCounterSnapshot{
            callback_counters_.received,
            callback_counters_.overwritten,
            pending_ != nullptr};
    }

    void resetPerformanceWindow(SteadyClock::time_point now = SteadyClock::now())
    {
        {
            std::lock_guard<std::mutex> lk(pending_mtx_);
            const auto snapshot = snapshotInputCountersLocked();
            // The counter snapshot and callback baselines are established at
            // the same pending-slot boundary.
            perf_window_start_ = now;
            perf_last_callback_received_ = snapshot.received;
            perf_last_callback_overwritten_ = snapshot.overwritten;
            perf_last_processed_ = processed_frames_;
        }
        clearTimingAccumulators();
    }

    void logPerformance(SteadyClock::time_point now)
    {
        if (params_.performance_log_period_s <= 0.0) {
            return;
        }

        const double window_s = std::chrono::duration<double>(now - perf_window_start_).count();
        if (window_s < params_.performance_log_period_s) {
            return;
        }

        uint64_t received_baseline;
        uint64_t overwritten_baseline;
        uint64_t processed_delta;
        InputCounterSnapshot snapshot;
        {
            // Keep the callback counters and pending state from one logical
            // instant, then advance the callback baselines before releasing
            // the lock.  A callback arriving afterwards belongs to the next
            // window and cannot be lost or counted in the wrong window.
            std::lock_guard<std::mutex> lk(pending_mtx_);
            received_baseline = perf_last_callback_received_;
            overwritten_baseline = perf_last_callback_overwritten_;
            snapshot = snapshotInputCountersLocked();
            perf_last_callback_received_ = snapshot.received;
            perf_last_callback_overwritten_ = snapshot.overwritten;
            processed_delta = processed_frames_ - perf_last_processed_;
            perf_last_processed_ = processed_frames_;
            perf_window_start_ = now;
        }
        const uint64_t received_delta = snapshot.received - received_baseline;
        const uint64_t overwritten_delta = snapshot.overwritten - overwritten_baseline;
        const double callback_received_wall_hz =
            static_cast<double>(received_delta) / window_s;
        const double processed_wall_hz = static_cast<double>(processed_delta) / window_s;
        // 这里只统计回调内 pending 槽覆盖，不代表 DDS/发布端端到端丢帧。
        const double callback_overwrite_pct = received_delta == 0
                                                  ? 0.0
                                                  : 100.0 * static_cast<double>(overwritten_delta) /
                                                        static_cast<double>(received_delta);

        RCLCPP_INFO(
            get_logger(),
            "PERF steady window=%.2fs provider=%s callback_received=%llu "
            "callback_received_wall_hz=%.2f processed=%llu processed_wall_hz=%.2f "
            "callback_overwritten=%llu(callback_overwrite_pct=%.2f%%; pending replacement only, "
            "not DDS/publisher end-to-end loss) pending=%llu",
            window_s, detector_->activeProvider().c_str(),
            static_cast<unsigned long long>(received_delta), callback_received_wall_hz,
            static_cast<unsigned long long>(processed_delta), processed_wall_hz,
            static_cast<unsigned long long>(overwritten_delta), callback_overwrite_pct,
            static_cast<unsigned long long>(snapshot.pending ? 1U : 0U));
        RCLCPP_INFO(
            get_logger(),
            "PERF stage_ms avg/p95/max queue=%.3f/%.3f/%.3f color=%.3f/%.3f/%.3f "
            "preprocess=%.3f/%.3f/%.3f ort_run=%.3f/%.3f/%.3f "
            "postprocess_nms=%.3f/%.3f/%.3f debug=%.3f/%.3f/%.3f e2e=%.3f/%.3f/%.3f",
            queue_stats_.mean(), queue_stats_.p95(), queue_stats_.maximum,
            color_conversion_stats_.mean(), color_conversion_stats_.p95(),
            color_conversion_stats_.maximum,
            preprocess_stats_.mean(), preprocess_stats_.p95(), preprocess_stats_.maximum,
            inference_stats_.mean(), inference_stats_.p95(), inference_stats_.maximum,
            postprocess_stats_.mean(), postprocess_stats_.p95(), postprocess_stats_.maximum,
            debug_image_stats_.mean(), debug_image_stats_.p95(), debug_image_stats_.maximum,
            end_to_end_stats_.mean(), end_to_end_stats_.p95(), end_to_end_stats_.maximum);

        clearTimingAccumulators();
    }

    void workerLoop()
    {
        while (!stop_) {
            sensor_msgs::msg::Image::ConstSharedPtr frame;
            SteadyClock::time_point received_at;
            {
                std::unique_lock<std::mutex> lk(pending_mtx_);
                pending_cv_.wait(lk, [this] { return stop_ || pending_ != nullptr; });
                if (stop_) return;
                frame = pending_;
                received_at = pending_received_at_;
                pending_ = nullptr;
            }

            const auto processing_start = SteadyClock::now();
            const bool metrics_enabled = params_.performance_log_period_s > 0.0;
            DetectionTiming timing;
            if (metrics_enabled) {
                timing.queue_ms = std::chrono::duration<double, std::milli>(
                    processing_start - received_at).count();
            }

            const auto color_conversion_start =
                metrics_enabled ? SteadyClock::now() : SteadyClock::time_point{};
            cv::Mat bgr = rgb8_to_bgr_mat(*frame);
            if (metrics_enabled) {
                timing.color_conversion_ms = std::chrono::duration<double, std::milli>(
                    SteadyClock::now() - color_conversion_start).count();
            }
            std::vector<Detection> dets;
            detector_->detect(bgr, dets, metrics_enabled ? &timing : nullptr);

            // 发布 DetectionInfoArray (归一化)
            easyfly_msgs::msg::DetectionInfoArray msg;
            msg.header = frame->header;  // 保留原始时间戳, 供 M5 时空对齐
            msg.detections.reserve(dets.size());
            for (const auto & d : dets) {
                easyfly_msgs::msg::DetectionInfo info;
                info.header = frame->header;
                info.track_id = -1;  // M4 跟踪器填充
                info.class_name = d.class_name;
                info.confidence = d.confidence;
                // 归一化 (图像像素坐标)
                info.bbox_center_x = (d.x1 + d.x2) * 0.5f / bgr.cols;
                info.bbox_center_y = (d.y1 + d.y2) * 0.5f / bgr.rows;
                info.bbox_width = (d.x2 - d.x1) / bgr.cols;
                info.bbox_height = (d.y2 - d.y1) / bgr.rows;
                info.position_valid = false;  // M5 pixel_to_3d 填充
                msg.detections.push_back(info);
            }
            detections_pub_->publish(msg);

            // 调试图像
            const auto debug_image_start =
                metrics_enabled ? SteadyClock::now() : SteadyClock::time_point{};
            if (params_.publish_debug_image && debug_pub_->get_subscription_count() > 0) {
                for (const auto & d : dets) {
                    cv::rectangle(bgr,
                                  cv::Point(cvRound(d.x1), cvRound(d.y1)),
                                  cv::Point(cvRound(d.x2), cvRound(d.y2)),
                                  cv::Scalar(0, 255, 0), 2);
                    cv::putText(bgr,
                                d.class_name + " " + cv::format("%.2f", d.confidence),
                                cv::Point(cvRound(d.x1), cvRound(d.y1) - 4),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
                debug_pub_->publish(bgr_mat_to_rgb8(bgr, frame->header));
            }
            if (metrics_enabled) {
                timing.debug_image_ms = std::chrono::duration<double, std::milli>(
                    SteadyClock::now() - debug_image_start).count();
                timing.end_to_end_ms = std::chrono::duration<double, std::milli>(
                    SteadyClock::now() - received_at).count();
            }

            ++processed_frames_;
            if (metrics_enabled) {
                queue_stats_.add(timing.queue_ms);
                color_conversion_stats_.add(timing.color_conversion_ms);
                preprocess_stats_.add(timing.preprocess_ms);
                inference_stats_.add(timing.inference_ms);
                postprocess_stats_.add(timing.postprocess_ms);
                debug_image_stats_.add(timing.debug_image_ms);
                end_to_end_stats_.add(timing.end_to_end_ms);
                logPerformance(SteadyClock::now());
            }

            // 节流
            const auto elapsed = SteadyClock::now() - processing_start;
            if (elapsed < max_interval_) {
                std::this_thread::sleep_for(max_interval_ - elapsed);
            }
        }
    }

    DetectorParams params_;
    std::unique_ptr<DetectorBase> detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<easyfly_msgs::msg::DetectionInfoArray>::SharedPtr detections_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::mutex pending_mtx_;
    std::condition_variable pending_cv_;
    sensor_msgs::msg::Image::ConstSharedPtr pending_;
    SteadyClock::time_point pending_received_at_{};
    std::chrono::duration<double> max_interval_{0};

    CallbackCounters callback_counters_;
    uint64_t processed_frames_{0};
    SteadyClock::time_point perf_window_start_{};
    uint64_t perf_last_callback_received_{0};
    uint64_t perf_last_callback_overwritten_{0};
    uint64_t perf_last_processed_{0};
    TimingAccumulator queue_stats_;
    TimingAccumulator color_conversion_stats_;
    TimingAccumulator preprocess_stats_;
    TimingAccumulator inference_stats_;
    TimingAccumulator postprocess_stats_;
    TimingAccumulator debug_image_stats_;
    TimingAccumulator end_to_end_stats_;
};

}  // namespace easyfly_object_detection

RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_object_detection::DetectionNode)
