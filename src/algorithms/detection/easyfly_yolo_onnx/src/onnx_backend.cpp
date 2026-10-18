// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// EasyFly P2: OnnxRuntime 后端实现
// 数据流: BGR 图 → letterbox → RGB float32 NCHW → Run → [1,4+nc,8400]
//         → 转置解析 (xywh, max conf, class) → 置信度过滤 → 按类别 NMS
// 与 Python 基准 (m2_onnx_verify.py) 完全同构, 结果可对拍。

#include "easyfly_yolo_onnx/onnx_backend.hpp"

#include <chrono>
#include <onnxruntime_cxx_api.h>
#include "easyfly_yolo_onnx/letterbox.hpp"

#include "easyfly_yolo_onnx/nms.hpp"

namespace easyfly::yolo_onnx
{
struct OnnxBackend::Impl
{
    explicit Impl(const DetectorParams & params);
    void detectImpl(const cv::Mat & bgr_img, std::vector<Detection> & detections, DetectionTiming * timing);
    DetectorParams params_;
    void createSession(const char * log_tag);
    void parseOutput(const float * output, int num_preds, int num_classes,
                     std::vector<Detection> & detections);

    Ort::Env env_{nullptr};
    Ort::SessionOptions session_opts_{nullptr};
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::string input_name_;
    std::vector<std::string> output_names_;

    // 预分配的推理缓冲
    std::vector<float> input_tensor_;        // 1*3*imgsz*imgsz
    std::vector<float> output_tensor_;       // 1*(4+nc)*num_preds
    LetterboxInfo last_lb_;
    std::string active_provider_;
    // 输出布局: num_classes_ 与 num_preds_ 从模型 shape 推断
    int num_classes_{0};
    int num_preds_{0};
};


OnnxBackend::Impl::Impl(const DetectorParams & params)
    : params_(params)
{
    env_ = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "easyfly_det");

    if (params_.use_gpu) {
        try {
            createSession("CUDA");
        } catch (const Ort::Exception & e) {
            fprintf(stderr,
                    "[easyfly_object_detection] CUDA EP 加载失败(%s), 自动回退 CPU EP\n",
                    e.what());
            createSession("CPU");
        }
    } else {
        createSession("CPU");
    }

    // 预分配输入缓冲
    input_tensor_.resize(3 * params_.imgsz * params_.imgsz);

    fprintf(stdout,
            "[easyfly_object_detection] OnnxBackend 就绪: %s | %s | imgsz=%d | classes=%zu | EP=%s\n",
            params_.model_path.c_str(),
            params_.class_names.empty() ? "?" : params_.class_names[0].c_str(),
            params_.imgsz, params_.class_names.size(), active_provider_.c_str());
}

void OnnxBackend::Impl::createSession(const char * log_tag)
{
    session_opts_ = Ort::SessionOptions();
    session_opts_.SetIntraOpNumThreads(2);
    session_opts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (std::string(log_tag) == "CUDA") {
        // device_id 显式设置, 部分驱动环境自动选择不可靠
        Ort::ThrowOnError(
            OrtSessionOptionsAppendExecutionProvider_CUDA(session_opts_, params_.gpu_device_id));
    }

    session_ = std::make_unique<Ort::Session>(env_, params_.model_path.c_str(), session_opts_);
    active_provider_ = std::string(log_tag) == "CUDA"
                           ? "CUDAExecutionProvider" : "CPUExecutionProvider";

    // 输入/输出名
    const auto inp = session_->GetInputTypeInfo(0);
    input_name_ = session_->GetInputNameAllocated(0, allocator_).get();

    for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
        output_names_.push_back(session_->GetOutputNameAllocated(i, allocator_).get());
    }

    // 校验输入 shape [1,3,?,?] 并推断输出布局 [1, 4+nc, num_preds]
    auto inp_shape = inp.GetTensorTypeAndShapeInfo().GetShape();
    if (inp_shape.size() != 4 || inp_shape[1] != 3) {
        throw std::runtime_error("模型输入 shape 不是 [N,3,H,W]: 期望 4 维 3 通道");
    }
    if (inp_shape[2] != params_.imgsz || inp_shape[3] != params_.imgsz) {
        char buf[256];
        snprintf(buf, sizeof(buf), "模型输入尺寸 %lld 与配置 imgsz %d 不一致",
                 static_cast<long long>(inp_shape[2]), params_.imgsz);
        throw std::runtime_error(buf);
    }

    auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    // YOLO11 导出布局: [1, 4+nc, num_preds]
    if (out_shape.size() == 3 && out_shape[2] > out_shape[1]) {
        num_classes_ = static_cast<int>(out_shape[1]) - 4;
        num_preds_ = static_cast<int>(out_shape[2]);
    } else if (out_shape.size() == 3) {
        // 转置布局 [1, num_preds, 4+nc]
        num_preds_ = static_cast<int>(out_shape[1]);
        num_classes_ = static_cast<int>(out_shape[2]) - 4;
    } else {
        throw std::runtime_error("模型输出 shape 不是 3 维 [N,4+nc,num_preds]");
    }

    if (!params_.class_names.empty() &&
        static_cast<int>(params_.class_names.size()) != num_classes_) {
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "类别数不匹配: 模型输出 %d 类, 配置 class_names 有 %zu 个 — 请检查 YAML",
                 num_classes_, params_.class_names.size());
        throw std::runtime_error(buf);
    }

    output_tensor_.resize((4 + num_classes_) * num_preds_);
}

void OnnxBackend::Impl::detectImpl(const cv::Mat & bgr_img, std::vector<Detection> & detections,
                             DetectionTiming * timing)
{
    detections.clear();

    // ---- 前处理: BGR → letterbox → RGB float NCHW ----
    const auto preprocess_start = std::chrono::steady_clock::now();
    cv::Mat lb;
    last_lb_ = letterbox(bgr_img, lb, params_.imgsz);

    const int sz = params_.imgsz;
    // BGR→RGB + HWC→CHW + /255, 一次性循环完成
    cv::Mat rgb;
    cv::cvtColor(lb, rgb, cv::COLOR_BGR2RGB);
    const uint8_t * p = rgb.data;
    const size_t plane = static_cast<size_t>(sz) * sz;
    float * ch_r = input_tensor_.data();
    float * ch_g = ch_r + plane;
    float * ch_b = ch_g + plane;
    for (size_t i = 0; i < plane; ++i) {
        ch_r[i] = p[i * 3 + 0] / 255.0f;
        ch_g[i] = p[i * 3 + 1] / 255.0f;
        ch_b[i] = p[i * 3 + 2] / 255.0f;
    }
    if (timing != nullptr) {
        timing->preprocess_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - preprocess_start).count();
    }

    // ---- 推理 ----
    const std::array<int64_t, 4> in_shape{1, 3, sz, sz};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(
        memory_info, input_tensor_.data(), input_tensor_.size(),
        in_shape.data(), in_shape.size());

    const char * input_names_c[] = {input_name_.c_str()};
    std::vector<const char *> output_names_c;
    for (const auto & n : output_names_) output_names_c.push_back(n.c_str());

    const auto inference_start = std::chrono::steady_clock::now();
    auto outputs = session_->Run(
        Ort::RunOptions{nullptr}, input_names_c, &input, 1,
        output_names_c.data(), output_names_c.size());
    if (timing != nullptr) {
        timing->inference_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - inference_start).count();
    }

    // ---- 后处理 ----
    const auto postprocess_start = std::chrono::steady_clock::now();
    const float * out = outputs[0].GetTensorData<float>();
    parseOutput(out, num_preds_, num_classes_, detections);
    if (timing != nullptr) {
        timing->postprocess_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - postprocess_start).count();
    }
}

void OnnxBackend::Impl::parseOutput(const float * output, int num_preds, int num_classes,
                              std::vector<Detection> & detections)
{
    // 模型布局 [1, 4+nc, num_preds] → 逐候选遍历 (列优先)
    std::vector<Detection> candidates;
    candidates.reserve(128);

    for (int p = 0; p < num_preds; ++p) {
        // 找该候选的最大类别置信度
        int best_cls = -1;
        float best_conf = 0.f;
        for (int c = 0; c < num_classes; ++c) {
            const float conf = output[(4 + c) * num_preds + p];
            if (conf > best_conf) {
                best_conf = conf;
                best_cls = c;
            }
        }
        if (best_conf < params_.conf_threshold) continue;

        const float cx = output[0 * num_preds + p];
        const float cy = output[1 * num_preds + p];
        const float w = output[2 * num_preds + p];
        const float h = output[3 * num_preds + p];

        Detection d;
        d.x1 = cx - w * 0.5f;
        d.y1 = cy - h * 0.5f;
        d.x2 = cx + w * 0.5f;
        d.y2 = cy + h * 0.5f;
        d.confidence = best_conf;
        d.class_id = best_cls;
        if (best_cls >= 0 && best_cls < static_cast<int>(params_.class_names.size())) {
            d.class_name = params_.class_names[best_cls];
        }
        candidates.push_back(d);
    }

    // 按类别 NMS
    detections = nms_by_class(candidates, static_cast<float>(params_.nms_threshold));
}

}  // namespace easyfly::yolo_onnx

namespace easyfly::yolo_onnx
{
OnnxBackend::OnnxBackend(const DetectorParams & params)
: DetectorBase(params), impl_(std::make_unique<Impl>(params)) {}
OnnxBackend::~OnnxBackend() = default;
std::string OnnxBackend::activeProvider() const {return impl_->active_provider_;}
void OnnxBackend::detectImpl(const cv::Mat & image, std::vector<Detection> & detections,
  DetectionTiming * timing) {impl_->detectImpl(image, detections, timing);}
void OnnxBackend::unletterboxImpl(Detection & d)
{unletterbox(d.x1, d.y1, d.x2, d.y2, impl_->last_lb_);}
}
