// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_yolo_onnx/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

// EasyFly P2: letterbox 前处理 (与 ultralytics 导出模型对齐)
// 等比缩放 + 居中填充 114, 输出 imgsz x imgsz x 3 (RGB, float32/255, NCHW)
#ifndef EASYFLY_YOLO_ONNX_LETTERBOX_HPP_
#define EASYFLY_YOLO_ONNX_LETTERBOX_HPP_

#include <opencv2/opencv.hpp>

namespace easyfly::yolo_onnx
{

struct LetterboxInfo {
    float scale{1.0f};   // 原图→缩放图比例
    int pad_x{0};        // 水平填充 (左)
    int pad_y{0};        // 垂直填充 (上)
};

/// src (任意尺寸 BGR/RGB) → dst (imgsz x imgsz, 与输入同通道序)
inline LetterboxInfo letterbox(const cv::Mat & src, cv::Mat & dst, int imgsz)
{
    LetterboxInfo info;
    const float r = std::min(
        static_cast<float>(imgsz) / src.rows,
        static_cast<float>(imgsz) / src.cols);
    info.scale = r;
    const int nw = std::round(src.cols * r);
    const int nh = std::round(src.rows * r);
    info.pad_x = (imgsz - nw) / 2;
    info.pad_y = (imgsz - nh) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    dst = cv::Mat(imgsz, imgsz, src.type(), cv::Scalar(114, 114, 114));
    resized.copyTo(dst(cv::Rect(info.pad_x, info.pad_y, nw, nh)));
    return info;
}

/// letterbox 坐标 → 原图坐标 (单个框)
inline void unletterbox(float & x1, float & y1, float & x2, float & y2,
                        const LetterboxInfo & info)
{
    x1 = (x1 - info.pad_x) / info.scale;
    x2 = (x2 - info.pad_x) / info.scale;
    y1 = (y1 - info.pad_y) / info.scale;
    y2 = (y2 - info.pad_y) / info.scale;
}

}  // namespace easyfly::yolo_onnx

#endif  // EASYFLY_YOLO_ONNX_LETTERBOX_HPP_
