// EasyFly M4: SORT 主类 (Track 生命周期 + 关联流程)
// 论文: Bewley et al., "Simple Online and Realtime Tracking", 2016
#ifndef EASYFLY_SORT_SORT_HPP_
#define EASYFLY_SORT_SORT_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "easyfly_sort/kalman.hpp"
#include "easyfly_sort/hungarian.hpp"

namespace easyfly::sort
{

/// 输入: 一帧检测 (像素坐标)
struct DetectionInput {
    BBox box;
    double confidence{};
    int class_id{-1};
    std::string class_name;
};

/// 输出: 一条跟踪状态
struct TrackOutput {
    int track_id{};
    BBox box;                 // 滤波后的框 (像素)
    BBox box_raw;             // 最近一次观测框 (像素)
    double confidence{};
    int class_id{-1};
    std::string class_name;
    double vx{}, vy{};        // 像素速度 (px/frame, 由滤波估计)
    int hits{};               // 累计命中
    int age{};                // 存活帧数
    int time_since_update{};  // 距上次命中帧数
    bool confirmed{};         // 是否已确认 (hits >= min_hits)
};

struct SortParams {
    double iou_threshold{0.3};   // 关联的最小 IoU
    int max_lost_frames{30};     // 丢失多少帧后删除
    int min_hits{3};             // 确认 track 需要的累计命中
    int max_tracks{100};         // 轨迹数上限
    // Q19: 中心距离门限系数 — IoU 门限对小目标/快速移动脆弱 (远处 person 框仅
    // 20×50px, 移动 15px 即 IoU≈0.2), 双门限 "IoU 达标 或 中心距达标" 才允许关联,
    // 代价仍由 IoU 主导 (匈牙利解质量不变, 仅放宽进入解的合法性)
    double dist_gate_scale{1.5};
};

class Sorter
{
public:
    explicit Sorter(const SortParams & params = SortParams()) : params_(params) {}

    /// 处理一帧检测, 返回当前所有存活 track (含未确认)
    std::vector<TrackOutput> update(const std::vector<DetectionInput> & detections)
    {
        // ---- 1. 全部 track 预测 ----
        for (auto & t : tracks_) t.kf.predict();

        // ---- 2. 关联 (代价 = 1 - IoU) ----
        const int n = static_cast<int>(tracks_.size());
        const int m = static_cast<int>(detections.size());
        std::vector<int> assignment(n, -1);
        std::vector<int> det_used(m, 0);

        if (n > 0 && m > 0) {
            std::vector<std::vector<double>> cost(n, std::vector<double>(m, 1.0));
            for (int i = 0; i < n; ++i) {
                const BBox pred = predictedBox(tracks_[i]);
                // Q19: 预测框几何量 (中心/对角线), 用于中心距离门限
                const double pcx = (pred.x1 + pred.x2) * 0.5;
                const double pcy = (pred.y1 + pred.y2) * 0.5;
                const double pw = pred.x2 - pred.x1, ph = pred.y2 - pred.y1;
                const double pred_diag = std::sqrt(pw * pw + ph * ph);
                for (int j = 0; j < m; ++j) {
                    // 类别不同的目标不允许关联
                    if (tracks_[i].class_id >= 0 && detections[j].class_id >= 0 &&
                        tracks_[i].class_id != detections[j].class_id) {
                        cost[i][j] = 2.0;  // 超过 max_cost=1.0, 必然拒绝
                        continue;
                    }
                    const double iou = pred.iou(detections[j].box);
                    // Q19: 中心距离门限 — 中心距 < max(两框对角线)×scale 视为可关联
                    // (对快速移动的小目标, IoU 秒崩而中心距离仍稳健)
                    const auto & db = detections[j].box;
                    const double dcx = (db.x1 + db.x2) * 0.5 - pcx;
                    const double dcy = (db.y1 + db.y2) * 0.5 - pcy;
                    const double dw = db.x2 - db.x1, dh = db.y2 - db.y1;
                    const double det_diag = std::sqrt(dw * dw + dh * dh);
                    const double dist = std::sqrt(dcx * dcx + dcy * dcy);
                    const bool iou_ok   = iou >= params_.iou_threshold;
                    const bool dist_ok  = dist < std::max(pred_diag, det_diag) * params_.dist_gate_scale;
                    if (!(iou_ok || dist_ok)) {
                        cost[i][j] = 2.0;   // 双门限都不满足 → 拒绝关联
                    } else {
                        cost[i][j] = 1.0 - iou;  // 代价仍由 IoU 主导 (关联质量)
                    }
                }
            }
            // Q19: gate 已在 cost 矩阵中体现 (2.0 拒绝), 匹配代价上界放宽为 1.0
            const double max_cost = 1.0;
            assignment = hungarianAssign(cost, max_cost);
            for (int i = 0; i < n; ++i)
                if (assignment[i] >= 0) det_used[assignment[i]] = 1;
        }

        // ---- 3. 更新已匹配 track ----
        for (int i = 0; i < n; ++i) {
            auto & t = tracks_[i];
            t.age++;
            if (assignment[i] >= 0) {
                const auto & det = detections[assignment[i]];
                t.kf.update(det.box.x1 + (det.box.x2 - det.box.x1) / 2,
                            det.box.y1 + (det.box.y2 - det.box.y1) / 2,
                            det.box.x2 - det.box.x1, det.box.y2 - det.box.y1);
                t.box_raw = det.box;
                t.confidence = det.confidence;
                t.class_id = det.class_id;
                if (!det.class_name.empty()) t.class_name = det.class_name;
                t.hits++;
                t.time_since_update = 0;
            } else {
                t.time_since_update++;
            }
        }

        // ---- 4. 未匹配检测 → 新建 track ----
        for (int j = 0; j < m; ++j) {
            if (det_used[j]) continue;
            if (static_cast<int>(tracks_.size()) >= params_.max_tracks) break;
            const auto & det = detections[j];
            Track t;
            t.kf.init(det.box.x1 + (det.box.x2 - det.box.x1) / 2,
                      det.box.y1 + (det.box.y2 - det.box.y1) / 2,
                      det.box.x2 - det.box.x1, det.box.y2 - det.box.y1);
            t.track_id = next_id_++;
            t.box_raw = det.box;
            t.confidence = det.confidence;
            t.class_id = det.class_id;
            t.class_name = det.class_name;
            t.hits = 1;
            t.age = 1;
            t.time_since_update = 0;
            tracks_.push_back(t);
        }

        // ---- 5. 删除超时 track ----
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [this](const Track & t) {
                               return t.time_since_update > params_.max_lost_frames;
                           }),
            tracks_.end());

        // ---- 6. 输出 ----
        std::vector<TrackOutput> out;
        out.reserve(tracks_.size());
        for (const auto & t : tracks_) {
            TrackOutput o;
            o.track_id = t.track_id;
            o.box = predictedBox(t);
            o.box_raw = t.box_raw;
            o.confidence = t.confidence;
            o.class_id = t.class_id;
            o.class_name = t.class_name;
            o.vx = t.kf.vcx();
            o.vy = t.kf.vcy();
            o.hits = t.hits;
            o.age = t.age;
            o.time_since_update = t.time_since_update;
            o.confirmed = t.hits >= params_.min_hits;
            out.push_back(o);
        }
        return out;
    }

    int activeCount() const { return static_cast<int>(tracks_.size()); }

private:
    struct Track {
        KalmanFilter kf;
        int track_id{-1};
        BBox box_raw;             // 最近观测
        double confidence{};
        int class_id{-1};
        std::string class_name;
        int hits{0};
        int age{0};
        int time_since_update{0};
    };

    static BBox predictedBox(const Track & t)
    {
        const double cx = t.kf.cx(), cy = t.kf.cy(), w = t.kf.w(), h = t.kf.h();
        return BBox{cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2};
    }

    SortParams params_;
    std::vector<Track> tracks_;
    int next_id_{0};
};

}  // namespace easyfly::sort

#endif  // EASYFLY_SORT_SORT_HPP_
