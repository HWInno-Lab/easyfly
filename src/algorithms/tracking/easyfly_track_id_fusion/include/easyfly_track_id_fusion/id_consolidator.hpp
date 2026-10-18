// EasyFly M4-Q19: 世界系 id 整合器 (轻量 ReID)
//
// 解决的问题: SORT 在像素空间工作, 目标离开视野/遮挡超过 max_lost 后轨迹
// 删除, 重现时必然分配新 id (实测一次飞行 ~26 个 id)。调参治不了视野离开
// 数秒的情况; 完整外观 ReID 成本高。
//
// 本方案: 利用 /target_positions 提供的每个 id 的世界坐标 (odom), 在 SORT
// 新建 id 时查其世界位置是否落在最近消失的同类别 id 附近 (radius/ttl 内) —
// 是则继承旧 id。静态目标离视野再回来位置不变; 移动目标由 radius 门限容忍
// 位移。SORT 的 id 单调递增永不复用, remap 无歧义。
//
// 纯逻辑 header, 无 ROS 依赖 (可单元测试)。仅应在单一线程调用。
#ifndef EASYFLY_TRACK_ID_FUSION_ID_CONSOLIDATOR_HPP_
#define EASYFLY_TRACK_ID_FUSION_ID_CONSOLIDATOR_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace easyfly::track_id_fusion
{

struct WorldObs
{
  double x{}, y{}, z{};
  std::string cls;
};

class IdConsolidator
{
public:
  struct Params
  {
    bool enabled{true};
    double reacquire_radius{3.0};  // 世界系重关联门限 [m]
    double ttl{10.0};              // 消失 id 保留时长 [s]
    size_t max_graves{128};        // 墓园容量上界 (防无界增长)
  };

  IdConsolidator() = default;
  explicit IdConsolidator(const Params & params) : params_(params) {}

  /// 每个跟踪帧调用一次。
  /// sort_ids: 本帧 SORT 输出的全部 id (含未确认)。
  /// world_by_sort: 最新 /target_positions 中 valid 目标的
  ///                {SORT id → 世界观测} 快照 (可为空/滞后一帧)。
  /// now_s: 单调时钟秒。
  void update(const std::vector<int> & sort_ids,
              const std::map<int, WorldObs> & world_by_sort,
              double now_s)
  {
    purgeGraves(now_s);

    // 1. 本帧存活 id: 建立/沿用 remap, 刷新世界观测
    for (const int sid : sort_ids) {
      auto it = remap_.find(sid);
      if (it != remap_.end()) {
        // A new raw ID is published before pixel_to_3d can return its position.
        // Retry once when that first world observation arrives; never reassign
        // a track that already owns an observed identity or inherited a grave.
        const auto first = world_by_sort.find(sid);
        if (params_.enabled && it->second == sid &&
            last_world_.find(sid) == last_world_.end() && first != world_by_sort.end()) {
          const int grave = matchGrave(first->second, now_s);
          if (grave >= 0) {
            it->second = grave;
            removeGrave(grave);
          }
        }
        refreshWorld(it->second, sid, world_by_sort);
        continue;
      }
      // 新 SORT id: 尝试从墓园继承
      int stable = sid;
      const auto w = world_by_sort.find(sid);
      if (params_.enabled && w != world_by_sort.end()) {
        const int grave = matchGrave(w->second, now_s);
        if (grave >= 0) {
          stable = grave;
          removeGrave(grave);
        }
      }
      remap_[sid] = stable;
      refreshWorld(stable, sid, world_by_sort);
    }

    // 2. 死亡检测: 上帧存活、本帧消失 → 入墓园 (有世界观测才值得保留)
    for (auto it = remap_.begin(); it != remap_.end();) {
      if (std::find(sort_ids.begin(), sort_ids.end(), it->first) ==
          sort_ids.end())
      {
        const auto w = last_world_.find(it->second);
        if (w != last_world_.end()) {
          graveyard_.push_back({it->second, w->second, now_s});
          if (graveyard_.size() > params_.max_graves) graveyard_.pop_front();
        }
        last_world_.erase(it->second);
        it = remap_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /// SORT id → 对外稳定 id (未映射时原样返回)
  int stableId(int sort_id) const
  {
    const auto it = remap_.find(sort_id);
    return it != remap_.end() ? it->second : sort_id;
  }

  void reset()
  {
    remap_.clear();
    last_world_.clear();
    graveyard_.clear();
  }

private:
  struct Grave
  {
    int id;
    WorldObs obs;
    double death_s;
  };

  void refreshWorld(int stable, int sid,
                    const std::map<int, WorldObs> & world_by_sort)
  {
    const auto w = world_by_sort.find(sid);
    if (w != world_by_sort.end()) last_world_[stable] = w->second;
  }

  void purgeGraves(double now_s)
  {
    while (!graveyard_.empty() &&
           now_s - graveyard_.front().death_s > params_.ttl)
      graveyard_.pop_front();
  }

  /// 墓园匹配: 同类别且世界距离 < radius; 返回旧 id (无匹配 -1)
  int matchGrave(const WorldObs & obs, double)
  {
    for (const auto & g : graveyard_) {
      if (g.obs.cls != obs.cls) continue;
      const double dx = g.obs.x - obs.x;
      const double dy = g.obs.y - obs.y;
      const double dz = g.obs.z - obs.z;
      if (dx * dx + dy * dy + dz * dz <
          params_.reacquire_radius * params_.reacquire_radius)
        return g.id;
    }
    return -1;
  }

  void removeGrave(int id)
  {
    for (auto it = graveyard_.begin(); it != graveyard_.end(); ++it) {
      if (it->id == id) {
        graveyard_.erase(it);
        return;
      }
    }
  }

  Params params_;
  std::map<int, int> remap_;              // SORT id → 稳定 id (存活轨迹)
  std::map<int, WorldObs> last_world_;    // 稳定 id → 最近世界观测
  std::deque<Grave> graveyard_;           // 消失 id 墓园
};

}  // namespace easyfly::track_id_fusion

#endif  // EASYFLY_TRACK_ID_FUSION_ID_CONSOLIDATOR_HPP_
