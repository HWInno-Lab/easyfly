// IdConsolidator 单元测试 (Q19): 世界系 id 重关联
#include <gtest/gtest.h>

#include "easyfly_track_id_fusion/id_consolidator.hpp"

using easyfly::track_id_fusion::IdConsolidator;
using easyfly::track_id_fusion::WorldObs;
using Vec = std::vector<int>;
using Map = std::map<int, WorldObs>;

static WorldObs obs(double x, double y, const std::string & cls = "person")
{
  return {x, y, 0.0, cls};
}

TEST(IdConsolidator, NewIdWithoutGraveKeepsIdentity)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  EXPECT_EQ(c.stableId(7), 7);
}

TEST(IdConsolidator, ReappearsNearbyInheritsOldId)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);   // id 7 存活
  c.update({}, {}, 1.0);                  // 消失 → 入墓园
  c.update({12}, {{12, obs(8.4, 2.1)}}, 2.0);  // 新 id 12 在 0.4m 外重现
  EXPECT_EQ(c.stableId(12), 7);           // 继承旧 id
}

TEST(IdConsolidator, TooFarDoesNotMerge)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(14.0, 2.0)}}, 2.0);  // 6m 外 (另一目标)
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, ClassMismatchDoesNotMerge)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2, "person")}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(8.1, 2.0, "car")}}, 2.0);  // 同位置不同类
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, TtlExpiryDoesNotMerge)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(8.1, 2.0)}}, 20.0);  // 19s 后 (> ttl 10s)
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, MovingTargetWithinRadiusMerges)
{
  IdConsolidator c;  // 1m/s 移动, 3s 后在 3m 外重现 (radius 3.0 边缘内)
  c.update({7}, {{7, obs(14, -2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(16.5, -2)}}, 4.0);  // 2.5m 位移
  EXPECT_EQ(c.stableId(12), 7);
}

TEST(IdConsolidator, RemapStableAcrossFrames)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(8.2, 2.0)}}, 2.0);   // 12 → 7
  c.update({12}, {{12, obs(8.3, 2.0)}}, 3.0);   // 12 连续帧
  c.update({12}, {}, 4.0);                       // 观测缺失 (深度无效)
  EXPECT_EQ(c.stableId(12), 7);                  // remap 保持
}

TEST(IdConsolidator, DisabledPassesThrough)
{
  IdConsolidator::Params p;
  p.enabled = false;
  IdConsolidator c(p);
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(8.1, 2.0)}}, 2.0);
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, NoWorldObsForNewIdKeepsIdentity)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {}, 2.0);  // 新 id 无深度/无世界观测 → 无法关联, 原样
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, RevivedGraveNotMatchedTwice)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {{12, obs(8.1, 2.0)}}, 2.0);  // 12 → 7 (墓园移除)
  c.update({13}, {{13, obs(8.2, 2.0)}}, 3.0);  // 另一新 id 同位置: 7 已复活无墓 → 不合并
  EXPECT_EQ(c.stableId(13), 13);
}

TEST(IdConsolidator, FirstDelayedWorldObservationCanInheritGrave)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {}, 2.0);
  EXPECT_EQ(c.stableId(12), 12);
  c.update({12}, {{12, obs(8.1, 2)}}, 2.1);
  EXPECT_EQ(c.stableId(12), 7);
  c.update({12, 13}, {{13, obs(8.1, 2)}}, 2.2);
  EXPECT_EQ(c.stableId(13), 13);
}

TEST(IdConsolidator, ObservedIdentityDoesNotChangeWhenGraveAppearsLater)
{
  IdConsolidator c;
  c.update({7, 12}, {{7, obs(8, 2)}, {12, obs(8.1, 2)}}, 0.0);
  c.update({12}, {{12, obs(8.1, 2)}}, 1.0);
  c.update({12}, {{12, obs(8.1, 2)}}, 2.0);
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, ResetClearsMappingsAndGraves)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.reset();
  c.update({12}, {{12, obs(8.1, 2)}}, 2.0);
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, DelayedWorldAfterTtlDoesNotInherit)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2)}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {}, 2.0);
  c.update({12}, {{12, obs(8.1, 2)}}, 12.0);
  EXPECT_EQ(c.stableId(12), 12);
}

TEST(IdConsolidator, DelayedDifferentClassDoesNotInherit)
{
  IdConsolidator c;
  c.update({7}, {{7, obs(8, 2, "person")}}, 0.0);
  c.update({}, {}, 1.0);
  c.update({12}, {}, 2.0);
  c.update({12}, {{12, obs(8.1, 2, "car")}}, 2.1);
  EXPECT_EQ(c.stableId(12), 12);
}
