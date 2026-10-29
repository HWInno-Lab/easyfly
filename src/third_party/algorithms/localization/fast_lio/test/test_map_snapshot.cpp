// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>
#include <thread>
#include <tuple>
#include "ikd-Tree/ikd_Tree.h"

namespace {
using Tree = KD_TREE<pcl::PointXYZINormal>;
using Points = Tree::PointVector;
using Keys = std::set<std::tuple<float, float, float, float>>;

Points grid(int first_x, int last_x, int rows)
{
    Points points;
    for (int x = first_x; x < last_x; ++x)
        for (int y = 0; y < rows; ++y)
        {
            pcl::PointXYZINormal point{};
            point.x = x + 0.25f;
            point.y = y + 0.25f;
            point.z = 0.25f;
            point.intensity = static_cast<float>(x * 1000 + y);
            points.push_back(point);
        }
    return points;
}

Keys keys(const Points &points)
{
    Keys result;
    for (const auto &point : points)
        result.emplace(point.x, point.y, point.z, point.intensity);
    return result;
}

bool snapshot(Tree &tree, Points &points)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do
    {
        if (tree.Snapshot(points))
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

TEST(MapSnapshot, EmptyMapReplacesPreviousOutput)
{
    // The native tree has a large fixed rebuild journal; never put it on the stack.
    auto tree = std::make_unique<Tree>();
    auto output = grid(0, 1, 1);
    ASSERT_TRUE(snapshot(*tree, output));
    EXPECT_TRUE(output.empty());
}

TEST(MapSnapshot, RepeatedPublicationKeepsOnlyCurrentMapPoints)
{
    auto tree = std::make_unique<Tree>();
    const auto input = grid(0, 40, 50);
    tree->Build(input);
    const auto expected = keys(input);
    Points output;
    for (int publication = 0; publication < 100; ++publication)
    {
        ASSERT_TRUE(snapshot(*tree, output));
        EXPECT_EQ(output.size(), expected.size());
        EXPECT_EQ(keys(output), expected);
    }
}

TEST(MapSnapshot, ReflectsDeletesAndInsertionsDuringNativeRebuilding)
{
    auto tree = std::make_unique<Tree>(0.2f, 0.6f, 0.5f);
    tree->Build(grid(0, 80, 100));
    // Exceed the native asynchronous-rebuild threshold and delete half the map.
    // Subsequent snapshots must not retain any removed historical scan points.
    BoxPointType box{};
    box.vertex_min[0] = box.vertex_min[1] = box.vertex_min[2] = -1.f;
    box.vertex_max[0] = 40.f;
    box.vertex_max[1] = 101.f;
    box.vertex_max[2] = 1.f;
    std::vector<BoxPointType> boxes{box};
    EXPECT_EQ(tree->Delete_Point_Boxes(boxes), 4000);
    auto additional = grid(90, 96, 100);
    auto expected = keys(grid(40, 80, 100));
    const auto added_keys = keys(additional);
    expected.insert(added_keys.begin(), added_keys.end());
    Points output;
    for (int scan = 0; scan < 40; ++scan)
    {
        tree->Add_Points(additional, true);
        ASSERT_TRUE(snapshot(*tree, output));
        EXPECT_EQ(output.size(), expected.size());
        EXPECT_EQ(keys(output), expected);
    }
}
}  // namespace
