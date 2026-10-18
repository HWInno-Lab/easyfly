// EasyFly M4: 匈牙利算法 (最小代价分配) + IoU 工具
// 经典 O(n³) 实现 ( potentials / KM 风格 ), 用于检测-轨迹关联。
#ifndef EASYFLY_SORT_HUNGARIAN_HPP_
#define EASYFLY_SORT_HUNGARIAN_HPP_

#include <algorithm>
#include <limits>
#include <vector>

namespace easyfly::sort
{

/// 轴对齐框 IoU
struct BBox {
    double x1{}, y1{}, x2{}, y2{};  // 像素坐标

    double iou(const BBox & o) const
    {
        const double xx1 = std::max(x1, o.x1), yy1 = std::max(y1, o.y1);
        const double xx2 = std::min(x2, o.x2), yy2 = std::min(y2, o.y2);
        const double w = std::max(0.0, xx2 - xx1), h = std::max(0.0, yy2 - yy1);
        const double inter = w * h;
        const double a1 = (x2 - x1) * (y2 - y1), a2 = (o.x2 - o.x1) * (o.y2 - o.y1);
        return inter / (a1 + a2 - inter + 1e-9);
    }
};

/// 求解最小代价分配。
/// cost: n×m 代价矩阵 (n 行=预测框, m 列=检测框)
/// 返回: 每个预测框配对的检测下标 (-1 = 未分配)
/// 拒绝代价 > max_cost 的配对。
inline std::vector<int> hungarianAssign(const std::vector<std::vector<double>> & cost,
                                        double max_cost)
{
    const int n = static_cast<int>(cost.size());
    const int m = n > 0 ? static_cast<int>(cost[0].size()) : 0;
    std::vector<int> result(n, -1);
    if (n == 0 || m == 0) return result;

    // 构造方阵 (不足处填 max_cost+1 使其不被选)
    const int sz = std::max(n, m);
    const double PAD = max_cost + 1.0;
    std::vector<std::vector<double>> a(sz, std::vector<double>(sz, PAD));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j) a[i][j] = cost[i][j];

    // Jonker-Volgenant 风格的短增广路实现 (对角势)
    std::vector<double> u(sz + 1, 0), v(sz + 1, 0);
    std::vector<int> p(sz + 1, 0), way(sz + 1, 0);   // p[j]=与列j配对的行 (1-based)

    for (int i = 1; i <= sz; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(sz + 1, std::numeric_limits<double>::infinity());
        std::vector<char> used(sz + 1, 0);

        do {
            used[j0] = 1;
            const int i0 = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            int j1 = -1;

            for (int j = 1; j <= sz; ++j) {
                if (used[j]) continue;
                const double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= sz; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else minv[j] -= delta;
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    for (int j = 1; j <= sz; ++j) {
        const int i = p[j] - 1;
        if (i < n && j - 1 < m && a[i][j - 1] <= max_cost) result[i] = j - 1;
    }
    return result;
}

}  // namespace easyfly::sort

#endif  // EASYFLY_SORT_HUNGARIAN_HPP_
