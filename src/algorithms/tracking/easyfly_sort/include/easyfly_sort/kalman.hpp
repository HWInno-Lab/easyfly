// EasyFly M4: SORT 卡尔曼滤波 (8 维状态, 匀速模型)
//
// 状态: [cx, cy, w, h, vcx, vcy, vw, vh]  (像素坐标)
// 观测: [cx, cy, w, h]
// 标准 SORT (Bewley et al. 2016) 的简化实现, 参数与原论文一致。
#ifndef EASYFLY_SORT_KALMAN_HPP_
#define EASYFLY_SORT_KALMAN_HPP_

#include <array>
#include <cmath>

namespace easyfly::sort
{

constexpr int KF_STATE_DIM = 8;   // 状态维
constexpr int KF_MEAS_DIM = 4;    // 观测维

class KalmanFilter
{
public:
    KalmanFilter()
    {
        // F: 状态转移 (dt=1 帧), F = [[I, I], [0, I]] (位置 += 速度, 速度保持)
        F_.fill(0.0);
        for (int i = 0; i < KF_STATE_DIM; ++i) F_[i * KF_STATE_DIM + i] = 1.0;  // 对角单位
        for (int i = 0; i < KF_MEAS_DIM; ++i)
            F_[i * KF_STATE_DIM + i + KF_MEAS_DIM] = 1.0;                       // 位置耦合速度
        // H: 观测矩阵 (取前 4 维)
        H_.fill(0.0);
        for (int i = 0; i < KF_MEAS_DIM; ++i) H_[i * KF_STATE_DIM + i] = 1.0;

        x_.fill(0.0);
        P_.fill(0.0);
    }

    /// 用首帧观测初始化状态
    void init(double cx, double cy, double w, double h)
    {
        x_ = {cx, cy, w, h, 0, 0, 0, 0};
        P_.fill(0.0);
        // 位置不确定度小 (观测到了), 速度不确定度大
        for (int i = 0; i < KF_MEAS_DIM; ++i) P_[i * KF_STATE_DIM + i] = 10.0;
        for (int i = KF_MEAS_DIM; i < KF_STATE_DIM; ++i)
            P_[i * KF_STATE_DIM + i] = 1000.0;
    }

    /// 预测一步 (x = F·x, P = F·P·Fᵀ + Q)
    void predict()
    {
        // x = F·x
        std::array<double, KF_STATE_DIM> x_new{};
        for (int r = 0; r < KF_STATE_DIM; ++r) {
            double s = 0;
            for (int c = 0; c < KF_STATE_DIM; ++c) s += F_[r * KF_STATE_DIM + c] * x_[c];
            x_new[r] = s;
        }
        x_ = x_new;

        // P = F·P·Fᵀ + Q
        std::array<double, KF_STATE_DIM * KF_STATE_DIM> FP{}, P_new{};
        for (int r = 0; r < KF_STATE_DIM; ++r)
            for (int c = 0; c < KF_STATE_DIM; ++c) {
                double s = 0;
                for (int k = 0; k < KF_STATE_DIM; ++k) s += F_[r * KF_STATE_DIM + k] * P_[k * KF_STATE_DIM + c];
                FP[r * KF_STATE_DIM + c] = s;
            }
        for (int r = 0; r < KF_STATE_DIM; ++r)
            for (int c = 0; c < KF_STATE_DIM; ++c) {
                double s = 0;
                for (int k = 0; k < KF_STATE_DIM; ++k) s += FP[r * KF_STATE_DIM + k] * F_[c * KF_STATE_DIM + k];
                P_new[r * KF_STATE_DIM + c] = s;
            }
        P_ = P_new;

        // Q: 过程噪声。
        // Q19 调优 (2026-09-13): 速度维 0.01→1.0 — 原论文假设固定帧率/温和运动,
        // 无人机平台检测帧间隔不规则(异步推理丢帧) + 目标像素快速移动,
        // 小 Q 让预测滞后 → IoU 崩 → ID 跳变。大 Q 让滤波器快速跟上速度变化。
        P_[4 * KF_STATE_DIM + 4] += 1.0;
        P_[5 * KF_STATE_DIM + 5] += 1.0;
        P_[6 * KF_STATE_DIM + 6] += 0.0001;
        P_[7 * KF_STATE_DIM + 7] += 0.0001;
        for (int i = 0; i < KF_MEAS_DIM; ++i) P_[i * KF_STATE_DIM + i] += 1.0;
    }

    /// 用观测更新 (K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹)
    void update(double cx, double cy, double w, double h)
    {
        const std::array<double, KF_MEAS_DIM> z{cx, cy, w, h};
        constexpr int N = KF_STATE_DIM, M = KF_MEAS_DIM;

        // S = H·P·Hᵀ + R (4x4)
        double S[M][M]{};
        for (int r = 0; r < M; ++r)
            for (int c = 0; c < M; ++c) {
                double s = 0;
                for (int k = 0; k < N; ++k) s += H_[r * N + k] * P_[k * N + c] * H_[c * N + k];
                S[r][c] = s;
            }
        for (int i = 0; i < M; ++i) S[i][i] += 1.0;  // R

        // S 求逆 (4x4, 高斯消元)
        double S_inv[M][M]{};
        invert4(S, S_inv);

        // K = P·Hᵀ·S⁻¹ (8x4)
        double K[N][M]{};
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < M; ++c) {
                double s = 0;
                for (int k = 0; k < M; ++k) {
                    double PH = 0;
                    for (int j = 0; j < N; ++j) PH += P_[r * N + j] * H_[k * N + j];
                    s += PH * S_inv[k][c];
                }
                K[r][c] = s;
            }

        // y = z - H·x
        double y[M];
        for (int r = 0; r < M; ++r) {
            double hx = 0;
            for (int k = 0; k < N; ++k) hx += H_[r * N + k] * x_[k];
            y[r] = z[r] - hx;
        }

        // x = x + K·y
        for (int r = 0; r < N; ++r) {
            double s = 0;
            for (int k = 0; k < M; ++k) s += K[r][k] * y[k];
            x_[r] += s;
        }

        // P = (I - K·H)·P  (先算 KH = K·H, 再 P -= KH·P)
        double KH[N][N]{};
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < N; ++c) {
                double s = 0;
                for (int k = 0; k < M; ++k) s += K[r][k] * H_[k * N + c];
                KH[r][c] = s;
            }
        std::array<double, N * N> P_new{};
        for (int r = 0; r < N; ++r)
            for (int c = 0; c < N; ++c) {
                double s = P_[r * N + c];
                for (int k = 0; k < N; ++k) s -= KH[r][k] * P_[k * N + c];
                P_new[r * N + c] = s;
            }
        P_ = P_new;
    }

    double cx() const { return x_[0]; }
    double cy() const { return x_[1]; }
    double w() const { return x_[2]; }
    double h() const { return x_[3]; }
    double vcx() const { return x_[4]; }
    double vcy() const { return x_[5]; }

private:
    static void invert4(double m[4][4], double inv[4][4])
    {
        // 增广矩阵高斯消元
        double a[4][8]{};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) a[i][j] = m[i][j];
            a[i][4 + i] = 1.0;
        }
        for (int col = 0; col < 4; ++col) {
            int piv = col;
            for (int r = col + 1; r < 4; ++r)
                if (std::abs(a[r][col]) > std::abs(a[piv][col])) piv = r;
            if (piv != col)
                for (int j = 0; j < 8; ++j) std::swap(a[col][j], a[piv][j]);
            const double d = a[col][col];
            for (int j = 0; j < 8; ++j) a[col][j] /= d;
            for (int r = 0; r < 4; ++r) {
                if (r == col) continue;
                const double f = a[r][col];
                for (int j = 0; j < 8; ++j) a[r][j] -= f * a[col][j];
            }
        }
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) inv[i][j] = a[i][4 + j];
    }

    std::array<double, KF_STATE_DIM> x_{};
    std::array<double, KF_STATE_DIM * KF_STATE_DIM> F_{};
    std::array<double, KF_MEAS_DIM * KF_STATE_DIM> H_{};
    std::array<double, KF_STATE_DIM * KF_STATE_DIM> P_{};
};

}  // namespace easyfly::sort

#endif  // EASYFLY_SORT_KALMAN_HPP_
