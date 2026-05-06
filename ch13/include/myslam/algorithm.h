/**
 * @file algorithm.h
 * @brief 视觉里程计中使用的核心算法
 *
 * 包含三角化、坐标转换等基础算法。
 * @note 调用位置:
 * - triangulation() 被 frontend.cpp 的 TriangulateNewPoints() 调用
 * - toVec2() 被 frontend.cpp 多处调用
 */

//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

namespace myslam {

/**
 * @brief 线性三角化（SVD方法）
 *
 * 根据多帧位姿和对应的归一化平面坐标，通过SVD分解求解三维点
 * @param poses    相机位姿列表（Tcw）
 * @param points   归一化平面上的2D点坐标
 * @param pt_world  输出的世界坐标系下三维点
 * @return true 如果求解成功（奇异性比值满足条件）
 * @note 调用位置: frontend.cpp 的 TriangulateNewPoints() 调用
 */
inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}

/**
 * @brief OpenCV Point2f 转 Eigen Vec2
 * @param p OpenCV 2D点
 * @return Eigen Vec2 类型
 * @note 调用位置: frontend.cpp 的 TrackLastFrame(), EstimateCurrentPose() 等多处调用
 */
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}  // namespace myslam

#endif  // MYSLAM_ALGORITHM_H
