/**
 * @file camera.h
 * @brief 针孔相机模型类
 *
 * 管理相机的内外参，提供世界坐标系、相机坐标系、像素坐标系之间的转换。
 * @note 调用位置:
 * - frontend.cpp 的 SetCameras(), Track(), StereoInit(), EstimateCurrentPose() 等多处调用
 * - backend.cpp 的 Optimize() 调用
 * - viewer.cpp 的 DrawFrame() 调用
 * - dataset.cpp 的 Init() 调用
 */

#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * @brief 针孔相机模型
 *
 * 管理双目相机的内参（fx, fy, cx, cy, baseline）和外参（pose），
 * 提供各坐标系间的转换功能。
 */
class Camera {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0;  // Camera intrinsics 内参
    SE3 pose_;             // extrinsic, from stereo camera to single camera 外参
    SE3 pose_inv_;         // inverse of extrinsics 外参的逆

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }

    // return intrinsic matrix
    /**
     * @brief 返回内参矩阵K
     * @return 3x3 内参矩阵
     * @note 调用位置: g2o_types.h 的 EdgeProjectionPoseOnly, EdgeProjection 调用
     */
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    // coordinate transform: world, camera, pixel
    /**
     * @brief 世界坐标转相机坐标
     * @param p_w 世界坐标点
     * @param T_c_w 相机到世界的变换
     * @return 相机坐标系下的三维点
     * @note 调用位置: frontend.cpp 的 Track(), EstimateCurrentPose() 调用
     */
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    /**
     * @brief 相机坐标转世界坐标
     * @param p_c 相机坐标点
     * @param T_c_w 相机到世界的变换
     * @return 世界坐标系下的三维点
     */
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    /**
     * @brief 相机坐标转像素坐标
     * @param p_c 相机坐标系下的三维点
     * @return 像素坐标 (u, v)
     */
    Vec2 camera2pixel(const Vec3 &p_c);

    /**
     * @brief 像素坐标转相机坐标（需要深度）
     * @param p_p 像素坐标
     * @param depth 深度值
     * @return 相机坐标系下的三维点
     */
    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    /**
     * @brief 像素坐标转世界坐标（需要深度）
     * @param p_p 像素坐标
     * @param T_c_w 相机到世界的变换
     * @param depth 深度值
     * @return 世界坐标系下的三维点
     */
    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    /**
     * @brief 世界坐标转像素坐标
     * @param p_w 世界坐标点
     * @param T_c_w 相机到世界的变换
     * @return 像素坐标 (u, v)
     * @note 调用位置: frontend.cpp 的 BuildInitMap() 调用
     */
    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
};

}  // namespace myslam
#endif  // MYSLAM_CAMERA_H
