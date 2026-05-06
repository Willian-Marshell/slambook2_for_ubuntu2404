/**
 * @file camera.cpp
 * @brief Camera类实现
 *
 * 实现相机坐标系转换功能。
 * @note 定义见 camera.h
 */

#include "myslam/camera.h"

namespace myslam {

Camera::Camera() {
}

/**
 * @brief 世界坐标转相机坐标
 * @param p_w 世界坐标点
 * @param T_c_w 相机到世界的变换（Twc）
 * @return 相机坐标系下的三维点
 */
Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) {
    return pose_ * T_c_w * p_w;
}

/**
 * @brief 相机坐标转世界坐标
 * @param p_c 相机坐标点
 * @param T_c_w 相机到世界的变换（Twc）
 * @return 世界坐标系下的三维点
 */
Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
}

/**
 * @brief 相机坐标转像素坐标
 * @param p_c 相机坐标系下的三维点
 * @return 像素坐标 (u, v)
 */
Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    return Vec2(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

/**
 * @brief 像素坐标转相机坐标（需要深度）
 * @param p_p 像素坐标
 * @param depth 深度值
 * @return 相机坐标系下的三维点
 */
Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) {
    return Vec3(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth
    );
}

/**
 * @brief 世界坐标转像素坐标
 * @param p_w 世界坐标点
 * @param T_c_w 相机到世界的变换（Twc）
 * @return 像素坐标 (u, v)
 */
Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

/**
 * @brief 像素坐标转世界坐标（需要深度）
 * @param p_p 像素坐标
 * @param T_c_w 相机到世界的变换（Twc）
 * @param depth 深度值
 * @return 世界坐标系下的三维点
 */
Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

}