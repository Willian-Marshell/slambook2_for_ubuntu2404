/**
 * @file backend.h
 * @brief 后端优化类
 *
 * 在独立线程中运行BA优化，维护地图结构。
 * @note 调用位置:
 * - visual_odometry.cpp 的 Init() 调用 SetMap()
 * - frontend.cpp 的 InsertKeyframe() 调用 UpdateMap()
 */

//
// Created by gaoxiang on 19-5-2.
//

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
class Map;

/**
 * @brief 后端优化类
 *
 * 在独立线程中运行Bundle Adjustment优化。
 * 当前端插入新关键帧时触发优化，更新地图点位置。
 */
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /**
     * @brief 构造函数，启动优化线程
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    /// 构造函数中启动优化线程并挂起
    Backend();

    // 设置左右目的相机，用于获得内外参
    /**
     * @brief 设置相机内外参
     * @param left 左相机
     * @param right 右相机
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        cam_left_ = left;
        cam_right_ = right;
    }

    /**
     * @brief 设置地图
     * @param map 地图指针
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    /// 设置地图
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /**
     * @brief 触发地图更新，启动优化
     * @note 调用位置: frontend.cpp 的 InsertKeyframe() 调用
     */
    /// 触发地图更新，启动优化
    void UpdateMap();

    /**
     * @brief 关闭后端线程
     * @note 调用位置: visual_odometry.cpp 的 Run() 循环退出时调用
     */
    /// 关闭后端线程
    void Stop();

   private:
    /**
     * @brief 后端优化循环
     *
     * 等待 map_update_ 信号，对活跃关键帧和地图点进行BA优化。
     */
    /// 后端线程
    void BackendLoop();

    /**
     * @brief 执行BA优化
     * @param keyframes 关键帧
     * @param landmarks 地图点
     * @note 使用 g2o_types.h 中定义的顶点，边进行优化
     */
    /// 对给定关键帧和路标点进行优化
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H