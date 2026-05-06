/**
 * @file map.h
 * @brief 地图管理类
 *
 * 管理关键帧和地图点的插入、删除、查询。采用滑动窗口机制，
 * 保持固定数量的活跃关键帧。
 * @note 调用位置:
 * - frontend.cpp 的 SetMap(), InsertKeyframe(), InsertMapPoint() 调用
 * - backend.cpp 的 UpdateMap(), Optimize() 调用
 * - viewer.cpp 的 SetMap(), UpdateMap(), DrawMapPoints() 调用
 * - visual_odometry.cpp 的 Init() 调用
 */

#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

/**
 * @brief 地图管理类
 *
 * 管理关键帧和地图点，维护活跃关键帧的滑动窗口。
 * 前端负责插入，后端负责优化和清理。
 */
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    /**
     * @brief 插入关键帧
     * @param frame 关键帧
     * @note 调用位置: frontend.cpp 的 InsertKeyframe() 调用
     */
    /// 增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);

    /**
     * @brief 插入地图点
     * @param map_point 地图点
     * @note 调用位置: frontend.cpp 的 BuildInitMap(), TriangulateNewPoints() 调用
     */
    /// 增加一个地图顶点
    void InsertMapPoint(MapPoint::Ptr map_point);

    /**
     * @brief 获取所有地图点
     * @return 地图点哈希表
     * @note 调用位置: backend.cpp 的 Optimize() 调用
     */
    /// 获取所有地图点
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    /**
     * @brief 获取所有关键帧
     * @return 关键帧哈希表
     */
    /// 获取所有关键帧
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /**
     * @brief 获取活跃地图点（用于优化）
     * @return 活跃地图点哈希表
     * @note 调用位置: backend.cpp 的 Optimize() 调用
     */
    /// 获取激活地图点
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /**
     * @brief 获取活跃关键帧
     * @return 活跃关键帧哈希表
     * @note 调用位置: backend.cpp 的 Optimize(), viewer.cpp 的 UpdateMap() 调用
     */
    /// 获取激活关键帧
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /**
     * @brief 清理地图
     *
     * 移除观测数量为零的无效地图点。
     * @note 调用位置: backend.cpp 的 BackendLoop() 调用
     */
    /// 清理map中观测数量为零的点
    void CleanMap();

   private:
    // 将旧的关键帧置为不活跃状态
    /**
     * @brief 移除旧的关键帧
     *
     * 当活跃关键帧数量超过 num_active_keyframes_ 时调用，
     * 将最旧的关键帧从活跃列表移除。
     */
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks 所有地图点
    LandmarksType active_landmarks_;  // active landmarks 活跃地图点
    KeyframesType keyframes_;         // all key-frames 所有关键帧
    KeyframesType active_keyframes_;  // all key-frames 活跃关键帧

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  // 激活的关键帧数量
};
}  // namespace myslam

#endif  // MAP_H
