/**
 * @file mappoint.h
 * @brief 地图点结构体
 *
 * 表示世界坐标系下的三维路标点，由特征点三角化产生。
 * @note 调用位置:
 * - frontend.cpp 的 BuildInitMap(), TriangulateNewPoints() 创建 MapPoint
 * - map.cpp 的 InsertMapPoint() 插入地图点
 * - backend.cpp 的 Optimize() 使用地图点
 * - viewer.cpp 的 DrawMapPoints() 绘制地图点
 */

#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

struct Frame;

struct Feature;

/**
 * @brief 路标点（地图点）
 *
 * 三维世界坐标点，由特征点三角化产生。
 * 维护被哪些特征点观测到的信息。
 */
struct MapPoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;  // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();  // Position in world 世界坐标位置
    std::mutex data_mutex_;
    int observed_times_ = 0;  // being observed by feature matching algo. 被观测次数
    std::list<std::weak_ptr<Feature>> observations_;  ///< 观测到该地图点的特征点列表

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    /**
     * @brief 获取位置（线程安全）
     * @return 世界坐标
     * @note 调用位置: viewer.cpp 的 DrawMapPoints() 调用
     */
    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    /**
     * @brief 设置位置（线程安全）
     * @param pos 新的位置
     */
    void SetPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    };

    /**
     * @brief 添加观测
     * @param feature 观测到该地图点的特征点
     */
    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    /**
     * @brief 移除观测
     * @param feat 要移除的特征点
     * @note 调用位置: map.cpp 的 CleanMap() 调用
     */
    void RemoveObservation(std::shared_ptr<Feature> feat);

    /**
     * @brief 获取所有观测（线程安全）
     * @return 特征点列表
     */
    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // factory function
    /**
     * @brief 工厂方法创建新地图点
     * @return 新创建的地图点指针
     * @note 调用位置: frontend.cpp 的 BuildInitMap(), TriangulateNewPoints() 调用
     */
    static MapPoint::Ptr CreateNewMappoint();
};
}  // namespace myslam

#endif  // MYSLAM_MAPPOINT_H
