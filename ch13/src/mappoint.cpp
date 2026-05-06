/**
 * @file mappoint.cpp
 * @brief MapPoint结构体实现
 *
 * @note 定义见 mappoint.h
 */

#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

/**
 * @brief 构造函数
 * @param id 地图点ID
 * @param position 世界坐标位置
 */
MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

/**
 * @brief 工厂方法创建新地图点
 * @return 新创建的地图点指针
 * @note 调用位置: frontend.cpp 的 BuildInitMap(), TriangulateNewPoints() 调用
 */
MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

/**
 * @brief 移除观测
 *
 * 从观测列表中移除指定特征点，同时减少观测计数。
 * @param feat 要移除的特征点
 * @note 调用位置: map.cpp 的 RemoveOldKeyframe() 调用
 */
void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        if (iter->lock() == feat) {
            observations_.erase(iter);
            feat->map_point_.reset();
            observed_times_--;
            break;
        }
    }
}

}  // namespace myslam