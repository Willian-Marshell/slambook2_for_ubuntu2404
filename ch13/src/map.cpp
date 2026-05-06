/**
 * @file map.cpp
 * @brief Map类实现
 *
 * 管理关键帧和地图点的插入、删除、查询。
 * @note 定义见 map.h
 */

#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {

/**
 * @brief 插入关键帧
 *
 * 将关键帧插入到关键帧列表，如果活跃关键帧数量超过阈值，
 * 调用 RemoveOldKeyframe() 移除旧的关键帧。
 * @param frame 关键帧
 * @note 调用位置: frontend.cpp 的 InsertKeyframe(), BuildInitMap() 调用
 */
void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframes_) {
        RemoveOldKeyframe();
    }
}

/**
 * @brief 插入地图点
 * @param map_point 地图点
 * @note 调用位置: frontend.cpp 的 BuildInitMap(), TriangulateNewPoints() 调用
 */
void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    if (landmarks_.find(map_point->id_) == landmarks_.end()) {
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

/**
 * @brief 移除旧的关键帧
 *
 * 计算当前帧与所有活跃关键帧的距离：
 * - 如果存在距离很近（<0.2）的关键帧，优先删除最近的
 * - 否则删除距离最远的关键帧
 * 同时移除该关键帧上所有特征点对该地图点的观测。
 * @note 调用位置: InsertKeyFrame() 当活跃关键帧超过数量时调用
 */
void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;
    // 寻找与当前帧最近与最远的两个关键帧
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;
        auto dis = (kf.second->Pose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;  // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // 删掉最远的
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feat : frame_to_remove->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }
    for (auto feat : frame_to_remove->features_right_) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    CleanMap();
}

/**
 * @brief 清理地图
 *
 * 移除活跃地图点中被观测次数为0的点。
 * @note 调用位置: RemoveOldKeyframe() 调用
 */
void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin();
         iter != active_landmarks_.end();) {
        if (iter->second->observed_times_ == 0) {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}

}  // namespace myslam