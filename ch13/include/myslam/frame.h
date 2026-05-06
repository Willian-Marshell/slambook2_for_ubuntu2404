/**
 * @file frame.h
 * @brief 帧结构体
 *
 * 表示双目视觉里程计中的一帧，包含左右目图像、位姿、特征点等信息。
 * @note 调用位置:
 * - dataset.cpp 的 NextFrame() 创建 Frame
 * - frontend.cpp 的 AddFrame(), StereoInit(), Track() 等多处调用
 * - map.cpp 的 InsertKeyFrame() 调用
 * - viewer.cpp 的 AddCurrentFrame(), DrawFrame() 调用
 */

#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {

// forward declare
struct MapPoint;
struct Feature;

/**
 * @brief 帧结构体
 *
 * 表示双目视觉里程计中的一帧数据，包含：
 * - 左右目图像
 * - 位姿（Tcw）
 * - 提取的特征点
 * - 时间戳和ID信息
 */
struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // id of this frame 当前帧ID
    unsigned long keyframe_id_ = 0;  // id of key frame 关键帧ID
    bool is_keyframe_ = false;       // 是否为关键帧
    double time_stamp_;              // 时间戳，暂不使用
    SE3 pose_;                       // Tcw 形式Pose 位姿
    std::mutex pose_mutex_;          // Pose数据锁
    cv::Mat left_img_, right_img_;   // stereo images 双目图像

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

   public:  // data members
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    /**
     * @brief 获取位姿（线程安全）
     * @return SE3 位姿
     * @note 调用位置: viewer.cpp 的 DrawFrame(), FollowCurrentFrame() 调用
     */
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    /**
     * @brief 设置位姿（线程安全）
     * @param pose 新的位姿
     */
    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// 设置关键帧并分配并键帧id
    /**
     * @brief 设置为关键帧并分配关键帧ID
     * @note 调用位置: frontend.cpp 的 InsertKeyframe() 调用
     */
    void SetKeyFrame();

    /// 工厂构建模式，分配id
    /**
     * @brief 工厂方法创建新帧，自动分配唯一ID
     * @return 新创建的帧指针
     * @note 调用位置: dataset.cpp 的 NextFrame() 调用
     */
    static std::shared_ptr<Frame> CreateFrame();
};

}  // namespace myslam

#endif  // MYSLAM_FRAME_H
