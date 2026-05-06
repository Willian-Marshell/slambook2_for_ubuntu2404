/**
 * @file frontend.h
 * @brief 前端类
 *
 * 视觉里程计前端，负责特征提取、跟踪、位姿估计和关键帧判断。
 * 状态机管理：INITING -> TRACKING_GOOD/BAD -> LOST。
 * @note 调用位置:
 * - visual_odometry.cpp 的 Init(), Step() 调用 AddFrame()
 * - backend.cpp 的 UpdateMap() 触发优化
 * - viewer.cpp 的 UpdateMap() 更新可视化
 */

#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

class Backend;
class Viewer;

/**
 * @brief 前端状态枚举
 *
 * - INITING: 初始化状态
 * - TRACKING_GOOD: 跟踪正常
 * - TRACKING_BAD: 跟踪较差
 * - LOST: 跟踪失败
 */
enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * @brief 视觉里程计前端
 *
 * 负责：
 * - 双目特征提取与匹配
 * - 相对位姿估计（光流跟踪 + PnP）
 * - 关键帧判断与三角化
 * - 触发后端优化
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    /**
     * @brief 添加帧并处理（外部接口）
     * @param frame 输入帧
     * @return true 处理成功
     * @note 调用位置: visual_odometry.cpp 的 Step() 调用
     */
    bool AddFrame(Frame::Ptr frame);

    /// Set函数
    /**
     * @brief 设置地图
     * @param map 地图指针
     */
    void SetMap(Map::Ptr map) { map_ = map; }

    /**
     * @brief 设置后端
     * @param backend 后端指针
     */
    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    /**
     * @brief 设置可视化器
     * @param viewer 可视化器指针
     */
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    /**
     * @brief 获取前端状态
     * @return FrontendStatus
     * @note 调用位置: visual_odometry.cpp 的 GetFrontendStatus() 调用
     */
    FrontendStatus GetStatus() const { return status_; }

    /**
     * @brief 设置相机
     * @param left 左相机
     * @param right 右相机
     */
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

   private:
    /**
     * @brief 正常跟踪模式
     * @return true 如果成功
     */
    bool Track();

    /**
     * @brief 跟踪失败时重置
     * @return true 如果成功
     */
    bool Reset();

    /**
     * @brief 跟踪上一帧（光流）
     * @return 跟踪到的特征点数量
     * @note 调用位置: Track() 调用
     */
    int TrackLastFrame();

    /**
     * @brief 估计当前帧位姿（PnP）
     * @return 内点数量
     * @note 调用位置: Track() 调用
     */
    int EstimateCurrentPose();

    /**
     * @brief 设置当前帧为关键帧并插入地图
     * @return true 如果成功
     * @note 调用位置: Track() 调用；调用 map_->InsertKeyFrame()
     */
    bool InsertKeyframe();

    /**
     * @brief 双目初始化
     * @return true 如果成功
     * @note 调用位置: AddFrame() 在 INITING 状态调用
     */
    bool StereoInit();

    /**
     * @brief 检测左图特征点（GFTT）
     * @return 检测到的特征点数量
     * @note 调用位置: StereoInit(), TrackLastFrame() 调用
     */
    int DetectFeatures();

    /**
     * @brief 在右图中查找对应特征点
     * @return 找到对应点的数量
     * @note 调用位置: StereoInit(), TrackLastFrame() 调用
     */
    int FindFeaturesInRight();

    /**
     * @brief 构建初始地图（双目三角化）
     * @return true 如果成功
     * @note 调用位置: StereoInit() 调用
     */
    bool BuildInitMap();

    /**
     * @brief 三角化新点
     * @return 三角化成功的点数量
     * @note 调用位置: InsertKeyframe() 调用
     */
    int TriangulateNewPoints();

    /**
     * @brief 为关键帧设置地图点观测
     * @note 调用位置: InsertKeyframe() 调用
     */
    void SetObservationsForKeyFrame();

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧
    Camera::Ptr camera_left_ = nullptr;   // 左侧相机
    Camera::Ptr camera_right_ = nullptr;  // 右侧相机

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};

}  // namespace myslam

#endif  // MYSLAM_FRONTEND_H