/**
 * @file visual_odometry.h
 * @brief 视觉里程计主类
 *
 * 整合前端、后端、地图、可视化模块，是VO系统的对外接口。
 * @note 调用位置:
 * - app/run_kitti_stereo.cpp 创建 VisualOdometry 实例并调用 Init(), Run()
 */

#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

namespace myslam {

/**
 * @brief 视觉里程计主类
 *
 * 整合前端(Frontend)、后端(Backend)、地图(Map)、可视化(Viewer)模块。
 * 负责系统初始化和数据流调度。
 */
class VisualOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /**
     * @brief 构造函数
     * @param config_path 配置文件路径
     * @note 调用位置: app/run_kitti_stereo.cpp 调用
     */
    /// constructor with config file
    VisualOdometry(std::string &config_path);

    /**
     * @brief 初始化系统
     * @return true 初始化成功
     * @note 调用位置: app/run_kitti_stereo.cpp 调用
     */
    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * @brief 运行VO系统
     *
     * 从数据集读取帧并处理，直到数据集结束。
     * @note 调用位置: app/run_kitti_stereo.cpp 调用
     */
    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * @brief 单步执行
     * @return true 成功处理一帧
     * @note 调用位置: Run() 循环中调用
     */
    /**
     * Make a step forward in dataset
     */
    bool Step();

    /**
     * @brief 获取前端状态
     * @return FrontendStatus
     * @note 调用位置: app/run_kitti_stereo.cpp 调用
     */
    /// 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

   private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;  ///< 前端
    Backend::Ptr backend_ = nullptr;   ///< 后端
    Map::Ptr map_ = nullptr;           ///< 地图
    Viewer::Ptr viewer_ = nullptr;     ///< 可视化

    // dataset
    Dataset::Ptr dataset_ = nullptr;   ///< 数据集
};
}  // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_H