/**
 * @file viewer.h
 * @brief 可视化类
 *
 * 使用 Pangolin 库进行3D可视化，显示关键帧轨迹和地图点。
 * 在独立线程中运行。
 * @note 调用位置:
 * - visual_odometry.cpp 的 Init() 调用 SetMap()
 * - frontend.cpp 的 AddFrame() 调用 AddCurrentFrame(), UpdateMap()
 */

//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

/**
 * @brief 可视化类
 *
 * 使用 Pangolin 库实现3D轨迹和地图点可视化。
 * 在独立线程中运行渲染循环。
 */
class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    /**
     * @brief 构造函数，启动渲染线程
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    Viewer();

    /**
     * @brief 设置地图
     * @param map 地图指针
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    void SetMap(Map::Ptr map) { map_ = map; }

    /**
     * @brief 关闭可视化窗口
     * @note 调用位置: visual_odometry.cpp 的 Run() 循环退出时调用
     */
    void Close();

    /**
     * @brief 添加当前帧用于显示
     * @param current_frame 当前帧
     * @note 调用位置: frontend.cpp 的 AddFrame() 调用
     */
    // 增加一个当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    /**
     * @brief 更新地图显示
     * @note 调用位置: frontend.cpp 的 InsertKeyframe() 调用
     */
    // 更新地图
    void UpdateMap();

   private:
    /**
     * @brief 渲染循环
     *
     * 在独立线程中持续渲染3D视图。
     */
    void ThreadLoop();

    /**
     * @brief 绘制帧（相机位姿）
     * @param frame 要绘制的帧
     * @param color 颜色
     */
    void DrawFrame(Frame::Ptr frame, const float* color);

    /**
     * @brief 绘制地图点
     */
    void DrawMapPoints();

    /**
     * @brief 跟随当前帧视角
     * @param vis_camera Pangolin相机状态
     */
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    /**
     * @brief 绘制当前帧图像特征点
     * @return 带有特征点标注的图像
     */
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}  // namespace myslam

#endif  // MYSLAM_VIEWER_H