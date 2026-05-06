/**
 * @file visual_odometry.cpp
 * @brief VisualOdometry类实现
 *
 * 整合前端、后端、地图、可视化模块，是VO系统的对外入口。
 * @note 定义见 visual_odometry.h
 */

#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam {

/**
 * @brief 构造函数
 * @param config_path 配置文件路径
 * @note 调用位置: app/run_kitti_stereo.cpp 调用
 */
VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

/**
 * @brief 初始化系统
 *
 * 1. 设置配置文件
 * 2. 初始化数据集
 * 3. 创建前端、后端、地图、可视化组件
 * 4. 建立组件间关联
 *
 * @return true 初始化成功
 * @note 调用位置: app/run_kitti_stereo.cpp 调用
 */
bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

/**
 * @brief 运行VO系统主循环
 *
 * 持续调用 Step() 处理每一帧，直到数据集结束。
 * 退出时关闭后端和可视化。
 * @note 调用位置: app/run_kitti_stereo.cpp 调用
 */
void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

/**
 * @brief 单步执行
 *
 * 从数据集读取一帧，交给前端处理。
 * @return true 成功处理一帧，false 表示数据集结束
 * @note 调用位置: Run() 循环中调用
 */
bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

}  // namespace myslam