/**
 * @file viewer.cpp
 * @brief Viewer类实现
 *
 * 使用Pangolin库进行3D可视化，在独立线程中运行。
 * @note 定义见 viewer.h
 */

#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

/**
 * @brief 构造函数，启动渲染线程
 * @note 调用位置: visual_odometry.cpp 的 Init() 创建Viewer时调用
 */
Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

/**
 * @brief 关闭可视化窗口
 * @note 调用位置: visual_odometry.cpp 的 Run() 循环退出时调用
 */
void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

/**
 * @brief 添加当前帧用于显示
 * @param current_frame 当前帧
 * @note 调用位置: frontend.cpp 的 Track(), StereoInit() 调用
 */
void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

/**
 * @brief 更新地图显示
 *
 * 从地图获取活跃关键帧和地图点用于渲染。
 * @note 调用位置: frontend.cpp 的 InsertKeyframe(), StereoInit() 调用
 */
void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

/**
 * @brief 渲染循环
 *
 * 在独立线程中持续渲染Pangolin窗口，显示：
 * - 当前帧相机位姿（绿色）
 * - 关键帧轨迹（红色）
 * - 地图点（红色）
 * - 当前帧图像（带特征点标注）
 */
void Viewer::ThreadLoop() {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            cv::Mat img = PlotFrameImage();
            cv::imshow("image", img);
            cv::waitKey(1);
        }

        if (map_) {
            DrawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }

    LOG(INFO) << "Stop viewer";
}

/**
 * @brief 绘制当前帧图像特征点
 *
 * 在左图上绘制有地图点关联的特征点（绿色圆点）。
 * @return 带有特征点标注的图像
 */
cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.lock()) {
            auto feat = current_frame_->features_left_[i];
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return img_out;
}

/**
 * @brief 跟随当前帧视角
 *
 * 将相机视角对准当前帧位置。
 * @param vis_camera Pangolin相机状态
 */
void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

/**
 * @brief 绘制帧（相机位姿）
 *
 * 使用OpenGL绘制相机锥体（8条边组成的金字塔）。
 * @param frame 要绘制的帧
 * @param color RGB颜色数组
 */
void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
    SE3 Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

/**
 * @brief 绘制地图点
 *
 * 使用OpenGL绘制所有活跃关键帧（红色相机锥体）
 * 和所有活跃地图点（红色点云）。
 */
void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : active_keyframes_) {
        DrawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

}  // namespace myslam