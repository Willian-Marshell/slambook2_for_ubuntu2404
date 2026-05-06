/**
 * @file frontend.cpp
 * @brief Frontend类实现
 *
 * 视觉里程计前端，实现特征提取、跟踪、位姿估计、关键帧判断等功能。
 * @note 定义见 frontend.h
 */

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

/**
 * @brief 构造函数
 *
 * 初始化GFTT特征检测器，从配置读取特征点数量参数。
 * @note 调用位置: visual_odometry.cpp 的 Init() 创建Frontend时调用
 */
Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

/**
 * @brief 添加帧并处理（外部接口）
 *
 * 根据当前状态机状态调用相应处理函数：
 * - INITING: 调用 StereoInit() 进行双目初始化
 * - TRACKING_GOOD/BAD: 调用 Track() 进行跟踪
 * - LOST: 调用 Reset() 重置
 *
 * @param frame 输入帧
 * @return true 处理成功
 * @note 调用位置: visual_odometry.cpp 的 Step() 调用
 */
bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

/**
 * @brief 正常跟踪模式
 *
 * 1. 根据上一帧位姿和相对运动设置当前帧初始位姿
 * 2. 调用 TrackLastFrame() 光流跟踪
 * 3. 调用 EstimateCurrentPose() PnP估计精化位姿
 * 4. 根据内点数量更新状态
 * 5. 判断是否需要插入关键帧
 *
 * @note 调用位置: AddFrame() 在 TRACKING_GOOD/BAD 状态时调用
 */
bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());    //用上一次计算的相对运动作为当前帧位姿初值，初始情况下为单位变换（第一帧）
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {               //跟踪到的内点数量足够多时，为跟踪良好
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {    //跟踪到的内点数量较少但仍然足够时，为跟踪较差
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {                                                        //跟踪到的内点数量很少时，为丢失
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

/**
 * @brief 设置当前帧为关键帧并插入地图
 *
 * 判断跟踪内点是否足够少（<阈值）来决定是否插入关键帧。
 * 插入关键帧后执行：
 * 1. 设置关键帧观测
 * 2. 检测新特征点
 * 3. 右图特征匹配
 * 4. 三角化新地图点
 * 5. 触发后端优化
 * 6. 更新可视化
 *
 * @return true 如果插入了关键帧
 * @note 调用位置: Track() 调用
 */
bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {       //与上一帧关键点关联的数量足够多时，不插入关键帧
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features 检测新特征

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

/**
 * @brief 为关键帧设置地图点观测
 *
 * 将左图所有特征点关联的地图点添加对该关键帧的观测。
 * @note 调用位置: InsertKeyframe() 调用
 */
void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

/**
 * @brief 三角化新点
 *
 * 对左图未关联地图点且有右图匹配的特征点进行三角化，
 * 创建新的地图点并插入地图。
 *
 * @return 三角化成功的点数量
 * @note 调用位置: InsertKeyframe() 调用
 */
int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.expired() &&  //这里expired()是因为之前的地图点可能被后端优化剔除掉了，所以需要检查是否过期
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

/**
 * @brief 估计当前帧位姿（PnP）
 *
 * 使用g2o构建仅位姿优化的图：
 * - VertexPose: 当前帧位姿顶点
 * - EdgeProjectionPoseOnly: 重投影误差边（已知的3D点投影到2D）
 *
 * 通过迭代优化调整位姿，剔除outlier。
 *
 * @return 内点数量（有效匹配）
 * @note 调用位置: Track() 调用
 */
int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

/**
 * @brief 跟踪上一帧（光流）
 *
 * 使用Lucas-Kanade光流跟踪上一帧特征点到当前帧：
 * - 有地图点的特征：用地图点世界坐标投影到当前帧估计位置作为光流初值
 * - 无地图点的特征：在上一帧位置附近搜索
 *
 * @return 跟踪成功的特征点数量
 * @note 调用位置: Track() 调用
 */
int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

/**
 * @brief 双目初始化
 *
 * 1. 检测左图特征点
 * 2. 在右图查找对应特征点
 * 3. 构建初始地图
 *
 * @return true 初始化成功
 * @note 调用位置: AddFrame() 在 INITING 状态时调用
 */
bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

/**
 * @brief 检测左图特征点（GFTT）
 *
 * 使用GFTT检测器在左图检测特征点，使用mask避免在已有特征点附近检测。
 *
 * @return 检测到的特征点数量
 * @note 调用位置: StereoInit(), InsertKeyframe() 调用
 */
int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

/**
 * @brief 在右图中查找对应特征点
 *
 * 使用光流法在右图中查找左图特征点的对应点：
 * - 有地图点的特征：用地图点投影估计右图位置作为初值
 * - 无地图点的特征：在左图位置附近搜索
 *
 * @return 找到对应点的数量
 * @note 调用位置: StereoInit(), InsertKeyframe() 调用
 */
int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

/**
 * @brief 构建初始地图（双目三角化）
 *
 * 对所有有左右匹配的特征点进行三角化，创建初始地图点。
 * 将第一帧设为关键帧，插入地图，触发后端优化。
 *
 * @return true 总是返回true（简化处理）
 * @note 调用位置: StereoInit() 调用
 */
bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

/**
 * @brief 跟踪失败时重置（未实现）
 *
 * @return true
 * @note 当前为占位实现
 */
bool Frontend::Reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

}  // namespace myslam