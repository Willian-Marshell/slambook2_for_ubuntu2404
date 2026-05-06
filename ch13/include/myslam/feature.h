/**
 * @file feature.h
 * @brief 2D特征点结构体
 *
 * 表示图像中提取的特征点，包含位置信息和关联的地图点。
 * @note 调用位置:
 * - frontend.cpp 的 DetectFeatures(), FindFeaturesInRight(), SetObservationsForKeyFrame() 调用
 * - mappoint.h 的 observations_ 关联 Feature
 * - map.cpp 的 CleanMap() 调用
 */

//
// Created by gaoxiang on 19-5-2.
//
#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.h"

namespace myslam {

struct Frame;
struct MapPoint;

/**
 * @brief 2D特征点
 *
 * 在图像中提取的特征点，三角化后关联地图点。
 * 通过 weak_ptr 避免循环引用。
 */
struct Feature {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;         // 持有该feature的frame
    cv::KeyPoint position_;              // 2D提取位置
    std::weak_ptr<MapPoint> map_point_;  // 关联地图点

    bool is_outlier_ = false;       // 是否为异常点
    bool is_on_left_image_ = true;  // 标识是否提在左图，false为右图

   public:
    Feature() {}

    /**
     * @brief 构造函数
     * @param frame 所属帧
     * @param kp OpenCV特征点
     */
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
}  // namespace myslam

#endif  // MYSLAM_FEATURE_H
