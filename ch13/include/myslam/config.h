/**
 * @file config.h
 * @brief 配置管理类
 *
 * 单例模式，通过配置文件（YAML）管理系统参数。
 * @note 调用位置:
 * - visual_odometry.cpp 的 VisualOdometry() 构造函数调用 SetParameterFile()
 * - dataset.cpp 的 Init() 调用 Get() 获取 dataset_dir
 * - camera.cpp 的 Camera() 调用 Get() 获取相机内外参
 */

#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * @brief 配置管理类（单例模式）
 *
 * 管理系统配置文件读取，提供模板化的参数获取接口。
 * 使用 SetParameterFile() 设置配置文件，Get<T>() 获取参数值。
 */
class Config {
   private:
    static std::shared_ptr<Config> config_;  ///< 单例实例
    cv::FileStorage file_;                    ///< OpenCV文件存储，用于读取YAML配置

    Config() {}  // private constructor makes a singleton
   public:
    ~Config();  // close the file when deconstructing

    /**
     * @brief 设置配置文件路径并加载
     * @param filename 配置文件路径（YAML格式）
     * @return true 加载成功
     * @note 调用位置: visual_odometry.cpp 的 VisualOdometry() 构造函数调用
     */
    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    /**
     * @brief 获取配置参数值
     * @tparam T 参数类型
     * @param key 参数名
     * @return T 类型的参数值
     * @note 调用位置: dataset.cpp, camera.cpp 等多处调用
     */
    template <typename T>
    static T Get(const std::string &key) {
        return T(Config::config_->file_[key]);
    }
};
}  // namespace myslam

#endif  // MYSLAM_CONFIG_H
