/**
 * @file config.cpp
 * @brief Config类实现
 *
 * @note 定义见 config.h
 */

#include "myslam/config.h"

namespace myslam {

/**
 * @brief 设置配置文件并加载
 * @param filename 配置文件路径
 * @return true 加载成功
 * @note 调用位置: visual_odometry.cpp 的 Init() 调用
 */
bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release();
        return false;
    }
    return true;
}

/**
 * @brief 析构函数，释放文件
 */
Config::~Config() {
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}