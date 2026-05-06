/**
 * @file dataset.h
 * @brief KITTI数据集读取类
 *
 * 读取KITTI格式的双目图像和相机标定参数。
 * @note 调用位置:
 * - visual_odometry.cpp 的 Init() 创建 Dataset 实例
 * - visual_odometry.cpp 的 Step() 调用 NextFrame()
 */

#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

/**
 * @brief 数据集读取类
 *
 * 读取KITTI格式数据集，包含：
 * - 双目图像（left/right）
 * - 相机标定参数
 */
class Dataset {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;

    /**
     * @brief 构造函数
     * @param dataset_path 数据集根目录
     */
    Dataset(const std::string& dataset_path);

    /**
     * @brief 初始化，读取相机标定参数
     * @return true 初始化成功
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    /// 初始化，返回是否成功
    bool Init();

    /**
     * @brief 读取下一帧
     * @return Frame::Ptr 包含双目图像的帧
     * @note 调用位置: visual_odometry.cpp 的 Step() 调用
     */
    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /**
     * @brief 获取相机
     * @param camera_id 相机ID（0为左目，1为右目）
     * @return 相机指针
     * @note 调用位置: visual_odometry.cpp 的 Init() 调用
     */
    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

   private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;  ///< 相机列表（双目的左右相机）
};
}  // namespace myslam

#endif