/**
 * @file frame.cpp
 * @brief Frame结构体实现
 *
 * @note 定义见 frame.h
 */

#include "myslam/frame.h"

namespace myslam {

/**
 * @brief 构造函数
 */
Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

/**
 * @brief 工厂方法创建新帧，自动分配唯一ID
 * @return 新创建的帧指针
 * @note 调用位置: dataset.cpp 的 NextFrame() 调用
 */
Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

/**
 * @brief 设置为关键帧并分配关键帧ID
 * @note 调用位置: frontend.cpp 的 InsertKeyframe(), BuildInitMap() 调用
 */
void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

}