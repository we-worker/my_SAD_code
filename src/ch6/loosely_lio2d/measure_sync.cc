//
// Created by xiang on 23-2-2.
//
#include "measure_sync.h"

namespace sad {
// MessageSync类的Sync方法
bool MessageSync::Sync() {
    // 如果激光雷达或IMU的缓冲区为空，则返回false
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    // 如果激光雷达数据还没有被推送
    if (!lidar_pushed_) {
        // 从激光雷达缓冲区的前端获取数据，并设置开始时间
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        // 计算激光雷达数据的结束时间
        lidar_end_time_ = measures_.lidar_begin_time_ + measures_.lidar_->scan_time;

        // 设置激光雷达数据的结束时间，并标记激光雷达数据已经被推送
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    // 如果最后一个IMU的时间戳小于激光雷达的结束时间，则返回false
    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    // 从IMU缓冲区的前端获取时间戳
    double imu_time = imu_buffer_.front()->timestamp_;
    measures_.imu_.clear();
    // 如果IMU缓冲区不为空，并且IMU的时间戳小于激光雷达的结束时间
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp_;
        // 如果IMU的时间戳大于激光雷达的结束时间，则跳出循环
        if (imu_time > lidar_end_time_) {
            break;
        }
        // 将IMU数据添加到measures_中，并从IMU缓冲区中移除
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    // 从激光雷达缓冲区和时间缓冲区中移除数据，并标记激光雷达数据未被推送
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // 如果设置了回调函数，则调用回调函数
    if (callback_) {
        callback_(measures_);
    }

    // 返回true表示同步成功
    return true;
}

}