
#include <execution>

#include "common/lidar_utils.h"
#include "common/timer/timer.h"
#include "common/point_cloud_utils.h"
#include "loosely_lio.h"
#include "ch6/mapping_2d.h"

namespace sad {


Mapping2D mapping;


LooselyLIO::LooselyLIO() {
    sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) { ProcessMeasurements(m); });

    if (mapping.Init(false) == false) {
        return ;
    }
}

void LooselyLIO::ProcessMeasurements(const MeasureGroup &meas) {
    // LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_.size();
    measures_ = meas;

    // if (imu_need_init_) {
    //     // 初始化IMU系统
    //     TryInitIMU();
    //     return;
    // }

    // 利用IMU数据进行状态预测
    Predict();

    // 对点云去畸变
    // Undistort();

    // 配准
    Align(meas);
}

void LooselyLIO::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(eskf_.GetNominalState());

    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) {
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
    }
}


// void LooselyLIO::Undistort() {
//     auto cloud = measures_.lidar_;
//     auto imu_state = eskf_.GetNominalState();  // 最后时刻的状态
//     SE3 T_end = SE3(imu_state.R_, imu_state.p_);

//     if (options_.save_motion_undistortion_pcd_) {
//         sad::SaveCloudToFile("./data/ch7/before_undist.pcd", *cloud);
//     }

//     /// 将所有点转到最后时刻状态上
//     std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto &pt) {
//         SE3 Ti = T_end;
//         NavStated match;

//         // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
//         math::PoseInterp<NavStated>(
//             measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_, [](const NavStated &s) { return s.timestamp_; },
//             [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

//         Vec3d pi = ToVec3d(pt);
//         Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

//         pt.x = p_compensate(0);
//         pt.y = p_compensate(1);
//         pt.z = p_compensate(2);
//     });
//     scan_undistort_ = cloud;

//     if (options_.save_motion_undistortion_pcd_) {
//         sad::SaveCloudToFile("./data/ch7/after_undist.pcd", *cloud);
//     }
// }
SE2 SE3toSE2(const SE3& pose) {
    // 提取SE3位姿的旋转和平移部分
    Eigen::Matrix3d rotation = pose.rotationMatrix();
    Eigen::Vector3d translation = pose.translation();

    // 创建一个3x3的SE2矩阵
    Eigen::Matrix3d se2_matrix;
    // 将SE3的旋转矩阵的左上角2x2部分复制到SE2矩阵
    se2_matrix.block<2, 2>(0, 0) = rotation.block<2, 2>(0, 0);
    // 将SE3的平移向量的前两个元素复制到SE2矩阵
    se2_matrix.block<2, 1>(0, 2) = translation.block<2, 1>(0, 0);

    // 设置SE2矩阵的右下角元素为1
    se2_matrix(2, 2) = 1;

    // 创建并返回SE2位姿
    return SE2(se2_matrix);
}
void LooselyLIO::Align(const MeasureGroup &meas) {

    /// 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
    SE3 pose_predict = eskf_.GetNominalSE3();
    SE2 pose_eskf=SE3toSE2(pose_predict);
    mapping.ProcessScan(meas.lidar_,pose_eskf);

    pose_of_lo_ = pose_predict;
    eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);

    // if (options_.with_ui_) {
    //     // 放入UI
    //     ui_->UpdateScan(current_scan, eskf_.GetNominalSE3());  // 转成Lidar Pose传给UI
    //     ui_->UpdateNavState(eskf_.GetNominalState());
    // }
    frame_num_++;
}

void LooselyLIO::ScanCallBack(const sensor_msgs::LaserScan::Ptr &msg){sync_->ProcessScan(msg);};
void LooselyLIO::IMUCallBack(IMUPtr msg_in) { sync_->ProcessIMU(msg_in); }

void LooselyLIO::Finish() {
    if (options_.with_ui_) {
        while (ui_->ShouldQuit() == false) {
            usleep(1e5);
        }

        ui_->Quit();
    }
    LOG(INFO) << "finish done";
}

}  // namespace sad