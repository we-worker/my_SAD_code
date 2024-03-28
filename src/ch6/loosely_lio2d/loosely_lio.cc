
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
    Undistort();

    // 配准
    Align();
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

void LooselyLIO::Undistort() {
    auto imu_state = eskf_.GetNominalState();  // 最后时刻的状态
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    std::vector<int> index(measures_.lidar_->ranges.size());
    std::for_each(index.begin(), index.end(), [idx = 0](int& i) mutable { i = idx++; });

    /// 将所有点转到最后时刻状态上
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto &i) {
        SE3 Ti = T_end;
        NavStated match;

        // 根据时间查找状态
        math::PoseInterp<NavStated>(
            measures_.lidar_->header.stamp.toSec() + i * measures_.lidar_->time_increment, imu_states_, [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

        //新建一个点，基于点去畸变
        float angle = measures_.lidar_->angle_min + i * measures_.lidar_->angle_increment;
        Vec3d pi(measures_.lidar_->ranges[i] * cos(angle), measures_.lidar_->ranges[i] * sin(angle), 0);

        Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

        // 更新measures_.lidar_->range,没有更新角度，因为不太能操作，如需更新角度，需要直接使用点云类型
        measures_.lidar_->ranges[i] = sqrt(p_compensate(0)*p_compensate(0) + p_compensate(1)*p_compensate(1));
        //scan.angle_min = atan2(p_compensate(1), p_compensate(0));
    });
}




SE2 SE3toSE2(const SE3& pose) {
    // 提取SE3位姿的旋转和平移部分
    Eigen::Matrix3d rotation = pose.rotationMatrix();
    Eigen::Vector3d translation = pose.translation();

    // 创建一个动态大小的矩阵，将SE3的旋转矩阵的左上角2x2部分复制到这个矩阵
    Eigen::MatrixXd rotation2d = rotation.block<2, 2>(0, 0);

    // 使用JacobiSVD计算一个最接近原矩阵的正交矩阵，U和V的转置的乘积是一个正交矩阵，U和V都是正交矩阵
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(rotation2d, Eigen::ComputeThinU | Eigen::ComputeThinV);
    rotation2d = svd.matrixU() * svd.matrixV().transpose();

    // 创建一个3x3的SE2矩阵
    Eigen::Matrix3d se2_matrix = Eigen::Matrix3d::Identity();
    // 将正交化后的2x2旋转矩阵复制到SE2矩阵的左上角
    se2_matrix.block<2, 2>(0, 0) = rotation2d;
    // 将SE3的平移向量的前两个元素复制到SE2矩阵
    se2_matrix.block<2, 1>(0, 2) = translation.block<2, 1>(0, 0);
    LOG(INFO) <<"predict\n"<<se2_matrix;
    // 创建并返回SE2位姿
    return SE2(se2_matrix);
}
SE3 SE2toSE3(const SE2& pose) {
    // 提取SE2位姿的旋转和平移部分
    Eigen::Matrix2d rotation = pose.rotationMatrix();
    Eigen::Vector2d translation = pose.translation();
    LOG(INFO) <<"observe_R\n"<<rotation;
    LOG(INFO) <<"observe_T\n"<<translation;
    // 创建一个4x4的SE3矩阵
    Eigen::Matrix4d se3_matrix = Eigen::Matrix4d::Identity();

    // 将SE2的旋转矩阵复制到SE3矩阵的左上角2x2部分
    se3_matrix.block<2, 2>(0, 0) = rotation.block<2, 2>(0, 0);

    // 将SE2的平移向量复制到SE3矩阵
    se3_matrix.block<2, 1>(0, 3) = translation;

    // 创建并返回SE3位姿
    return SE3(se3_matrix);
}

void LooselyLIO::Align() {

    /// 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
    if(frame_num_){
        SE3 pose_predict = eskf_.GetNominalSE3();
        SE2 pose_eskf=SE3toSE2(pose_predict);
        mapping.ProcessScan(measures_.lidar_,pose_eskf);
    }else{
        mapping.ProcessScan(measures_.lidar_);
    }
    
    SE3 observePose=SE2toSE3(mapping.current_frame_->pose_);
    eskf_.ObserveSE3(observePose, 1e-2, 1e-2);

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