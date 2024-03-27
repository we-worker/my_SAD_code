
// #include "ch6/loosely_lio2d/loosely_lio.cc"
//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/mapping_2d.h"
#include "common/io_utils.h"
// #include "measure_sync.h"
#include "ch6/loosely_lio2d/loosely_lio.h"
#include "common/timer/timer.h"


DEFINE_string(bag_path, "./dataset/sad/2dmapping/31.bag", "数据包路径");
DEFINE_bool(with_loop_closing, false, "是否使用回环检测");

/// 测试2D lidar SLAM

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    // sad::RosbagIO rosbag_io("./dataset/sad/2dmapping/31.bag");
    sad::Mapping2D mapping;

    // sad::LooselyLIO::Options options;
    // options.with_ui_ = FLAGS_display_map;
    // sad::LooselyLIO lm(options);
    sad::LooselyLIO lm;
    // lm.Init();
    // lm.Init(FLAGS_config);

    // if (mapping.Init(FLAGS_with_loop_closing) == false) {
    //     return -1;
    // }
    // std::shared_ptr<sad::MessageSync> sync_ = nullptr;  // 消息同步器
    // sync_ = std::make_shared<sad::MessageSync>([](const sad::MeasureGroup &m) {});
    rosbag_io
        .AddScan2DHandle("/scan", [&](Scan2d::Ptr scan) { 
            // mapping.ProcessScan(scan);
            // sync_->ProcessScan(scan);
            
            sad::common::Timer::Evaluate([&]() { lm.ScanCallBack(scan); }, "loosely lio");
            return true; 
        })

            // return lm.PCLCallBack(scan); })

        .AddImuHandle("/handsfree/imu",[&](IMUPtr imu) { 
            // sync_->ProcessIMU(imu);
            lm.IMUCallBack(imu);
            return true;
        })
    .Go();
    cv::imwrite("./data/ch6/global_map.png", mapping.ShowGlobalMap(2000));
    // lm.Finish();
    // sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";
    return 0;
}