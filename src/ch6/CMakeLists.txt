add_library(${PROJECT_NAME}.ch6
        icp_2d.cc
        lidar_2d_utils.cc
        likelihood_field.cc
        occupancy_map.cc
        submap.cc
        mapping_2d.cc
        multi_resolution_likelihood_field.cc
        loop_closing.cc
        frame.cc


        loosely_lio2d/loosely_lio.cc
        loosely_lio2d/measure_sync.cc
        # loosely_lio2d/test_loosely_lio2d.cc
        )

target_link_libraries(${PROJECT_NAME}.ch6
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}.common
        )

add_executable(test_2dlidar_io
        test_2dlidar_io.cc
        )

target_link_libraries(test_2dlidar_io
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${third_party_libs}
        )

add_executable(test_2d_icp_s2s
        test_2d_icp_s2s.cc
        )

target_link_libraries(test_2d_icp_s2s
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        )

add_executable(test_2d_icp_likelihood
        test_2d_icp_likelihood.cc
        )

target_link_libraries(test_2d_icp_likelihood
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(test_occupancy_grid
        test_occupancy_grid.cc
        )

target_link_libraries(test_occupancy_grid
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(test_2d_mapping
        test_2d_mapping.cc
        )

target_link_libraries(test_2d_mapping
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(test_mr_matching
        test_mr_matching.cc
        )

target_link_libraries(test_mr_matching
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

# add_subdirectory(loosely_lio2d)
add_executable(test_loosely_lio2d
        loosely_lio2d/test_loosely_lio2d.cc
        loosely_lio2d/measure_sync.cc
        loosely_lio2d/loosely_lio.cc
        )

target_link_libraries(test_loosely_lio2d
        ${PROJECT_NAME}.ch6
        ${PROJECT_NAME}.common
        ${third_party_libs}
        glog gflags
)
# add_executable(pointcloud_clustering
# pointcloud_clustering.cc
#         )

# target_link_libraries(pointcloud_clustering
#         ${PROJECT_NAME}.ch6
#         ${OpenCV_LIBS}
#         ${catkin_LIBRARIES}
#         glog gflags
#         ${PCL_LIBRARIES}
#         )
