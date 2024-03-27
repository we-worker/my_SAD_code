//
// Created by xiang on 2022/7/21.
//
#include "ch7/loam-like/feature_extraction.h"
#include <glog/logging.h>

#include <execution>

namespace sad {

void FeatureExtraction::Extract(FullCloudPtr pc_in, CloudPtr pc_out_edge, CloudPtr pc_out_surf) {
    int num_scans = 16;
    std::vector<CloudPtr> scans_in_each_line;  // 分线数的点云
    for (int i = 0; i < num_scans; i++) {
        scans_in_each_line.emplace_back(new PointCloudType);
    }

    for (auto &pt : pc_in->points) {
        assert(pt.ring >= 0 && pt.ring < num_scans);
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;

        scans_in_each_line[pt.ring]->points.emplace_back(p);
    }

    // 处理曲率
    // for (int i = 0; i < num_scans; i++) {
    std::for_each(std::execution::par_unseq, scans_in_each_line.begin(), scans_in_each_line.end(), [this,&pc_in,&pc_out_edge,&pc_out_surf](auto & scan_line) {
        if (scan_line->points.size() < 131) {
            return;
        }

        std::vector<IdAndValue> cloud_curvature;  // 每条线对应的曲率
        int total_points = scan_line->points.size() - 10;
    
        std::vector<int> indices(scan_line->points.size());
        std::iota(indices.begin(), indices.end(), 0);  // Fill with 0, 1, ..., n.

        std::transform(std::execution::par_unseq, indices.begin() + 5, indices.end() - 5, cloud_curvature.begin(),
            [scan_line](const auto& index) {
                // const auto& point = scan_line->points[index];
                // 两头留一定余量，采样周围10个点取平均值
                double diffX = scan_line->points[index- 5].x + scan_line->points[index- 4].x + scan_line->points[index- 3].x + scan_line->points[index- 2].x + scan_line->points[index- 1].x 
                                - 10 * scan_line->points[index].x +scan_line->points[index+ 1].x + scan_line->points[index+ 2].x + scan_line->points[index+ 3].x + scan_line->points[index+ 4].x + scan_line->points[index+ 5].x;
                double diffY = scan_line->points[index- 5].y + scan_line->points[index- 4].y + scan_line->points[index- 3].y + scan_line->points[index- 2].y + scan_line->points[index- 1].y
                                - 10 * scan_line->points[index].y +scan_line->points[index+ 1].y + scan_line->points[index+ 2].y + scan_line->points[index+ 3].y + scan_line->points[index+ 4].y + scan_line->points[index+ 5].y;
                double diffZ = scan_line->points[index- 5].z + scan_line->points[index- 4].z + scan_line->points[index- 3].z + scan_line->points[index- 2].z + scan_line->points[index- 1].z
                                - 10 * scan_line->points[index].z +scan_line->points[index+ 1].z + scan_line->points[index+ 2].z + scan_line->points[index+ 3].z + scan_line->points[index+ 4].z + scan_line->points[index+ 5].z;
                return IdAndValue(index, diffX * diffX + diffY * diffY + diffZ * diffZ);  // 创建distance对象
            });

        // 对每个区间选取特征，把360度分为6个区间
        for (int j = 0; j < 6; j++) {
            int sector_length = (int)(total_points / 6);
            int sector_start = sector_length * j;
            int sector_end = sector_length * (j + 1) - 1;
            if (j == 5) {
                sector_end = total_points - 1;
            }

            std::vector<IdAndValue> sub_cloud_curvature(cloud_curvature.begin() + sector_start,
                                                        cloud_curvature.begin() + sector_end);

            ExtractFromSector(scan_line, sub_cloud_curvature, pc_out_edge, pc_out_surf);
        }
    });
}

void FeatureExtraction::ExtractFromSector(const CloudPtr &pc_in, std::vector<IdAndValue> &cloud_curvature,
                                          CloudPtr &pc_out_edge, CloudPtr &pc_out_surf) {
    // 按曲率排序
    std::sort(cloud_curvature.begin(), cloud_curvature.end(),
              [](const IdAndValue &a, const IdAndValue &b) { return a.value_ < b.value_; });

    int largest_picked_num = 0;
    int point_info_count = 0;

    /// 按照曲率最大的开始搜，选取曲率最大的角点
    std::vector<int> picked_points;  // 标记被选中的角点，角点附近的点都不会被选取
    for (int i = cloud_curvature.size() - 1; i >= 0; i--) {
        int ind = cloud_curvature[i].id_;//ind为点的索引
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
            if (cloud_curvature[i].value_ <= 0.1) {
                break;
            }

            largest_picked_num++;
            picked_points.push_back(ind);

            if (largest_picked_num <= 20) {//只选20个角点
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            } else {
                break;
            }

            for (int k = 1; k <= 5; k++) {
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                    break;
                }
                picked_points.push_back(ind + k);//相近的点直接加入点集，下次pc_out_edge不会再加入
            }
            for (int k = -1; k >= -5; k--) {
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                    break;
                }
                picked_points.push_back(ind + k);
            }
        }
    }

    /// 选取曲率较小的平面点（去掉角点和角点附近的点）
    for (int i = 0; i <= (int)cloud_curvature.size() - 1; i++) {
        int ind = cloud_curvature[i].id_;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
}

}  // namespace sad