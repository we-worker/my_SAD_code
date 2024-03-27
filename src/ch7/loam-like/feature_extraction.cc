//
// Created by xiang on 2022/7/21.
//
#include "ch7/loam-like/feature_extraction.h"
#include <glog/logging.h>

#include <execution>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <common/math_utils.h>

namespace sad {


void FeatureExtraction::groundRemoval(std::vector<CloudPtr> &scans_in_each_line,int num_scans,CloudPtr &pc_out_ground) {
    // 假设我们已经知道每条扫描线上的点的数量为num_points
    int num_points = scans_in_each_line[0]->points.size();
    // 初始化_ground_mat为一个num_scans x num_points的零矩阵
    bool _ground_mat[int(num_scans/2-1)][num_points]={0};
  // 同一列由下到上 _horizontal_scans = 1800  _ground_scan_index=7(地面点的上限)16线设置为7  因为水平地面点不可能在天上面对把
  for (size_t j = 0; j < num_points; ++j)
  {
    for (size_t i = 0; i < int(num_scans/2)-2; ++i)
    {
        //跳过无效点
        if (scans_in_each_line[i]->points[j].intensity == -1 ||
            scans_in_each_line[i+1]->points[j].intensity == -1){
            continue;
        }

      // 取出同一列相邻点的索引
      PointType lowerp = scans_in_each_line[i]->points[j];
      PointType upperp = scans_in_each_line[i+1]->points[j];
 
 
      float dX =upperp.x - lowerp.x;
      float dY =upperp.y - lowerp.y;
      float dZ =upperp.z - lowerp.z;
 
      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY));
 
      // TODO: review this change
      // 和上文的公式相同 _sensor_mount_angle设置为（_sensor_mount_angle *= DEG_TO_RAD） （DEG_TO_RAD = M_PI / 180.0）
      if ( vertical_angle  <= 10 * math::kDEG2RAD)
      {
        _ground_mat[i][j] = true;
        _ground_mat[i + 1][j] = true;
      }
    }
  }
  // 遍历特征矩阵 1800 * 16 如果是地面点/无效点的话 设置为-1 不参与线特征面特征的提取

  pcl::ExtractIndices<sad::PointType> extract;
  for (size_t i = 0; i < int(num_scans/2-1); ++i)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());//删除的点的索引
    for (size_t j = 0; j < num_points; ++j)
    {
        if (_ground_mat[i][j])
        {
            pc_out_ground->push_back(scans_in_each_line[i]->points[j]);
            inliers->indices.push_back(j);
        }
    }
    if(scans_in_each_line[i]->points.size()<inliers->indices.size()){
      LOG(ERROR)<<"inliers_size"<<inliers->indices.size()<<"scan_size"<<scans_in_each_line[i]->points.size();
    }
    extract.setInputCloud(scans_in_each_line[i]);
    extract.setIndices(inliers);
    extract.setNegative(true);//“负向”运行 filter() 方法，得到原始云减去你的点数
    extract.filter(*scans_in_each_line[i]);
  }

//   if(pc_out_ground->size()>30){
//     groundCloud_legoloam->height = 1;
//     groundCloud_legoloam->width = groundCloud_legoloam->size();
//     pcl::io::savePCDFileASCII("./data/ch7/ground_LegoLOAM.pcd", *groundCloud_legoloam);
//   }

}




void FeatureExtraction::Extract(FullCloudPtr pc_in, CloudPtr pc_out_edge, CloudPtr pc_out_surf, CloudPtr pc_out_ground) {
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

    groundRemoval(scans_in_each_line,num_scans,pc_out_ground);//地面去除

    
    // 处理曲率
    std::for_each(std::execution::seq, scans_in_each_line.begin(), scans_in_each_line.end(), [this,&pc_in,&pc_out_edge,&pc_out_surf](auto & scan_line) {
        if (scan_line->points.size() < 131) {
            return;
        }

        std::vector<IdAndValue> cloud_curvature;  // 每条线对应的曲率
        int total_points = scan_line->points.size() - 10;
        
        std::vector<int> indices(scan_line->points.size());//新建一个索引容器
        std::iota(indices.begin(), indices.end(), 0);  // Fill with 0, 1, ..., n.
        cloud_curvature.resize(scan_line->points.size()-10);//调整大小
        std::transform(std::execution::par_unseq, indices.begin() + 5, indices.end() - 5, cloud_curvature.begin(),
            [scan_line](const auto& index) {
                if (index - 5 >= 0 && index + 5 < scan_line->points.size()) {
                // const auto& point = scan_line->points[index];
                // 两头留一定余量，采样周围10个点取平均值
                double diffX = scan_line->points[index- 5].x + scan_line->points[index- 4].x + scan_line->points[index- 3].x + scan_line->points[index- 2].x + scan_line->points[index- 1].x 
                                - 10 * scan_line->points[index].x +scan_line->points[index+ 1].x + scan_line->points[index+ 2].x + scan_line->points[index+ 3].x + scan_line->points[index+ 4].x + scan_line->points[index+ 5].x;
                double diffY = scan_line->points[index- 5].y + scan_line->points[index- 4].y + scan_line->points[index- 3].y + scan_line->points[index- 2].y + scan_line->points[index- 1].y
                                - 10 * scan_line->points[index].y +scan_line->points[index+ 1].y + scan_line->points[index+ 2].y + scan_line->points[index+ 3].y + scan_line->points[index+ 4].y + scan_line->points[index+ 5].y;
                double diffZ = scan_line->points[index- 5].z + scan_line->points[index- 4].z + scan_line->points[index- 3].z + scan_line->points[index- 2].z + scan_line->points[index- 1].z
                                - 10 * scan_line->points[index].z +scan_line->points[index+ 1].z + scan_line->points[index+ 2].z + scan_line->points[index+ 3].z + scan_line->points[index+ 4].z + scan_line->points[index+ 5].z;
                return IdAndValue(index, diffX * diffX + diffY * diffY + diffZ * diffZ);  // 创建distance对象
                }
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
        int ind = cloud_curvature[i].id_;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
            if (cloud_curvature[i].value_ <= 0.1) {
                break;
            }

            largest_picked_num++;
            picked_points.push_back(ind);

            if (largest_picked_num <= 20) {
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
                picked_points.push_back(ind + k);
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

    /// 选取曲率较小的平面点
    for (int i = 0; i <= (int)cloud_curvature.size() - 1; i++) {
        int ind = cloud_curvature[i].id_;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
}

}  // namespace sad