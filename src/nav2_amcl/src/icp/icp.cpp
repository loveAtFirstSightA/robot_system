/*
 Copyright 2024 ROS2 LLC

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include "nav2_amcl/icp/icp.hpp"
#include "nav2_amcl/logger.hpp"

namespace nav2_amcl
{
ICP::ICP()
{
    // 
}

ICP::~ICP()
{
    // 
}

void
ICP::converMapToPcl(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    std::cout << getCurrentTime() << "Start converting map data" << std::endl;
    unsigned int width = static_cast<unsigned int>(map->info.width);
    unsigned int height = static_cast<unsigned int>(map->info.height);
    double resolution = static_cast<double>(map->info.resolution);
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;
    std::cout << getCurrentTime() << "map: " << width << " * " << height << " " << resolution << " [" << origin_x << " ," << origin_y << "]" << std::endl;
    // map data_status occ - 100 unknown - -1 free - 0
    // Obstacles
    unsigned int sum_occ = 0;
    std::vector<double> occ_x;
    std::vector<double> occ_y;
    occ_x.clear();
    occ_y.clear();
    for (size_t i = 0; i < map->data.size(); i++) {
        // std::cout << getCurrentTime() << i << ": " << static_cast<int>(map->data[i]) << std::endl;
        if (map->data[i] == 100) {
            sum_occ ++;
            // Extract the location of each obstacle point under the coordinates of the map
            // Calculate the sequence and sequence of the grid
            int row = i / width;     //   Line number
            int col = i % width;     //   Column serial number
            // Calculate the actual position
            double x = origin_x + row * resolution;
            double y = origin_y + col * resolution;
            occ_x.push_back(x);
            occ_y.push_back(y);
        }
    }
    // convert pcl
    // Create a point cloud pointer in PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // Clear cloud data
    point_tmp->clear();
    // Set the point cloud size as the obstacle point.
    point_tmp->resize(sum_occ);
    // Create PCL dot variables
    pcl::PointXYZ pcl_point;
    // Traversing OCC data
    for (size_t i = 0; i < sum_occ; i++) {
        pcl_point.x = occ_x[i];
        pcl_point.y = occ_y[i];
        pcl_point.z = 0.0f;
        point_tmp->push_back(pcl_point);
        point_tmp->push_back(pcl_point);
    }
    
    // Set the width, height and dense attributes of the point cloud
    point_tmp->width = point_tmp->size();
    point_tmp->height = 1;
    point_tmp->is_dense = true;

    // Convert the header of the sensor message to the header in PCL format
    pcl_conversions::toPCL(map->header, point_tmp->header);
    target_pointcloud_ = point_tmp;
}

void
ICP::converScanToPcl(const sensor_msgs::msg::LaserScan::SharedPtr scan, double x, double y, double yaw)
{
    // Create a point cloud pointer in PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // Clear cloud data
    point_tmp->clear();
    // Set point cloud size to radar data size
    point_tmp->resize(scan->ranges.size());
    // Create PCL dot variables
    pcl::PointXYZ pcl_point;
        // Traversing radar data
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            // Proposal invalid data
            if (!std::isfinite(range)) {
                continue;
            }
            // Determine whether the data is within the valid range
            if (range > scan->range_min && range < scan->range_max) {
                // The polar coordinate angle of the calculation point
                float angle = scan->angle_min + scan->angle_increment * i;
                float lx = range * std::cos(angle);
                float ly = range * std::sin(angle);

                // Considering the coordinate system transform radar coordinate system transformation to the map coordinate system
                float mx = lx * std::cos(yaw) - ly * std::sin(yaw) + x;
                float my = lx * std::sin(yaw) + ly * std::cos(yaw) + y;

                // Convert the pole coordinates to Cartesian coordinates
                pcl_point.x = mx;
                pcl_point.y = my;
                pcl_point.z = 0.0f;

                //Add the point to the point cloud
                point_tmp->push_back(pcl_point);
            }
        }

    // Set the width, height and dense attributes of the point cloud
    point_tmp->width = point_tmp->size();
    point_tmp->height = 1;
    point_tmp->is_dense = true;

    // Convert the header of the sensor message to the header in PCL format
    pcl_conversions::toPCL(scan->header, point_tmp->header);
    // Assign the conversion point cloud to the current point cloud data
    source_pointcloud_ = point_tmp;
}

bool
ICP::processICPScanWithMap(int times, float & x, float & y, float & yaw)
{
    icp_.setInputSource(source_pointcloud_);
    icp_.setInputTarget(target_pointcloud_);
    icp_.setMaximumIterations(times);

    pcl::PointCloud<pcl::PointXYZ> unused_result;
    icp_.align(unused_result);

    if (icp_.hasConverged() == false) {
        std::cout << "Not Converged" << std::endl;
        return false;
    } else {
        // After converging, get coordinate changes
        Eigen::Affine3f transfrom;
        transfrom = icp_.getFinalTransformation();
        // Convert EIGEN :: Affine3F to x, y, theta, and print it
        // float x, y, z, roll, pitch, yaw;
        float z, roll, pitch;
        pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
        std::cout << getCurrentTime() << "icp transfrom: [" << x << ", " << y << ", " << yaw << "]" << std::endl;
        return true;
    }
}

}  // namespace nav2_amcl
