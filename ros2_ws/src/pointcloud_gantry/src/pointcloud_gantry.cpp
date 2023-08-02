#include "../include/pointcloud_gantry/realsense_connector.h"
#include <iostream>

#include <opencv2/opencv.hpp>

REALSENSE_DEVICE LIDAR_L515(0, 640, 480, 30, 0.15f, 2.00f);

int main()
{
    std::cout << "Pointcloud Gantry" << std::endl;
    LIDAR_L515.initialize();
    while (true)
    {
        rs2::frame depth_frame = LIDAR_L515.get_depth_frame();

        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        LIDAR_L515.get_pointcloud(depth_frame, pointcloud);

        // // Print X, Y, Z value
        // for (int i = 0; i < (int)pointcloud->size(); i++)
        // {
        //     std::cout << "X: " << pointcloud->points[i].x << " Y: " << pointcloud->points[i].y << " Z: " << pointcloud->points[i].z << std::endl;
        // }

        // rs2::frame colorized_frame = LIDAR_L515.get_colorized_depth_frame(depth_frame);
        // cv::Mat image(cv::Size(LIDAR_L515.get_frame_width(), LIDAR_L515.get_frame_height()), CV_8UC3, (void *)colorized_frame.get_data(), cv::Mat::AUTO_STEP);
        // cv::imshow("Lalala", image);

        // if (cv::waitKey(1) == 'q')
        // {
        //     break;
        // }
    }
}