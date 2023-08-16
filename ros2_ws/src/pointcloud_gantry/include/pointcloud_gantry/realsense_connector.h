#ifndef REALSENSE_CONNECTOR_H
#define REALSENSE_CONNECTOR_H

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class REALSENSE_DEVICE
{
public:
    // Constructor
    REALSENSE_DEVICE(int color_mode, int frame_width, int frame_height, int fps, float min_distance, float max_distance);
    ~REALSENSE_DEVICE() = default;
    void initialize();
    rs2::frame get_depth_frame();
    void calculate_pointcloud(rs2::frame depth_frame);
    rs2::frame get_colorized_depth_frame(rs2::frame depth_frame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud();


    int get_frame_width();
    int get_frame_height();

private:
    int color_mode_;
    int frame_width_;
    int frame_height_;
    int fps_;
    float min_distance_;
    float max_distance_;

    rs2::context context_;
    rs2::config configuration_;
    rs2::pipeline pipeline_;
    rs2::threshold_filter threshold_filter_;
    rs2::colorizer colorizer_;
    rs2::frameset frames_;
    rs2::pointcloud realsense_pointcloud;
    rs2::points realsense_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud_;
};

#endif // REALSENSE_CONNECTOR_H