#ifndef REALSENSE_CONNECTOR_H
#define REALSENSE_CONNECTOR_H

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

class REALSENSE_DEVICE
{
public:
    // Constructor
    REALSENSE_DEVICE(int color_mode_, int width_, int height_, int fps_);
    REALSENSE_DEVICE();
    void set_distance_range(float min_distance_, float max_distance_);
    bool wait_for_frames();
    rs2::frame get_depth_frame();
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pointcloud();

private:
    rs2::pipeline pipe;
    rs2::config configuration;
    rs2::depth_sensor depth_sensor;
    rs2::threshold_filter threshold_filter;
    rs2::colorizer colorizer;
    rs2::frameset frames;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int color_mode = 2;
    int width;
    int height;
    int fps;
    float min_distance = 0.15;
    float max_distance = 3.0;
};

#endif // REALSENSE_CONNECTOR_H