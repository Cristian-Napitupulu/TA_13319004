#include "../include/realsense_connector.h"

REALSENSE_DEVICE::REALSENSE_DEVICE(int color_mode_, int width_, int height_, int fps_) : REALSENSE_DEVICE::REALSENSE_DEVICE()
{
    this->color_mode = color_mode_;
    this->width = width_;
    this->height = height_;
    this->fps = fps_;

    configuration.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    colorizer.set_option(RS2_OPTION_COLOR_SCHEME, color_mode);

    pipe.start(configuration);

    REALSENSE_DEVICE::set_distance_range(min_distance, max_distance);
}

void REALSENSE_DEVICE::set_distance_range(float min_distance_, float max_distance_)
{
    this->min_distance = min_distance_;
    this->max_distance = max_distance_;

    depth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);
    depth_sensor.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);
}

bool REALSENSE_DEVICE::wait_for_frames()
{
    frames = pipe.wait_for_frames();
    return true;
}

rs2::frame REALSENSE_DEVICE::get_depth_frame()
{
    if (REALSENSE_DEVICE::wait_for_frames())
    {
        rs2::frame depth_frame = frames.get_depth_frame();
        depth_frame = threshold_filter.process(depth_frame);
        depth_frame = colorizer.colorize(depth_frame);
        return depth_frame;
    }
    else
    {
        std::cout << "Error: Could not get depth frame" << std::endl;
        return rs2::frame();
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr REALSENSE_DEVICE::get_pointcloud()
{
    rs2::frame depth_frame = REALSENSE_DEVICE::get_depth_frame();
    points = pointcloud.calculate(depth_frame);
    const rs2::vertex *vertices = points.get_vertices();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < points.size(); i++)
    {
        pcl_pointcloud->points[i].x = vertices[i].x;
        pcl_pointcloud->points[i].y = vertices[i].y;
        pcl_pointcloud->points[i].z = vertices[i].z;
        pcl_pointcloud->push_back(pcl_pointcloud->points[i]);
    }
    return pcl_pointcloud;
}

