#include "../include/pointcloud_gantry/realsense_connector.h"

REALSENSE_DEVICE::REALSENSE_DEVICE(int color_mode, int frame_width, int frame_height, int fps, float min_distance, float max_distance) : color_mode_(color_mode), frame_width_(frame_width), frame_height_(frame_height), fps_(fps), min_distance_(min_distance), max_distance_(max_distance)
{
    this->color_mode_ = color_mode;
    this->frame_width_ = frame_width;
    this->frame_height_ = frame_height;
    this->fps_ = fps;
    this->min_distance_ = min_distance;
    this->max_distance_ = max_distance;
}

void REALSENSE_DEVICE::initialize()
{
    context_ = rs2::context();
    this->configuration_.enable_stream(RS2_STREAM_DEPTH, this->frame_width_, this->frame_height_, RS2_FORMAT_Z16, this->fps_);
    this->pipeline_.start(this->configuration_);

    this->colorizer_.set_option(RS2_OPTION_COLOR_SCHEME, this->color_mode_);

    this->threshold_filter_.set_option(RS2_OPTION_MIN_DISTANCE, this->min_distance_);
    this->threshold_filter_.set_option(RS2_OPTION_MAX_DISTANCE, this->max_distance_);
}

rs2::frame REALSENSE_DEVICE::get_depth_frame()
{
    frames_ = this->pipeline_.wait_for_frames();
    rs2::frame depth_frame = frames_.get_depth_frame();
    rs2::frame depth_frame_ = this->threshold_filter_.process(depth_frame);
    return depth_frame_;
}

void REALSENSE_DEVICE::calculate_pointcloud(rs2::frame depth_frame)
{
    this->realsense_points = this->realsense_pointcloud.calculate(depth_frame);
    const rs2::vertex *vertices = this->realsense_points.get_vertices();

    this->pcl_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < (this->frame_width_ * this->frame_height_); i++)
    {
        if (
            (vertices[i].x == 0.0) &&
            (vertices[i].y == 0.0) &&
            (vertices[i].z == 0.0))
        {
            continue;
        }
        pcl::PointXYZ pcl_point;
        pcl_point.x = vertices[i].x;
        pcl_point.y = vertices[i].y;
        pcl_point.z = vertices[i].z;
        this->pcl_pointcloud_->push_back(pcl_point);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr REALSENSE_DEVICE::pointcloud()
{
    return this->pcl_pointcloud_;
}

rs2::frame REALSENSE_DEVICE::get_colorized_depth_frame(rs2::frame depth_frame)
{
    rs2::frame depth_frame_ = depth_frame.apply_filter(this->colorizer_);
    return depth_frame_;
}

int REALSENSE_DEVICE::get_frame_width()
{
    return this->frame_width_;
}

int REALSENSE_DEVICE::get_frame_height()
{
    return this->frame_height_;
}
