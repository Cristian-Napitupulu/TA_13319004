#include "../include/pointcloud_gantry/object_detection_algorithm.h"

PCL_PLANE_SEGMENTATION::PCL_PLANE_SEGMENTATION(int Method, float distance_threshold) : Method_(Method), distance_threshold_(distance_threshold)
{
    this->Method_ = Method;
    this->distance_threshold_ = distance_threshold;
}