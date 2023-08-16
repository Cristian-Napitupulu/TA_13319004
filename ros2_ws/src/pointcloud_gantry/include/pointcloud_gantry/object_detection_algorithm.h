#ifndef OBJECT_DETECTION_ALGORITHM_H
#define OBJECT_DETECTION_ALGORITHM_H

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class PCL_PLANE_SEGMENTATION
{
    public:
    PCL_PLANE_SEGMENTATION (int Method, float distance_threshold);
    ~PCL_PLANE_SEGMENTATION() = default;
    void evaluate(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud_input);

    private:
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::SACSegmentation<pcl::PointXYZ> segmentator;

    int Method_;
    float distance_threshold_;

};


#endif  // OBJECT_DETECTION_ALGORITHM_H