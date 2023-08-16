#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <QApplication>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

const int frame_width = 640;  // pixels
const int frame_height = 480; // pixels
const int frame_rate = 30;    // fps

const float max_distance = 1.00f; // meters
const float min_distance = 0.15f; // meters

// Custom comparator function for sorting vertices by X-coordinate
bool compareVerticesX(const rs2::vertex &vertex1, const rs2::vertex &vertex2)
{
    return vertex1.x < vertex2.x;
}

// Custom comparator function for sorting vertices by Y-coordinate
bool compareVerticesY(const rs2::vertex &vertex1, const rs2::vertex &vertex2)
{
    return vertex1.y < vertex2.y;
}
// Custom comparator function for sorting vertices by Z-coordinate
bool compareVerticesZ(const rs2::vertex &vertex1, const rs2::vertex &vertex2)
{
    return vertex1.z < vertex2.z;
}

int main(int argc, char *argv[])
{

    QApplication application(argc, argv);

    QWidget depth_frame_window;
    depth_frame_window.setMinimumSize(frame_width, frame_height);

    QLabel depth_frame_label(&depth_frame_window);
    depth_frame_label.setAlignment(Qt::AlignCenter);

    // QWidget text_window;
    // text_window.setMinimumSize(frame_width, 20);
    // QLabel text_label(&text_window);
    // text_label.setText("Depth Frame");
    // text_label.setAlignment(Qt::AlignCenter);

    QVBoxLayout layout;
    layout.addWidget(&depth_frame_label);
    // layout.addWidget(&text_label);
    depth_frame_window.setLayout(&layout);

    depth_frame_window.show();

    rs2::pipeline pipe;
    rs2::config configuration;

    configuration.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, frame_rate);
    pipe.start(configuration);

    rs2::depth_sensor depth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0);
    depth_sensor.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);
    // depth_sensor.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);

    rs2::threshold_filter threshold_filter;
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);

    rs2::colorizer colorizer;
    colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 2);

    // Set up RealSense PointCloud object
    rs2::pointcloud pointcloud;
    rs2::points points;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::ModelCoefficients> plane_coefficients;

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
                     {
        // Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame depth_frame = frames.get_depth_frame();
        rs2::frame filtered_frame = threshold_filter.process(depth_frame);
        rs2::frame colorized_frame = filtered_frame.apply_filter(colorizer);
        
        // Generate the pointcloud
        points = pointcloud.calculate(filtered_frame);

        // // Get the vertices and texture coordinates
        // const rs2::vertex* vertices = points.get_vertices();
        // const int num_vertices = points.size();

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->points.resize(cloud->width * cloud->height);

        auto ptr = points.get_vertices();
        for (auto& point : *cloud) {
            point.x = ptr->x;
            point.y = ptr->y;
            point.z = ptr->z;
            ptr++;
        }

        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setMaxIterations(1000);
        // seg.setDistanceThreshold(0.05);

        // while (cloud->size() > 10000){
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // seg.setInputCloud(cloud);
        // seg.segment(*inliers, *coefficients);

        // plane_coefficients.push_back(*coefficients);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::ExtractIndices<pcl::PointXYZ> extract;
        // extract.setInputCloud(cloud);
        // extract.setIndices(inliers);
        // extract.setNegative(false); // Extract the inliers (plane points)
        // extract.filter(*plane);

        // // Remove the inliers from the cloud to focus on remaining points
        // extract.setNegative(true); // Extract the outliers (non-plane points)
        // extract.filter(*cloud);
        // }

        // if (plane_coefficients.size() >= 2) {
        //     // Assume the first two planes are perpendicular
        //     float dot_product = plane_coefficients[0].values[0] * plane_coefficients[1].values[0] +
        //                        plane_coefficients[0].values[1] * plane_coefficients[1].values[1] +
        //                        plane_coefficients[0].values[2] * plane_coefficients[1].values[2];

        //     if (dot_product == 0.0) {
        //         std::cout << "Two detected planes are perpendicular to each other." << std::endl;
        //     } else {
        //         std::cout << "Two detected planes are NOT perpendicular to each other." << std::endl;
        //     }
        // }


        // // Remove all vertices with a Z-coordinate of 0 (to remove depth noise)
        // std::vector<rs2::vertex> vertices_filtered;
        // for (int i = 0; i < num_vertices; i++) {
        //     if (vertices[i].z != 0) {
        //         vertices_filtered.push_back(vertices[i]);
        //     }
        // }

        // // Sort the vertices by Z-coordinate (from smallest to largest)
        // std::sort(vertices_filtered.begin(), vertices_filtered.end(), compareVerticesZ);

        // // Get the vertices with certain threshold value between each Z-coordinate value
        // const float threshold_Z = 0.1f; // meters
        // std::vector<rs2::vertex> vertices_threshold_Z;
        // for (int i = 0; i < (int) vertices_filtered.size(); i++) {
        //     if (vertices_filtered[i].z - vertices_filtered[i - 1].z < threshold_Z) {
        //         vertices_threshold_Z.push_back(vertices_filtered[i]);
        //     }
        // }

        // // Sort the vertices by Y-coordinate (from smallest to largest)
        // std::sort(vertices_threshold_Z.begin(), vertices_threshold_Z.end(), compareVerticesY);

        // // Get the vertices with certain threshold value between each Y-coordinate value
        // const float threshold_Y = 0.1f; // meters
        // std::vector<rs2::vertex> vertices_threshold_Y;
        // for (int i = 0; i < (int) vertices_threshold_Z.size(); i++) {
        //     if (vertices_threshold_Z[i].y - vertices_threshold_Z[i - 1].y < threshold_Y) {
        //         vertices_threshold_Y.push_back(vertices_threshold_Z[i]);
        //     }
        // }

        // // Sort the vertices by X-coordinate (from smallest to largest)
        // std::sort(vertices_threshold_Y.begin(), vertices_threshold_Y.end(), compareVerticesX);

        // // Get the vertices with certain threshold value between each X-coordinate value
        // const float threshold_X = 0.1f; // meters
        // std::vector<rs2::vertex> vertices_threshold_X;
        // for (int i = 0; i < (int) vertices_threshold_Y.size(); i++) {
        //     if (vertices_threshold_Y[i].x - vertices_threshold_Y[i - 1].x < threshold_X) {
        //         vertices_threshold_X.push_back(vertices_threshold_Y[i]);
        //     }
        // }
               
        // // Get average value of the X, Y, and Z coordinates
        // float x = 0.0f;
        // float y = 0.0f;
        // float z = 0.0f;
        // for (int i = 0; i < (int) vertices_threshold_X.size(); i++) {
        //     x += vertices_threshold_X[i].x;
        //     y += vertices_threshold_X[i].y;
        //     z += vertices_threshold_X[i].z;
        // }
        // x /= (int) vertices_threshold_X.size();
        // y /= (int) vertices_threshold_X.size();
        // z /= (int) vertices_threshold_X.size();
                
        // // Make moving average of the vertices X, Y, and Z coordinates
        // float x_moving_average = 0.0f;
        // float y_moving_average = 0.0f;
        // float z_moving_average = 0.0f;
        // static int num_frames = 0;
        // static float x_sum = 0.0f;
        // static float y_sum = 0.0f;
        // static float z_sum = 0.0f;
        // if (num_frames < 10) {
        //     x_sum += x;
        //     y_sum += y;
        //     z_sum += z;
        //     num_frames++;
        // }
        // else {
        //     x_sum -= x_moving_average;
        //     y_sum -= y_moving_average;
        //     z_sum -= z_moving_average;
        //     x_sum += x;
        //     y_sum += y;
        //     z_sum += z;
        //     x_moving_average = x_sum / 10;
        //     y_moving_average = y_sum / 10;
        //     z_moving_average = z_sum / 10;
        // }


        // // Print the average X, Y, and Z coordinates
        // std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;


        // Convert the RealSense depth frame to an OpenCV Mat
        cv::Mat image(cv::Size(frame_width, frame_height), CV_8UC3, (void*)colorized_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the OpenCV Mat to a QImage
        QImage qImage((uchar*)image.data, image.cols, image.rows, QImage::Format_RGB888);

        // Scale and display the QImage in the QLabel
        QPixmap pixmap = QPixmap::fromImage(qImage);
        depth_frame_label.setPixmap(pixmap.scaled(depth_frame_label.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        depth_frame_label.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        plane_coefficients.clear();

        // Update the Qt GUI
        application.processEvents(); });

    timer.start((int)1000 / (2 * frame_rate)); // Update the GUI at around 30fps (1000ms/30 ≈ 33ms)

    return application.exec();
}