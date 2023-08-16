#include "../include/pointcloud_gantry/realsense_connector.h"
#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <QApplication>
#include <QMainWindow>

// REALSENSE_DEVICE LIDAR_L515(0, 640, 480, 30, 0.15f, 2.00f);

int main(int argc, char **argv)
{
    std::cout << "Pointcloud Gantry" << std::endl;
    // LIDAR_L515.initialize();
    // // Create a PCL Visualizer object
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    // // // Add the point cloud to the viewer
    // // viewer->addPointCloud<pcl::PointXYZ>(LIDAR_L515.pointcloud(), "cloud");

    // while (true)
    // {
    //     rs2::frame depth_frame = LIDAR_L515.get_depth_frame();
    //     LIDAR_L515.calculate_pointcloud(depth_frame);

    //     // // Print X, Y, Z value
    //     // for (int i = 0; i < (int)LIDAR_L515.pointcloud()->size(); i++)
    //     // {
    //     //     std::cout << "X: " << LIDAR_L515.pointcloud()->points[i].x << " Y: " << LIDAR_L515.pointcloud()->points[i].y << " Z: " << LIDAR_L515.pointcloud()->points[i].z << std::endl;
    //     // }

    //     // Print point cloud size
    //     std::cout << "Point cloud size: " << LIDAR_L515.pointcloud()->size() << std::endl;

    //     // rs2::frame colorized_frame = LIDAR_L515.get_colorized_depth_frame(depth_frame);
    //     // cv::Mat image(cv::Size(LIDAR_L515.get_frame_width(), LIDAR_L515.get_frame_height()), CV_8UC3, (void *)colorized_frame.get_data(), cv::Mat::AUTO_STEP);
    //     // cv::imshow("Lalala", image);

    //     // if (cv::waitKey(1) == 'q')
    //     // {
    //     //     break;
    //     // }

    //     // // Optionally, set the point size for better visibility
    //     // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    //     // // Run the visualization loop (this will display the point cloud until the window is closed)
    //     // while (!viewer->wasStopped())
    //     // {
    //     //     viewer->spinOnce(100);
    //     //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     // }
    //     // viewer->spinOnce(100);
    //     // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Start the pipeline
    pipe.start(cfg);

    QApplication app(argc, argv);
    QMainWindow mainWindow;
    mainWindow.setGeometry(100, 100, 800, 600);

    // Enter the main loop to process frames
    while (true) //! viewer->wasStopped())
    {
        // Wait for the next frame from the camera
        std::cout << "Pointcloud Gantry 0" << std::endl;
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame depth_frame = frames.get_depth_frame();

        // Convert the RealSense depth frame to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth_frame);
        cloud->width = depth_frame.as<rs2::video_frame>().get_width();
        cloud->height = depth_frame.as<rs2::video_frame>().get_height();
        cloud->is_dense = true;
        cloud->points.resize(cloud->width * cloud->height);

        std::cout << "Pointcloud Gantry 1" << std::endl;
        auto vertices = points.get_vertices();
        for (int i = 0; i < (int)cloud->points.size(); ++i)
        {
            cloud->points[i].x = vertices[i].x;
            cloud->points[i].y = vertices[i].y;
            cloud->points[i].z = vertices[i].z;
        }

        // Create PCL visualizer
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

        viewer->setBackgroundColor(0.0, 0.0, 0.0);
        std::cout << "Pointcloud Gantry 2" << std::endl;

        // Update the point cloud in the visualizer
        viewer->removeAllPointClouds();

        std::cout << "Pointcloud Gantry 3" << std::endl;
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "point_cloud");

        std::cout << "Pointcloud Gantry 4" << std::endl;
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point_cloud");
        // viewer->addCoordinateSystem(0.1);
        std::cout << "Pointcloud Gantry 5" << std::endl;
        // Change the camera position
        viewer->setCameraPosition(0, 0, -2.0, 0, -1, 0);
        // Show the origin of the coordinate system
        std::cout << "Pointcloud Gantry 6" << std::endl;
        viewer->spinOnce();
        std::cout << "Pointcloud Gantry 7" << std::endl;

        // // Set up Qt/VTK interactor
        // vtkSmartPointer<vtkRenderWindow> renderWindow = viewer->getRenderWindow();
        // // QVTKWidget *qvtkWidget = new QVTKWidget(&mainWindow);
        // // qvtkWidget->SetRenderWindow(renderWindow);
        // // mainWindow.setCentralWidget(qvtkWidget);

        // mainWindow.show();
        // app.exec();
    }

    return 0;
}