#include <iostream>
#include <chrono>
#include <thread>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

const float max_distance = 2.50f; // meters
const float min_distance = 0.15f; // meters

int main()
{
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    int width = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().width();
    int height = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().height();

    rs2::depth_sensor depth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0); // 0 = false, 1 = true
    depth_sensor.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);

    rs2::threshold_filter threshold_filter;
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);

    while (true)
    {

        rs2::frameset frames = pipe.wait_for_frames();

        rs2::frame depth_frame = threshold_filter.process(frames.get_depth_frame()).apply_filter(color_map);

        cv::Mat depth_image(cv::Size(width, height), CV_8UC3, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat grey_image;
        cv::cvtColor(depth_image, grey_image, cv::COLOR_RGB2GRAY);

        // cv::Mat mask1;
        // double thresholdValue1 = 2;
        // cv::threshold(grey_image, mask1, thresholdValue1, 10, cv::THRESH_BINARY_INV);
        // cv::inpaint(depth_image, mask1, depth_image, 1, cv::INPAINT_NS);
        
        cv::Mat mask2;
        double thresholdValue2 = 2;
        cv::threshold(grey_image, mask2, thresholdValue2, 255, cv::THRESH_BINARY_INV);
        cv::inpaint(grey_image, mask2, grey_image, 1, cv::INPAINT_NS);

        cv::Mat edges;
        cv::Canny(grey_image, edges, 20, 250);
        
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1);

        // for (int i = 0; i < (int) contours.size(); i++)
        // {
        //     double area = cv::contourArea(contours[i]);
        //     double perimeter = cv::arcLength(contours[i], true);

        //     std::vector<cv::Point> approx;
        //     cv::approxPolyDP(contours[i], approx, 0.04 * perimeter, true);

        //     if (cv::isContourConvex(contours[i])) //    area> 100 && area <= 10000 && 
        //     {
        //         cv::drawContours(grey_image, contours, i, cv::Scalar(255, 255, 255), 2);
        //     }
        // }
        cv::drawContours(depth_image, contours, -1, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Canny Image", edges);
        cv::imshow("Depth Image", grey_image);
        

        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

    return 0;
}