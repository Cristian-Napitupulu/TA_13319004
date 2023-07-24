#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <QApplication>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>

const int frame_width = 640;  // pixels
const int frame_height = 480; // pixels
const int frame_rate = 30;    // fps

const float max_distance = 2.50f; // meters
const float min_distance = 0.15f; // meters

int main(int argc, char *argv[])
{

    QApplication application(argc, argv);
    QWidget window;
    window.setMinimumSize(frame_width, frame_height);

    QLabel label(&window);
    label.setAlignment(Qt::AlignCenter);
    // label.setFixedSize(frame_width, frame_height);

    QVBoxLayout layout;
    layout.addWidget(&label);
    window.setLayout(&layout);

    window.show();

    rs2::pipeline pipe;
    rs2::config configuration;

    configuration.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, frame_rate);
    pipe.start(configuration);

    rs2::depth_sensor depth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0);
    depth_sensor.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);

    rs2::threshold_filter threshold_filter;
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);

    rs2::colorizer colorizer;
    colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 2);

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
                     {
        // Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame depth_frame = frames.get_depth_frame();
        rs2::frame filtered_frame = threshold_filter.process(depth_frame);
        rs2::frame colorized_frame = filtered_frame.apply_filter(colorizer);

        // Convert the RealSense depth frame to an OpenCV Mat
        cv::Mat image(cv::Size(frame_width, frame_height), CV_8UC3, (void*)colorized_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the OpenCV Mat to a QImage
        QImage qImage((uchar*)image.data, image.cols, image.rows, QImage::Format_RGB888);

        // Scale and display the QImage in the QLabel
        QPixmap pixmap = QPixmap::fromImage(qImage);
        label.setPixmap(pixmap.scaled(label.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        label.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);


        // Update the Qt GUI
        application.processEvents(); });

    timer.start((int)1000 / frame_rate); // Update the GUI at around 30fps (1000ms/30 ≈ 33ms)

    return application.exec();
}