#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

class RealsenseCamera
{
public:
    RealsenseCamera(ros::NodeHandle nh);
    void run();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    rs2::pipeline pipeline_;
    rs2::colorizer color_map;
    rs2::config cfg_;
    rs2::frameset frameset_;
    rs2::frame depth_frame_;
    cv::Mat color_image_;
};

RealsenseCamera::RealsenseCamera(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
{
    // set kamera
    cfg_.enable_stream(RS2_STREAM_DEPTH, 640 , 480, RS2_FORMAT_Z16);
    pipeline_.start(cfg_);

    // set publisher
    image_pub_ = it_.advertise("camera/image", 1);
}

void RealsenseCamera::run()
{
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // memperbarui set frame kamera
        frameset_ = pipeline_.wait_for_frames();

        // ngambil warna dan convert ke cv mat
        depth_frame_ = frameset_.get_depth_frame().apply_filter(color_map);;
        

        const int w = depth_frame_.as<rs2::video_frame>().get_width();
        const int h = depth_frame_.as<rs2::video_frame>().get_height();

        color_image_ = cv::Mat(cv::Size(w,h), CV_8UC3, (void*)depth_frame_.get_data(), cv::Mat::AUTO_STEP);

        // publish gambar menjadi ros msg
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image_).toImageMsg();
        image_pub_.publish(msg);

        ros::spinOnce();
         loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_camera");
    ros::NodeHandle nh;
    RealsenseCamera camera(nh);
    camera.run();
    return 0;
}