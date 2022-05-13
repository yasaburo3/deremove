#ifndef CAM_FUSION_TOOLS_IMAGE_CONVERTER_HPP_
#define CAM_FUSION_TOOLS_IMAGE_CONVERTER_HPP_

#include <deque>
#include <mutex>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cam_fusion {

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
public:
    ImageConverter(ros::NodeHandle& nh,
                   const std::string& topic_input_name,
                   const std::string& topic_output_name);


    ~ImageConverter();

    void ParseData(std::deque<cv_bridge::CvImage>& deque_image_data);

private:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    std::deque<cv_bridge::CvImage> new_image_data_;

    std::mutex buff_mutex_;

};
}

#endif