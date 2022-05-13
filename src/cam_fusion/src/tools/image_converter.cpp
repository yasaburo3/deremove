#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cam_fusion/tools/image_converter.hpp"

namespace cam_fusion {
    ImageConverter::ImageConverter(ros::NodeHandle& nh,
                   const std::string& topic_input_name,
                   const std::string& topic_output_name)
        : it_(nh)
    {
        image_sub_ = it_.subscribe(topic_input_name, 1,
            &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise(topic_output_name, 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ImageConverter::~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg) {
        buff_mutex_.lock();
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch(cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        new_image_data_.push_back(*cv_ptr);


        // Draw an example circle on the video stream
        // if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) 
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        // cv::waitKey(0);

        // Output modified video stream
        // image_pub_.publish(cv_ptr->toImageMsg());

        buff_mutex_.unlock();
    }

    void ImageConverter::ParseData(std::deque<cv_bridge::CvImage>& image_data_buff) {
        buff_mutex_.lock();

        if(new_image_data_.size() > 0) {
            image_data_buff.insert(image_data_buff.end(), new_image_data_.begin(), new_image_data_.end());
            new_image_data_.clear();
        }

        buff_mutex_.unlock();
    }


}

