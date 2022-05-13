#ifndef CAM_FUSION_REMOVE_DYNAMIC_FLOW_HPP_
#define CAM_FUSION_REMOVE_DYNAMIC_FLOW_HPP_

#include <ros/ros.h>

// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "cam_fusion/publisher/cloud_publisher.hpp"
#include "cam_fusion/subscriber/cloud_subscriber.hpp"
#include "cam_fusion/tools/image_converter.hpp"



namespace cam_fusion {
class RemoveDynamicFlow {
public:
    RemoveDynamicFlow(ros::NodeHandle& nh, 
                      std::string& cloud_input_topic,
                      std::string& cloud_output_topic,
                      std::string& image_input_topic, 
                      std::string& image_output_topic);
    bool Run();

private:

    bool loadCalibration();

    void Process();

private:
    ros::NodeHandle nh_;
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<ImageConverter> image_converter_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    CloudData current_cloud_data_;

    std::deque<cv_bridge::CvImage> image_data_buff_;
    cv_bridge::CvImage current_image_data_;     // mask

    std::shared_ptr<cv::Mat> P_rect_00;
    std::shared_ptr<cv::Mat> R_rect_00;
    std::shared_ptr<cv::Mat> RT;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in_view_ptr_;
    
};


}

#endif