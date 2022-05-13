#ifndef CAM_FUSION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define CAM_FUSION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include "cam_fusion/sensor_data/cloud_data.hpp"

namespace cam_fusion {
class CloudPublisher {
public:
    CloudPublisher(ros::NodeHandle& nh, 
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    CloudPublisher() = default;

    void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr_input, double time);
    void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr_input);

    void Publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ptr_input, double time);
    void Publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ptr_input);

    bool HasSubscribers();

private:
    void PublishData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr_input, ros::Time time);
    void PublishData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ptr_input, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

};
}

#endif