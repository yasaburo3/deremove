#include <ros/ros.h>

#include <memory>

#include "cam_fusion/remove_dynamic/remove_dynamic_flow.hpp"

using namespace cam_fusion;

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    std::string cloud_input_topic;
    std::string cloud_output_topic;
    std::string image_input_topic;
    std::string image_output_topic;
    nh.param<std::string>("cloud_input_topic", cloud_input_topic, "/kitti/velo/pointcloud");
    nh.param<std::string>("cloud_output_topic", cloud_output_topic, "/point_cloud_without_dynamic_object");
    nh.param<std::string>("image_input_topic", image_input_topic, "/segmentation_output");
    nh.param<std::string>("image_output_topic", image_output_topic, "/image_projected");

    std::shared_ptr<RemoveDynamicFlow> remove_dynamic_flow_ptr = std::make_shared<RemoveDynamicFlow>(nh, cloud_input_topic, cloud_output_topic,
            image_input_topic, image_output_topic);

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();

        remove_dynamic_flow_ptr->Run();

        rate.sleep();
    }

    return 0;

}