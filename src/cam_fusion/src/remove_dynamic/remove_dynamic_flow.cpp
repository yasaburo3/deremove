#include <cam_fusion/remove_dynamic/remove_dynamic_flow.hpp>

namespace cam_fusion{
    RemoveDynamicFlow::RemoveDynamicFlow(ros::NodeHandle& nh, 
                      std::string& cloud_input_topic,
                      std::string& cloud_output_topic,
                      std::string& image_input_topic, 
                      std::string& image_output_topic)
        : nh_(nh) {
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, cloud_input_topic, 100000);
        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh_, cloud_output_topic, "/map", 100);
        image_converter_ptr_ = std::make_shared<ImageConverter>(nh_, image_input_topic, image_output_topic);        // 接收语义
        cloud_data_buff_.clear();

        loadCalibration();
    }

    bool RemoveDynamicFlow::loadCalibration() {
        P_rect_00 = std::make_shared<cv::Mat>(3, 4, cv::DataType<double>::type);
        R_rect_00 = std::make_shared<cv::Mat>(4, 4, cv::DataType<double>::type);
        RT = std::make_shared<cv::Mat>(4, 4, cv::DataType<double>::type);

        RT->at<double>(0,0) = 7.967514e-03; RT->at<double>(0,1) = -9.999679e-01; RT->at<double>(0,2) = -8.462264e-04; RT->at<double>(0,3) = -1.377769e-02;
        RT->at<double>(1,0) = -2.771053e-03;RT->at<double>(1,1) = 8.241710e-04;  RT->at<double>(1,2) = -9.999958e-01; RT->at<double>(1,3) = -5.542117e-02;
        RT->at<double>(2,0) = 9.999644e-01; RT->at<double>(2,1) = 7.969825e-03;  RT->at<double>(2,2) = -2.764397e-03; RT->at<double>(2,3) = -2.918589e-01;
        RT->at<double>(3,0) = 0.0; RT->at<double>(3,1) = 0.0; RT->at<double>(3,2) = 0.0; RT->at<double>(3,3) = 1.0;

        R_rect_00->at<double>(0,0) = 9.999191e-01; R_rect_00->at<double>(0,1) = 1.228161e-02; R_rect_00->at<double>(0,2) = -3.316013e-03; R_rect_00->at<double>(0,3) = 0.0;
        R_rect_00->at<double>(1,0) = -1.228209e-02; R_rect_00->at<double>(1,1) = 9.999246e-01; R_rect_00->at<double>(1,2) = -1.245511e-04; R_rect_00->at<double>(1,3) = 0.0;
        R_rect_00->at<double>(2,0) = 3.314233e-03; R_rect_00->at<double>(2,1) = 1.652686e-04; R_rect_00->at<double>(2,2) = 9.999945e-01; R_rect_00->at<double>(2,3) = 0.0;
        R_rect_00->at<double>(3,0) = 0.0; R_rect_00->at<double>(3,1) = 0.0; R_rect_00->at<double>(3,2) = 0.0; R_rect_00->at<double>(3,3) = 1.0;
        
        P_rect_00->at<double>(0,0) = 7.188560e+02; P_rect_00->at<double>(0,1) = 0.000000e+00; P_rect_00->at<double>(0,2) = 6.071928e+02; P_rect_00->at<double>(0,3) = 4.538225e+01;
        P_rect_00->at<double>(1,0) = 0.000000e+00; P_rect_00->at<double>(1,1) = 7.188560e+02; P_rect_00->at<double>(1,2) = 1.852157e+02; P_rect_00->at<double>(1,3) = -1.130887e-01;
        P_rect_00->at<double>(2,0) = 0.000000e+00; P_rect_00->at<double>(2,1) = 0.000000e+00; P_rect_00->at<double>(2,2) = 1.000000e+00; P_rect_00->at<double>(2,3) = 3.779761e-03;

    }

    void RemoveDynamicFlow::Process() {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in_view_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        cv::Mat X(4,1,cv::DataType<double>::type);
        cv::Mat Y(3,1,cv::DataType<double>::type);

        int num_after = 0;
        int num_in_view = 0;
        int num_delete = 0;

        double time = current_cloud_data_.time;

        for(int idx = 0; idx < current_cloud_data_.cloud_ptr->size(); idx++) {
            pcl::PointXYZRGB pt;
            pt.x = current_cloud_data_.cloud_ptr->points[idx].x;
            pt.y = current_cloud_data_.cloud_ptr->points[idx].y;
            pt.z = current_cloud_data_.cloud_ptr->points[idx].z;

            if(pt.x <= 0.0) {
                continue;
            }

            // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
            X.at<double>(0,0) = pt.x;
            X.at<double>(1,0) = pt.y;
            X.at<double>(2,0) = pt.z;
            X.at<double>(3,0) = 1;

            // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera. 
            // Store the result in Y.
            Y = (*P_rect_00) * (*R_rect_00) * (*RT) * X;  //坐标转换

            // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
            cv::Point p;
            p.x = Y.at<double>(0,0) / Y.at<double>(0,2);
            p.y = Y.at<double>(1,0) / Y.at<double>(0,2);

            // 在视野范围内
            if(p.x > 0 && p.x < current_image_data_.image.cols && p.y >= 0 && p.y < current_image_data_.image.rows) {
                if(current_image_data_.image.at<uchar>(p.y, p.x) == 0 || pt.z < -1.4) {
                    pt.b = 255;
                    pt.g = 0;
                    pt.r = 0;
                    // point_cloud_after->push_back(pt);
                    num_after++;
                }
                else{
                    pt.b = 0;
                    pt.g = 0;
                    pt.r = 255;
                    // point_cloud_delete->push_back(pt);
                    num_delete++;
                }

                point_cloud_in_view_ptr_->push_back(pt);
                num_in_view++;
            
            }

        }

        cloud_pub_ptr_->Publish(point_cloud_in_view_ptr_, time);

    }

    bool RemoveDynamicFlow::Run() {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        image_converter_ptr_->ParseData(image_data_buff_);

        if(!cloud_data_buff_.empty() && !image_data_buff_.empty()) {

            while(cloud_data_buff_.front().time < image_data_buff_.front().header.stamp.toSec()) {
                cloud_data_buff_.pop_front();
            }

            if(cloud_data_buff_.front().time > image_data_buff_.front().header.stamp.toSec() + 0.1) {
                image_data_buff_.pop_front();
            }
            else{
                current_image_data_ = image_data_buff_.front();
                current_cloud_data_ = cloud_data_buff_.front();

                cloud_data_buff_.pop_front();
                image_data_buff_.pop_front();
                std::cout << std::setprecision(16);
                std::cout << "Img time: " << current_image_data_.header.stamp.toSec() << ", Cloud time: " << current_cloud_data_.time << std::endl;

                Process();
            }   

        }

        // CloudData cloud_data;
        // while(!cloud_data_buff_.empty()) {
        //     cloud_data = cloud_data_buff_.front();
        //     cloud_data_buff_.pop_front();
        //     sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        //     pcl::toROSMsg(*cloud_data.cloud_ptr, *cloud_ptr_output);

        //     cloud_pub_ptr_->Publish(cloud_data.cloud_ptr, cloud_data.time);
        
        // }
    }


}