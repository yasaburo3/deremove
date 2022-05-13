#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// load calib params
void loadCalibration(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat & RT) {
    RT.at<double>(0,0) = 7.967514e-03; RT.at<double>(0,1) = -9.999679e-01; RT.at<double>(0,2) = -8.462264e-04; RT.at<double>(0,3) = -1.377769e-02;
    RT.at<double>(1,0) = -2.771053e-03;RT.at<double>(1,1) = 8.241710e-04;  RT.at<double>(1,2) = -9.999958e-01; RT.at<double>(1,3) = -5.542117e-02;
    RT.at<double>(2,0) = 9.999644e-01; RT.at<double>(2,1) = 7.969825e-03;  RT.at<double>(2,2) = -2.764397e-03; RT.at<double>(2,3) = -2.918589e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;

    R_rect_00.at<double>(0,0) = 9.999191e-01; R_rect_00.at<double>(0,1) = 1.228161e-02; R_rect_00.at<double>(0,2) = -3.316013e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -1.228209e-02; R_rect_00.at<double>(1,1) = 9.999246e-01; R_rect_00.at<double>(1,2) = -1.245511e-04; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 3.314233e-03; R_rect_00.at<double>(2,1) = 1.652686e-04; R_rect_00.at<double>(2,2) = 9.999945e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0.0; R_rect_00.at<double>(3,1) = 0.0; R_rect_00.at<double>(3,2) = 0.0; R_rect_00.at<double>(3,3) = 1.0;
    
    P_rect_00.at<double>(0,0) = 7.188560e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.071928e+02; P_rect_00.at<double>(0,3) = 4.538225e+01;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.188560e+02; P_rect_00.at<double>(1,2) = 1.852157e+02; P_rect_00.at<double>(1,3) = -1.130887e-01;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 3.779761e-03;


// R_rect_02: 9.999191e-01 1.228161e-02 -3.316013e-03 -1.228209e-02 9.999246e-01 -1.245511e-04 3.314233e-03 1.652686e-04 9.999945e-01
// P_rect_02: 7.188560e+02 0.000000e+00 6.071928e+02 4.538225e+01 0.000000e+00 7.188560e+02 1.852157e+02 -1.130887e-01 0.000000e+00 0.000000e+00 1.000000e+00 3.779761e-03
}

void projectLidarToCam0() {
    // load image from file
    cv::Mat img = cv::imread("../data/0000000000.png");

    // load lidar points from file
    std::string infile = "../data/0000000000.bin";
    std::string outfile = "../output/0000000000.pcd";
    std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()) {
        std::cerr << "Could not read file: " << infile << std::endl;
        exit(EXIT_FAILURE);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for(i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char*)&point.x, 3*sizeof(float));
        input.read((char*)&point.intensity, sizeof(float));
        point_cloud->push_back(point);
    }
    input.close();

    std::cout << "Read KITTI point cloud with " << i << " points, writting to " << outfile << std::endl;

    pcl::io::savePCDFile(outfile, *point_cloud);

    // load mask from file
    cv::Mat mask = cv::imread("../data/mask.png", 0);
    // cv::imshow("mask", mask);
    // cv::waitKey(0);

    // point cloud after removing dynamic object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_after(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in_view(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_delete(new pcl::PointCloud<pcl::PointXYZRGB>);

    // store calibration 
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type);
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type);
    cv::Mat RT(4, 4, cv::DataType<double>::type);
    loadCalibration(P_rect_00, R_rect_00, RT);

    // project lidar points
    cv::Mat visImg = img.clone();
    cv::Mat overlay = visImg.clone();

    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y(3,1,cv::DataType<double>::type);
    
    int num_after = 0;
    int num_in_view = 0;
    int num_delete = 0;

    for(int idx = 0; idx < point_cloud->points.size(); idx++) {
        pcl::PointXYZRGB pt;
        pt.x = point_cloud->points[idx].x;
        pt.y = point_cloud->points[idx].y;
        pt.z = point_cloud->points[idx].z;
        
        float maxX = 25.0, maxY = 6.0, minZ = -1.4;
        // if(pt.x > maxX || pt.x < 0.0 || abs(pt.y) > maxY || pt.z < minZ || pt.intensity < 0.01) {
        //     continue;
        // }
        if(pt.x < 0.0) {
            continue;
        }

        // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
        X.at<double>(0,0) = pt.x;
        X.at<double>(1,0) = pt.y;
        X.at<double>(2,0) = pt.z;
        X.at<double>(3,0) = 1;

        // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera. 
        // Store the result in Y.
        Y = P_rect_00 * R_rect_00 * RT * X;  //坐标转换

        // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
        cv::Point p;
        p.x = Y.at<double>(0,0) / Y.at<double>(0,2);
        p.y = Y.at<double>(1,0) / Y.at<double>(0,2);

        
        

        float val = pt.x;
        float maxVal = 20.0;
        int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
        int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
        cv::circle(overlay, p, 1, cv::Scalar(0, green, red), -1);  //设置投影点在图像平面overlay上的位置，大小，颜色

        // 在视野范围内
        if(p.x >= 0 && p.x < mask.cols && p.y >= 0 && p.y < mask.rows) {
            
            if(mask.at<uchar>(p.y, p.x) == 0 || pt.z < -1.4) {
                pt.b = 255;
                pt.g = 0;
                pt.r = 0;
                point_cloud_after->push_back(pt);
                num_after++;
            }
            else{
                pt.b = 0;
                pt.g = 0;
                pt.r = 255;
                point_cloud_delete->push_back(pt);
                num_delete++;
            }
            
            point_cloud_in_view->push_back(pt);
            num_in_view++;
        }
        // point_cloud_after->push_back(pt);
        // num_after++;

    }
    std::string after_file = "../output/after.pcd";
    std::cout << num_after << " points, writting to " << after_file << std::endl;
    pcl::io::savePCDFile(after_file, *point_cloud_after);

    std::string in_view_file = "../output/in_view.pcd";
    std::cout << num_in_view << " points, writting to " << after_file << std::endl;
    pcl::io::savePCDFile(in_view_file, *point_cloud_in_view);

    std::string delete_file = "../output/delete.pcd";
    std::cout << num_in_view << " points, writting to " << after_file << std::endl;
    pcl::io::savePCDFile(delete_file, *point_cloud_delete);

    float opacity = 0.6;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

    // std::string windowName = "LiDAR data on image overlay";
    // cv::namedWindow( windowName, 3 );
    // cv::imshow( windowName, visImg );
    // cv::waitKey(0); // wait for key to be pressed

}

int main() {
    projectLidarToCam0();
}