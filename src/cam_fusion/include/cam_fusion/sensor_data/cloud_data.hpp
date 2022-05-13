#ifndef CAM_FUSION_SENSOR_DATA_CLOUD_DATA_HPP_
#define CAM_FUSION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cam_fusion {
class CloudData {
public:
    // using POINT = pcl::PointXYZ;
    // using CLOUD = pcl::PointCloud<POINT>;
    // using CLOUD_PTR = CLOUD::Ptr;

public:
    CloudData()
        : cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()) {
    }

public:
    double time = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
} ;

}

#endif