#include <ros/ros.h>

#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_pointcloud
{
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}; //namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

namespace velodyne_filter
{
struct PointXYZIRL
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  uint16_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

};  //namespace velodyne_filter

#define SLRPointXYZIRL velodyne_filter::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_filter::PointXYZIRL, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label, label))

class VelodyneFilter
{

private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_filtered_point_;

  float voxel_size_, std_multiplier_;
  int num_neigbor_points_;

  pcl::PointCloud<SLRPointXYZIRL>::Ptr all_pointcloud;
  pcl::PointCloud<VPoint>::Ptr voxel_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr sor_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr raw_pointcloud;
  pcl::PointCloud<VPoint>::Ptr cropbox_filtered_pointcloud;

  void cloud_sor(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_voxel(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void cloud_cropbox(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);

public:
  VelodyneFilter(ros::NodeHandle &nh);
  ~VelodyneFilter();
  void Spin();
};
