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
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_pointcloud{
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

namespace velodyne_filter_add
{
struct PointXYZIRL
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  uint16_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

};

#define SLRPointXYZIRL velodyne_filter_add::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_filter_add::PointXYZIRL, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label, label))

class VelodyneFilterAdd
{

private:
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_filtered_point_;
  ros::Publisher pub_plane_;

  float voxel_size_, std_multiplier_, distance_threshold_, search_radius_, upsampling_radius_, upsampling_step_size_;
  int num_neigbor_points_, max_iterations_;

  //pcl::PointCloud<SLRPointXYZIRL>::Ptr all_pointcloud;
  pcl::PointCloud<VPoint>::Ptr voxel_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr sor_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr raw_pointcloud;
  pcl::PointCloud<VPoint>::Ptr cropbox_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr inlierPoints;
  pcl::PointCloud<VPoint>::Ptr inlierPoints_neg;
  pcl::PointCloud<VPoint>::Ptr upsampling_pointcloud;
  pcl::PointCloud<VPoint>::Ptr ptr_transformed;

  void cloud_sor(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_voxel(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void cloud_cropbox(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_remove_plane(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr plane, const pcl::PointCloud<VPoint>::Ptr out_without_plane);
  void cloud_upsampling(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out);

public:
  VelodyneFilterAdd(ros::NodeHandle &nh);
  ~VelodyneFilterAdd();
  void Spin();
};
