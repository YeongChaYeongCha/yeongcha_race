#include <ros/ros.h>

#define PCL_NO_PRECOMPILE

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <std_msgs/Bool.h>

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

namespace euclidean_cluster
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

#define SLRPointXYZIRL euclidean_cluster::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

POINT_CLOUD_REGISTER_POINT_STRUCT(euclidean_cluster::PointXYZIRL, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label, label))

class EuclideanCluster
{

private:
  ros::Subscriber sub_remove_plane_;
  ros::Publisher pub_bounding_box_;
  ros::Publisher pub_drum_condition_;
  ros::Publisher pub_drum_bounding_box_;
  ros::Publisher pub_bounding_left_box_;
  ros::Publisher pub_bounding_right_box_;

  int min_cluster_size_, max_cluster_size_;
  float cluster_tolerance_;
  bool drum_con_=false;

  pcl::PointCloud<VPoint>::Ptr raw_pointcloud;

  void cloud_cluster_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

public:
  EuclideanCluster(ros::NodeHandle &nh);
  ~EuclideanCluster();
  void Spin();
};
