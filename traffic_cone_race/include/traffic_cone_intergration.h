#include <ros/ros.h>

#define PCL_NO_PRECOMPILE

#include <algorithm>
#include <vector>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <race/drive_values.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#define RAD2DEG(x) (x*(180./M_PI))
using namespace std;

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

#define VPoint velodyne_pointcloud::PointXYZIR

class TrafficConeRace
{

private:
  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_filtered_point_;
  ros::Publisher pub_plane_;
  ros::Publisher pub_bounding_box_;
  ros::Publisher way_point_marker_;
  ros::Publisher way_point_angle_;
  ros::Publisher drive_value_;

  float voxel_size_, std_multiplier_, distance_threshold_, cluster_tolerance_;
  int num_neigbor_points_, max_iterations_, min_cluster_size_, max_cluster_size_;

  vector<pair<float, float>> closest_traffic_cone_;
  vector<pair<float, float>> way_point_;

  pcl::PointCloud<VPoint>::Ptr voxel_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr sor_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr raw_pointcloud;
  pcl::PointCloud<VPoint>::Ptr cropbox_filtered_pointcloud;
  pcl::PointCloud<VPoint>::Ptr inlierPoints;
  pcl::PointCloud<VPoint>::Ptr inlierPoints_neg;
  pcl::PointCloud<VPoint>::Ptr ptr_transformed;

  void cloud_sor(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_voxel(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void cloud_cropbox(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out);
  void cloud_remove_plane(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr plane, const pcl::PointCloud<VPoint>::Ptr out_without_plane);
  void cloud_cluster_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

public:
  TrafficConeRace(ros::NodeHandle &nh);
  ~TrafficConeRace();
  void Spin();
};
