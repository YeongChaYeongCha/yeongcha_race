#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <race/drive_values.h>
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point.h>
#define RAD2DEG(x) (x*(180./M_PI))
using namespace std;

class traffic_cone_race{
private:
  ros::Subscriber cone_pos_;
  ros::Subscriber cone_left_pos_;
  ros::Subscriber cone_right_pos_;
  ros::Publisher car_steer_;
  ros::Publisher u_turn_condition_;
  ros::Publisher way_point_marker_;
  ros::Publisher way_point_angle_;
  ros::Publisher drive_value_;

  int throttle_, max_speed_, min_speed_;
  float dis, drive_ratio_, curve_constant_;
  //set<pair <float, float>> closest_traffic_cone_;
  vector<pair<float, float>> closest_traffic_cone_;
  vector<pair<float, float>> right_curve_traffic_cone_;
  vector<pair<float, float>> left_curve_traffic_cone_;
  vector<pair<float, float>> left_traffic_cone_;
  vector<pair<float, float>> right_traffic_cone_;
  vector<pair<float, float>> way_point_;

  void cone_callback(const visualization_msgs::MarkerArray &cone_boundingbox);
  void left_cone_callback(const visualization_msgs::MarkerArray &left_cone_boundingbox);
  void right_cone_callback(const visualization_msgs::MarkerArray &right_cone_boundingbox);
  void process_data();

public:
  traffic_cone_race(ros::NodeHandle &nh);
  ~traffic_cone_race();
  void Spin();
};
