#include <ros/ros.h>
#include <race/drive_values.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#define RAD2DEG(x) (x*(180./M_PI))
using namespace std;

class traffic_cone_race{
private:
  ros::Subscriber cone_pos_;
  ros::Subscriber drum_cone_condition_;
  ros::Publisher car_steer_;
  ros::Publisher u_turn_condition_;
  ros::Publisher way_point_marker_;
  ros::Publisher drive_value_;

  int throttle_;
  float dis, drive_ratio_;
  bool drum_con_;
  bool count_bool_=false;
  bool count_bool1_=false;
  //set<pair <float, float>> closest_traffic_cone_;
  vector<pair<float, float>> closest_traffic_cone_;
  vector<pair<float, float>> way_point_;

  void cone_callback(const visualization_msgs::MarkerArray &cone_boundingbox);
  void drum_cone_callback(const std_msgs::Bool &drum_cone_condition);

public:
  traffic_cone_race(ros::NodeHandle &nh);
  ~traffic_cone_race();
  void Spin();
};
