#include <traffic_cone_race.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "traffic_cone_race_node");
  ros::NodeHandle nh("~");

  traffic_cone_race core(nh);

  return 0;
}
