#include <traffic_cone_intergration.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "traffic_cone_race_node");
  ros::NodeHandle nh("~");

  TrafficConeRace core(nh);

  return 0;
}

