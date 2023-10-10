#include <traffic_cone_race.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "u_turn_node");
  ros::NodeHandle nh("~");

  traffic_cone_race core(nh);

  return 0;
}



