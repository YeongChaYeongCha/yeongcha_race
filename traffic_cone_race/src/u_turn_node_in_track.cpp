#include <traffic_cone_race_u_turn.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "u_turn_node_in_track");
  ros::NodeHandle nh("~");

  traffic_cone_race core(nh);

  return 0;
}




