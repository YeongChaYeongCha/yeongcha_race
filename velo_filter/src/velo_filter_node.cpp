#include "velo_filter.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "velo_filter");

	ros::NodeHandle nh("~");

	VelodyneFilter core(nh);
	return 0;
}
