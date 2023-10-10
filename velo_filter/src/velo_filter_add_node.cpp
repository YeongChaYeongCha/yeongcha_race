#include "velo_filter_add.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "velo_filter_add");

	ros::NodeHandle nh("~");

	VelodyneFilterAdd core(nh);
	return 0;
}

