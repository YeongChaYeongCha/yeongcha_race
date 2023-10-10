#include "euclidean_cluster.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "euclidean_cluster");

	ros::NodeHandle nh("~");

	EuclideanCluster core(nh);
	return 0;
}

