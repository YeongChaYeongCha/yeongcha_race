#include "velo_filter_add.h"

VelodyneFilterAdd::VelodyneFilterAdd(ros::NodeHandle &nh){
	sub_point_cloud_=nh.subscribe("/velodyne_points", 10, &VelodyneFilterAdd::cloud_cb, this);
	pub_filtered_point_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 1000);
	pub_plane_=nh.advertise<sensor_msgs::PointCloud2>("/plane_point", 1000);
	
	//all_pointcloud=pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);
	voxel_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	sor_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	raw_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	cropbox_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	inlierPoints=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	inlierPoints_neg=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	upsampling_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	
	//voxel_size | num_neigbor_points | std_multiplier
	nh.getParam("voxel_size", voxel_size_);
	ROS_INFO("voxel_size_ : %f", voxel_size_);
	nh.getParam("num_neigbor_points", num_neigbor_points_);
	ROS_INFO("num_neigbor_points : %d", num_neigbor_points_);
	nh.getParam("std_multiplier", std_multiplier_);
	ROS_INFO("std_multiplier : %f", std_multiplier_);
	nh.getParam("max_iterations", max_iterations_);
	ROS_INFO("max_iterations : %d", max_iterations_);
	nh.getParam("distance_threshold", distance_threshold_);
	ROS_INFO("distance_threshold : %f", distance_threshold_);
	
	ros::spin();
}

VelodyneFilterAdd::~VelodyneFilterAdd() {
}

void VelodyneFilterAdd::Spin(){
}

void VelodyneFilterAdd::cloud_upsampling(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::MovingLeastSquares<VPoint, VPoint> up_filter;
	up_filter.setInputCloud(in);

	pcl::search::KdTree<VPoint>::Ptr kdtree(new pcl::search::KdTree<VPoint>);
	up_filter.setSearchMethod(kdtree);

	up_filter.setSearchRadius(search_radius_);
	up_filter.setUpsamplingMethod(pcl::MovingLeastSquares<VPoint, VPoint>::SAMPLE_LOCAL_PLANE);
	up_filter.setUpsamplingRadius(upsampling_radius_);
	up_filter.setUpsamplingStepSize(upsampling_step_size_);
	up_filter.process(*out);
}

void VelodyneFilterAdd::cloud_remove_plane(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr plane, const pcl::PointCloud<VPoint>::Ptr out_without_plane){
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

	pcl::SACSegmentation<VPoint> seg;
	seg.setOptimizeCoefficients(true);
	seg.setInputCloud(in);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(max_iterations_);
	seg.setDistanceThreshold(distance_threshold_);
	seg.segment(*inliers, *coefficients);

	/*
	pcl::copyPointCloud<VPoint>(*in, *inliers, *plane);
	std::cout<<"plane : "<<plane->points.size()<<std::endl;
	*/
	pcl::ExtractIndices<VPoint> extract;
	extract.setInputCloud(in);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*out_without_plane);

	pcl::ExtractIndices<VPoint> extract_plane;
	extract_plane.setInputCloud(in);
	extract_plane.setIndices(inliers);
	extract_plane.filter(*plane);
	std::cout<<"plane : "<<plane->points.size()<<std::endl;
	std::cout<<"out_without_plane : "<<out_without_plane->points.size()<<std::endl;
}

void VelodyneFilterAdd::cloud_voxel(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::VoxelGrid<VPoint> voxel_filter;
	voxel_filter.setInputCloud(in);
	voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
	voxel_filter.filter(*out);
}

void VelodyneFilterAdd::cloud_sor(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::StatisticalOutlierRemoval<VPoint> sor;
	sor.setInputCloud(in);
	sor.setMeanK(num_neigbor_points_);
	sor.setStddevMulThresh(std_multiplier_);
	sor.filter(*out);
}

void VelodyneFilterAdd::cloud_cropbox(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::CropBox<VPoint> region(false);
	region.setMin(Eigen::Vector4f(-0.5, -3.5, -1.7, 1));
	region.setMax(Eigen::Vector4f(8, 3.5, 2.5, 1));
	region.setInputCloud(in);
	region.filter(*out);
	
	std::vector<int> indices;
	pcl::CropBox<VPoint> roof(true);
	roof.setMin(Eigen::Vector4f(-2.0, -0.60, -1.2, 1));
	roof.setMax(Eigen::Vector4f(0.0, 0.60, 0.5, 1));
	roof.setInputCloud(out);
	roof.filter(indices);
	
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	for(auto& point : indices)
		inliers->indices.push_back(point);
		
	pcl::ExtractIndices<VPoint> extract;
	extract.setInputCloud(out);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*out);
}

void VelodyneFilterAdd::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr){
	// pcl::PointCloud<VPoint> laserCloudIn;
	pcl::fromROSMsg(*in_cloud_ptr, *raw_pointcloud);
	// //SLRPointXYZIRL SLRpoint;
	// VPoint V_point;
	
	// for (size_t i=0; i<laserCloudIn.points.size(); i++){
	// 	V_point.x=laserCloudIn.points[i].x;
	// 	V_point.y=laserCloudIn.points[i].y;
	// 	V_point.z=laserCloudIn.points[i].z;
	// 	V_point.intensity=laserCloudIn.points[i].intensity;
	// 	V_point.ring=laserCloudIn.points[i].ring;
	// 	raw_pointcloud->points.push_back(V_point);
	// }
	
	ROS_INFO("----------------------");
	//std::cout<<"raw_pointcloud_size : "<<raw_pointcloud->points.size()<<std::endl;
	cloud_voxel(raw_pointcloud, voxel_filtered_pointcloud);
	cloud_cropbox(voxel_filtered_pointcloud, cropbox_filtered_pointcloud);
	//std::cout<<"voxel_filtered_pointcloud : "<<voxel_filtered_pointcloud->points.size()<<std::endl;
	cloud_sor(cropbox_filtered_pointcloud, sor_filtered_pointcloud);
	//std::cout<<"sor_filtered_pointcloud : "<<sor_filtered_pointcloud->points.size()<<std::endl;
	cloud_remove_plane(sor_filtered_pointcloud, inlierPoints, inlierPoints_neg);

	sensor_msgs::PointCloud2 filtered_msg;
	pcl::toROSMsg(*inlierPoints_neg, filtered_msg);
	filtered_msg.header.stamp=in_cloud_ptr->header.stamp;
	filtered_msg.header.frame_id=in_cloud_ptr->header.frame_id;
	pub_filtered_point_.publish(filtered_msg);

	sensor_msgs::PointCloud2 plane_msg;
	pcl::toROSMsg(*inlierPoints, plane_msg);
	plane_msg.header.stamp=in_cloud_ptr->header.stamp;
	plane_msg.header.frame_id=in_cloud_ptr->header.frame_id;
	pub_plane_.publish(plane_msg);
	
	//all_pointcloud->clear();
	cropbox_filtered_pointcloud->clear();
	voxel_filtered_pointcloud->clear();
	sor_filtered_pointcloud->clear();
	raw_pointcloud->clear();
	inlierPoints->clear();
	inlierPoints_neg->clear();
	upsampling_pointcloud->clear();
}
