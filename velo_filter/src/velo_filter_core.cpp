#include "velo_filter.h"

VelodyneFilter::VelodyneFilter(ros::NodeHandle &nh){
	sub_point_cloud_=nh.subscribe("/velodyne_points", 10, &VelodyneFilter::cloud_cb, this);
	pub_filtered_point_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_point", 10);
	
	all_pointcloud=pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);
	voxel_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	sor_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	raw_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	cropbox_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	
	//voxel_size | num_neigbor_points | std_multiplier
	nh.getParam("voxel_size", voxel_size_);
	ROS_INFO("voxel_size_ : %f", voxel_size_);
	nh.getParam("num_neigbor_points", num_neigbor_points_);
	ROS_INFO("num_neigbor_points : %d", num_neigbor_points_);
	nh.getParam("std_multiplier", std_multiplier_);
	ROS_INFO("std_multiplier : %f", std_multiplier_);
	
	ros::spin();
}

VelodyneFilter::~VelodyneFilter() {}

void VelodyneFilter::Spin(){
}
/*
void VelodyneFilter::cloud_voxel(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::VoxelGrid<VPoint> voxel_filter;
	//float voxelsize=0.3;
	voxel_filter.setInputCloud(in);
	voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
	voxel_filter.filter(*out);
}
*/
void VelodyneFilter::cloud_sor(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out){
	//int num_neigbor_points=10;
	//double std_multiplier=0.5;
	
	pcl::StatisticalOutlierRemoval<VPoint> sor;
	sor.setInputCloud(in);
	sor.setMeanK(num_neigbor_points_);
	sor.setStddevMulThresh(std_multiplier_);
	sor.filter(*out);
}

void VelodyneFilter::cloud_cropbox(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::CropBox<VPoint> region(true);
	region.setMin(Eigen::Vector4f(-5, -10, -1.7, 1));
	region.setMax(Eigen::Vector4f(15, 10, 2.5, 1));
	region.setInputCloud(in);
	region.filter(*out);
	
	std::vector<int> indices;
	pcl::CropBox<VPoint> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.5, -1.2, 1));
	roof.setMax(Eigen::Vector4f(0.6, 1.5, 0.5, 1));
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

void VelodyneFilter::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr){
	pcl::PointCloud<VPoint> laserCloudIn;
	pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);
	SLRPointXYZIRL SLRpoint;
	VPoint V_point;
	
	for (size_t i=0; i<laserCloudIn.points.size(); i++){
		SLRpoint.x=laserCloudIn.points[i].x;
		SLRpoint.y=laserCloudIn.points[i].y;
		SLRpoint.z=laserCloudIn.points[i].z;
		SLRpoint.intensity=laserCloudIn.points[i].intensity;
		SLRpoint.ring=laserCloudIn.points[i].ring;
		SLRpoint.label=0u;
		all_pointcloud->points.push_back(SLRpoint);
		
		V_point.x=laserCloudIn.points[i].x;
		V_point.y=laserCloudIn.points[i].y;
		V_point.z=laserCloudIn.points[i].z;
		V_point.intensity=laserCloudIn.points[i].intensity;
		V_point.ring=laserCloudIn.points[i].ring;
		raw_pointcloud->points.push_back(V_point);
	}
	
	ROS_INFO("----------------------");
	//std::cout<<"raw_pointcloud_size : "<<raw_pointcloud->points.size()<<std::endl;
	//cloud_voxel(raw_pointcloud, voxel_filtered_pointcloud);
	cloud_cropbox(raw_pointcloud, cropbox_filtered_pointcloud);
	//std::cout<<"voxel_filtered_pointcloud : "<<voxel_filtered_pointcloud->points.size()<<std::endl;
	cloud_sor(cropbox_filtered_pointcloud, sor_filtered_pointcloud);
	//std::cout<<"sor_filtered_pointcloud : "<<sor_filtered_pointcloud->points.size()<<std::endl;
		
	sensor_msgs::PointCloud2 filtered_msg;
	pcl::toROSMsg(*sor_filtered_pointcloud, filtered_msg);
	filtered_msg.header.stamp=in_cloud_ptr->header.stamp;
	filtered_msg.header.frame_id=in_cloud_ptr->header.frame_id;
	pub_filtered_point_.publish(filtered_msg);
	
	cropbox_filtered_pointcloud->clear();
	voxel_filtered_pointcloud->clear();
	sor_filtered_pointcloud->clear();
	raw_pointcloud->clear();
}
