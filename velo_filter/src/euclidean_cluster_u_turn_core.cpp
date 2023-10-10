#include "euclidean_cluster.h"

EuclideanCluster::EuclideanCluster(ros::NodeHandle &nh){
	sub_remove_plane_=nh.subscribe("/filtered_points_no_ground", 1000, &EuclideanCluster::cloud_cluster_callback, this);
	pub_bounding_box_=nh.advertise<visualization_msgs::MarkerArray>("/object_bounding_boxes", 1000);
	pub_drum_condition_=nh.advertise<std_msgs::Bool>("/drum_object_condition", 1000);
	pub_drum_bounding_box_=nh.advertise<visualization_msgs::MarkerArray>("/drum_object_bounding_boxes", 1000);
	
	raw_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	
	nh.getParam("cluster_tolerance", cluster_tolerance_);
	ROS_INFO("cluster_tolerance : %f", cluster_tolerance_);
	nh.getParam("min_cluster_size", min_cluster_size_);
	ROS_INFO("min_cluster_size_ : %d", min_cluster_size_);
	nh.getParam("max_cluster_size", max_cluster_size_);
	ROS_INFO("max_cluster_size : %d", max_cluster_size_);
	
	ros::spin();
}

EuclideanCluster::~EuclideanCluster() {}

void EuclideanCluster::Spin(){
}

void EuclideanCluster::cloud_cluster_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr){
	// pcl::PointCloud<VPoint> laserCloudIn;
	pcl::fromROSMsg(*in_cloud_ptr, *raw_pointcloud);
	VPoint V_point;
	
	int buffer_size=3000000;
	uint8_t buffer[buffer_size];
	ros::serialization::OStream stream(buffer, buffer_size);
	
	// for (size_t i=0; i<laserCloudIn.points.size(); i++){
	// 	V_point.x=laserCloudIn.points[i].x;
	// 	V_point.y=laserCloudIn.points[i].y;
	// 	V_point.z=laserCloudIn.points[i].z;
	// 	V_point.intensity=laserCloudIn.points[i].intensity;
	// 	V_point.ring=laserCloudIn.points[i].ring;
	// 	raw_pointcloud->points.push_back(V_point);
	// }
	pcl::search::KdTree<VPoint>::Ptr tree(new pcl::search::KdTree<VPoint>);
	tree->setInputCloud(raw_pointcloud);

	std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<VPoint> ec;
	ec.setInputCloud(raw_pointcloud);
	ec.setClusterTolerance(cluster_tolerance_);
	ec.setMinClusterSize(min_cluster_size_); 
	ec.setMaxClusterSize(max_cluster_size_);
	ec.setSearchMethod(tree);
	ec.extract(cluster_indices);

	//cloud_cluster->points.push_back(cluster_indeices);
	std::vector<pcl::PointCloud<VPoint>::Ptr> cloud_clusters;
	
	visualization_msgs::MarkerArray bounding_box_array;
	visualization_msgs::MarkerArray Drum_bounding_box_array;
	std::cout<<"Cluster Size : "<<cluster_indices.size()<<std::endl;
	for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it){
		pcl::PointCloud<VPoint>::Ptr cloud_cluster(new pcl::PointCloud<VPoint>);
		for(std::vector<int>::const_iterator pit=it->indices.begin(); pit!=it->indices.end(); ++pit){
				cloud_cluster->points.push_back(raw_pointcloud->points[*pit]);
		}
		cloud_clusters.push_back(cloud_cluster);
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::copyPointCloud(*cloud_cluster, *cloud_cluster_xyzi);
		
		//pcl::PointXYZRGB cluster_min_point, cluster_max_point, cluster_centroid;
		pcl::PointXYZI cluster_min_point, cluster_max_point, cluster_centroid;
		pcl::getMinMax3D(*cloud_cluster_xyzi, cluster_min_point, cluster_max_point);
		pcl::computeCentroid(*cloud_cluster, cluster_centroid);
		
		float cluster_depth=cluster_max_point.x - cluster_min_point.x;
		float cluster_width=cluster_max_point.y - cluster_min_point.y;
		float cluster_height=cluster_max_point.z - cluster_min_point.z;
		
		/*
		std::cout<<"Cluster Bounding Box"<<std::endl;
		std::cout<<"Min Point : "<<cluster_min_point<<std::endl;
		std::cout<<"Max Point : "<<cluster_max_point<<std::endl;
		std::cout<<"Width : "<<cluster_width<<std::endl;
		std::cout<<"Depth : "<<cluster_depth<<std::endl;
		std::cout<<"Height : "<<cluster_height<<std::endl;
		std::cout<<"Centroid : "<<cluster_centroid<<std::endl;
		*/
		/*
		pcl::visualization::PCLVisualizer viewer("Cluster Bounding Box");
		viewer.addPointCloud<pcl::PointXYZI>(cloud_cluster_xyzi, "cloud_cluster");
		viewer.addCube(cluster_min_point.x, cluster_max_point.x, cluster_min_point.y, cluster_max_point.y, cluster_min_point.z, cluster_max_point.z);
		viewer.spin();
		*/

		//Case 1 Drum Cone
		if((cluster_depth<0.25 && cluster_width<0.25) && (cluster_height>0.30 && cluster_height<0.55)){
			if((cluster_centroid.x<8.0 && cluster_centroid.x>0.1) && (cluster_centroid.y<5.0 && cluster_centroid.y>-3.5) && cluster_centroid.z<-0.2){
				// avg_depth : 0.147(0.09~0.20) avg_width : 0.10635(0.08~0.20) avg_height : 0.444(0.39~0.50)
				// avg_centroid_z : -0.368(-0.50~-0.29)
				std::cout<<"Traffic Cone Cluster Bounding Box"<<std::endl;
				std::cout<<"Min Point : "<<cluster_min_point<<std::endl;
				std::cout<<"Max Point : "<<cluster_max_point<<std::endl;
				std::cout<<"Width : "<<cluster_width<<std::endl;
				std::cout<<"Depth : "<<cluster_depth<<std::endl;
				std::cout<<"Height : "<<cluster_height<<std::endl;
				std::cout<<"Centroid : "<<cluster_centroid<<std::endl;
				
				visualization_msgs::Marker bounding_box;
				bounding_box.header.frame_id="velodyne";
				bounding_box.header.stamp=ros::Time::now();
				bounding_box.ns="cluster_bounding_boxes";
				bounding_box.id=static_cast<int>(it - cluster_indices.begin());
				bounding_box.type=visualization_msgs::Marker::CUBE;
				bounding_box.action=visualization_msgs::Marker::ADD;
				bounding_box.pose.position.x=cluster_centroid.x;
				bounding_box.pose.position.y=cluster_centroid.y;
				bounding_box.pose.position.z=cluster_centroid.z;
				bounding_box.scale.x=cluster_depth;
				bounding_box.scale.y=cluster_width;
				bounding_box.scale.z=cluster_height;
				bounding_box.color.r=1.0;
				bounding_box.color.g=0.0;
				bounding_box.color.b=0.0;
				bounding_box.color.a=1.0;
				bounding_box.lifetime=ros::Duration(0.2);
			
				bounding_box_array.markers.emplace_back(bounding_box);
			}
		}
		else if((cluster_depth<0.5 && cluster_width<0.50) && (cluster_height>0.45 && cluster_height<0.75)){
			if((cluster_centroid.x<5.0 && cluster_centroid.x>0.1) && (cluster_centroid.y<10.0 && cluster_centroid.y>0.0) && cluster_centroid.z<0.0){
				// avg_depth : 0.147(0.09~0.20) avg_width : 0.10635(0.08~0.20) avg_height : 0.444(0.39~0.50)
				// avg_centroid_z : -0.368(-0.50~-0.29)
				std::cout<<"Drum Cone Cluster Bounding Box"<<std::endl;
				std::cout<<"Min Point : "<<cluster_min_point<<std::endl;
				std::cout<<"Max Point : "<<cluster_max_point<<std::endl;
				std::cout<<"Width : "<<cluster_width<<std::endl;
				std::cout<<"Depth : "<<cluster_depth<<std::endl;
				std::cout<<"Height : "<<cluster_height<<std::endl;
				std::cout<<"Centroid : "<<cluster_centroid<<std::endl;
				
				visualization_msgs::Marker Drum_bounding_box;
				Drum_bounding_box.header.frame_id="velodyne";
				Drum_bounding_box.header.stamp=ros::Time::now();
				Drum_bounding_box.ns="cluster_bounding_boxes";
				Drum_bounding_box.id=static_cast<int>(it - cluster_indices.begin());
				Drum_bounding_box.type=visualization_msgs::Marker::CUBE;
				Drum_bounding_box.action=visualization_msgs::Marker::ADD;
				Drum_bounding_box.pose.position.x=cluster_centroid.x;
				Drum_bounding_box.pose.position.y=cluster_centroid.y;
				Drum_bounding_box.pose.position.z=cluster_centroid.z;
				Drum_bounding_box.scale.x=cluster_depth;
				Drum_bounding_box.scale.y=cluster_width;
				Drum_bounding_box.scale.z=cluster_height;
				Drum_bounding_box.color.r=0.0;
				Drum_bounding_box.color.g=0.0;
				Drum_bounding_box.color.b=1.0;
				Drum_bounding_box.color.a=1.0;
				Drum_bounding_box.lifetime=ros::Duration(0.2);
			
				Drum_bounding_box_array.markers.emplace_back(Drum_bounding_box);
			}
		}
	}
	std_msgs::Bool drum_condition;
	if(Drum_bounding_box_array.markers.size()>=2){
		drum_con_=true;
	}
	// else{
	// 	drum_con_=false;
	// }
	drum_condition.data=drum_con_;
	pub_bounding_box_.publish(bounding_box_array);
	pub_drum_bounding_box_.publish(Drum_bounding_box_array);
	pub_drum_condition_.publish(drum_condition);
	std::cout<<"============================"<<std::endl;
	
	raw_pointcloud->clear();
	cluster_indices.clear();
}

