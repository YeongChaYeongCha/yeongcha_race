#include "euclidean_cluster.h"

bool cmp(std::pair<float, float> cone_0, std::pair<float, float> cone_1){
  float angle_0=RAD2DEG(atan(cone_0.second/cone_0.first));
  float angle_1=RAD2DEG(atan(cone_1.second/cone_1.first));
  
  return angle_0<angle_1;
}
// bool cmp_dis(std::pair<float, float> cone_0, std::pair<float, float> cone_1){
// 	return sqrt(std::pow(cone_0.first, 2) + std::pow(cone_0.second, 2)) > sqrt(std::pow(cone_1.first, 2) + std::pow(cone_1.second, 2));
// }

// bool cmp(std::pair<float, float> cone_0, std::pair<float, float> cone_1){
//   return sqrt(std::pow(cone_0.first, 2) + std::pow(cone_0.second, 2)) < sqrt(std::pow(cone_1.first, 2) + std::pow(cone_1.second, 2));
// }

// bool cmp_rev(std::pair<float, float> cone_0, std::pair<float, float> cone_1){
//   return sqrt(std::pow(cone_0.first, 2) + std::pow(cone_0.second, 2)) > sqrt(std::pow(cone_1.first, 2) + std::pow(cone_1.second, 2));
// }

EuclideanCluster::EuclideanCluster(ros::NodeHandle &nh){
	sub_remove_plane_=nh.subscribe("/filtered_points_no_ground", 1000, &EuclideanCluster::cloud_cluster_callback, this);
	pub_bounding_left_box_=nh.advertise<visualization_msgs::MarkerArray>("/object_bounding_left_boxes", 1000);
	pub_bounding_right_box_=nh.advertise<visualization_msgs::MarkerArray>("/object_bounding_right_boxes", 1000);

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
	std::vector<std::pair<float, float>> left_cloud_clusters;
	std::vector<std::pair<float, float>> right_cloud_clusters;
	
	visualization_msgs::MarkerArray bounding_box_left_array;
	visualization_msgs::MarkerArray bounding_box_right_array;

	std::vector<std::pair<float, float>> traffic_cone;
	// visualization_msgs::MarkerArray bounding_box_array;
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
		pcl::visualization::PCLVisualizer viewer("Cluster Bounding Box");
		viewer.addPointCloud<pcl::PointXYZI>(cloud_cluster_xyzi, "cloud_cluster");
		viewer.addCube(cluster_min_point.x, cluster_max_point.x, cluster_min_point.y, cluster_max_point.y, cluster_min_point.z, cluster_max_point.z);
		viewer.spin();
		*/
		if((cluster_depth<0.25 && cluster_width<0.25) && (cluster_height>0.30 && cluster_height<0.55)){
			if((cluster_centroid.x<6.0 && cluster_centroid.x>0.5) && abs(cluster_centroid.y)<3.5 && cluster_centroid.z<-0.2){ 
				// avg_depth : 0.147(0.09~0.20) avg_width : 0.10635(0.08~0.20) avg_height : 0.444(0.39~0.50)
				// avg_centroid_z : -0.368(-0.50~-0.29)

				std::cout<<"Min Point : "<<cluster_min_point<<"\n";
				std::cout<<"Max Point : "<<cluster_max_point<<"\n";
				std::cout<<"Width : "<<cluster_width<<"\n";
				std::cout<<"Depth : "<<cluster_depth<<"\n";
				std::cout<<"Height : "<<cluster_height<<"\n";
				std::cout<<"Centroid : "<<cluster_centroid<<"\n";
				std::cout<<"--------------------------"<<"\n";
				traffic_cone.push_back({cluster_centroid.x, cluster_centroid.y});
			}
		}
	}
	sort(traffic_cone.begin(), traffic_cone.end(), cmp);

	std::cout<<"Traffic Cone Coordination"<<"\n";
	for(auto iter=traffic_cone.begin(); iter!=traffic_cone.end(); iter++){
		std::cout<<iter->first<<" "<<iter->second<<"\n";
	}
	std::cout<<"--------------------------"<<"\n";

	if(traffic_cone.size()>2){
		float pre_x_pos=traffic_cone.begin()->first; float pre_y_pos=traffic_cone.begin()->second;
		float for_x_pos=(traffic_cone.begin()+1)->first; float for_y_pos=(traffic_cone.begin()+1)->second;
		bool std_dir=false;

		visualization_msgs::Marker bounding_left_box;
		visualization_msgs::Marker bounding_right_box;

		if(sqrt(pow(traffic_cone.begin()->first - 1.0, 2) + pow(traffic_cone.begin()->second + 2.0, 2))<2.3 || traffic_cone.begin()->second<-0.5){
			bounding_right_box.header.frame_id="velodyne";
			bounding_right_box.header.stamp=ros::Time::now();
			bounding_right_box.ns="cluster_bounding_boxes";
			bounding_right_box.id=0;
			bounding_right_box.type=visualization_msgs::Marker::CUBE;
			bounding_right_box.action=visualization_msgs::Marker::ADD;
			bounding_right_box.pose.position.x=traffic_cone.begin()->first;
			bounding_right_box.pose.position.y=traffic_cone.begin()->second;
			bounding_right_box.pose.position.z=-0.37;
			bounding_right_box.scale.x=0.150;
			bounding_right_box.scale.y=0.150;
			bounding_right_box.scale.z=0.450;
			bounding_right_box.color.r=0.0;
			bounding_right_box.color.g=0.0;
			bounding_right_box.color.b=1.0;
			bounding_right_box.color.a=1.0;
			bounding_right_box.lifetime=ros::Duration(0.1);
			bounding_box_right_array.markers.emplace_back(bounding_right_box);
		}

		if(sqrt(pow((traffic_cone.end()-1)->first - 1.0, 2) + pow((traffic_cone.end()-1)->second - 2.0, 2))<2.3 || traffic_cone.begin()->second>0.5){
			bounding_left_box.header.frame_id="velodyne";
			bounding_left_box.header.stamp=ros::Time::now();
			bounding_left_box.ns="cluster_bounding_boxes";
			bounding_left_box.id=traffic_cone.size()-1;
			bounding_left_box.type=visualization_msgs::Marker::CUBE;
			bounding_left_box.action=visualization_msgs::Marker::ADD;
			bounding_left_box.pose.position.x=(traffic_cone.end()-1)->first;
			bounding_left_box.pose.position.y=(traffic_cone.end()-1)->second;
			bounding_left_box.pose.position.z=-0.37;
			bounding_left_box.scale.x=0.150;
			bounding_left_box.scale.y=0.150;
			bounding_left_box.scale.z=0.450;
			bounding_left_box.color.r=1.0;
			bounding_left_box.color.g=1.0;
			bounding_left_box.color.b=0.0;
			bounding_left_box.color.a=1.0;
			bounding_left_box.lifetime=ros::Duration(0.1);
			bounding_box_left_array.markers.emplace_back(bounding_left_box);
		}
		
		bool btn=false;
		for(auto iter=traffic_cone.begin(); iter!=traffic_cone.end(); ++iter){
			cout<<iter->first<<" "<<-iter->second;
			if(iter==traffic_cone.begin() || iter==traffic_cone.end()-1){
				cout<<"\n";
				continue;
			}

			if(iter->first!=(traffic_cone.end()-1)->first && iter->second!=(traffic_cone.end()-1)->second){
				for_x_pos=(iter+1)->first;
				for_y_pos=(iter+1)->second;
			}

			std::cout<<"\n";
			std::cout<<"Previous : "<<pre_x_pos<<" "<<-pre_y_pos<<"\n";
			std::cout<<"Former : "<<for_x_pos<<" "<<-for_y_pos<<"\n";

			std::cout<<"Distance"<<"\n";
			std::cout<<sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2))<<" "<<sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))<<"\n";
			std::cout<<"---------------"<<"\n";

			// if(iter->first-pre_x_pos<0/* || sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) > 3.0*/){
			// 	// std::cout<<"Distance"<<"\n";
			// 	// std::cout<<sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2))<<" "<<sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))<<"\n";
			// 	// std::cout<<"---------------"<<"\n";
			// 	if(sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) < sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))){
			// 		std_dir=true;
			// 	}
			// }

			if(iter->first-pre_x_pos<0){
				std_dir=true;
			}

			// if(iter->first-pre_x_pos<0 && sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) < sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))){
			// 	std_dir=true;
			// }
			// if(iter->first-pre_x_pos<0 && btn==false){
			// 	if(sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) < sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))){
			// 		std_dir=true;
			// 	}
			// 	else if(sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) > sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))){
			// 		std_dir=false;
			// 		btn=true;
			// 	}
			// }

			// if(sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) < sqrt(pow(iter->first - for_x_pos, 2) + pow(iter->second - for_y_pos, 2))){
			// 	if((iter->first-pre_x_pos>0 && iter->first-for_x_pos<0)||(iter->first-pre_x_pos>0 && iter->first-for_x_pos>0)){
			// 		std_dir=true;
			// 	}
			// }

			std::cout<<"Condition : "<<std_dir<<"\n";
			if(std_dir==false){
				bounding_right_box.header.frame_id="velodyne";
				bounding_right_box.header.stamp=ros::Time::now();
				bounding_right_box.ns="cluster_bounding_boxes";
				bounding_right_box.id=(iter - traffic_cone.begin());
				bounding_right_box.type=visualization_msgs::Marker::CUBE;
				bounding_right_box.action=visualization_msgs::Marker::ADD;
				bounding_right_box.pose.position.x=iter->first;
				bounding_right_box.pose.position.y=iter->second;
				bounding_right_box.pose.position.z=-0.37;
				bounding_right_box.scale.x=0.150;
				bounding_right_box.scale.y=0.150;
				bounding_right_box.scale.z=0.450;
				bounding_right_box.color.r=0.0;
				bounding_right_box.color.g=0.0;
				bounding_right_box.color.b=1.0;
				bounding_right_box.color.a=1.0;
				bounding_right_box.lifetime=ros::Duration(0.1);
				// cout<<" Blue"<<"\n";
			
				bounding_box_right_array.markers.emplace_back(bounding_right_box);
			}
			else{
				bounding_left_box.header.frame_id="velodyne";
				bounding_left_box.header.stamp=ros::Time::now();
				bounding_left_box.ns="cluster_bounding_boxes";
				bounding_left_box.id=(iter - traffic_cone.begin());
				bounding_left_box.type=visualization_msgs::Marker::CUBE;
				bounding_left_box.action=visualization_msgs::Marker::ADD;
				bounding_left_box.pose.position.x=iter->first;
				bounding_left_box.pose.position.y=iter->second;
				bounding_left_box.pose.position.z=-0.37;
				bounding_left_box.scale.x=0.150;
				bounding_left_box.scale.y=0.150;
				bounding_left_box.scale.z=0.450;
				bounding_left_box.color.r=1.0;
				bounding_left_box.color.g=1.0;
				bounding_left_box.color.b=0.0;
				bounding_left_box.color.a=1.0;
				bounding_left_box.lifetime=ros::Duration(0.1);
				// cout<<" Yellow"<<"\n";
			
				bounding_box_left_array.markers.emplace_back(bounding_left_box);
			}

			// if(iter->first-pre_x_pos>0 && sqrt(pow(iter->first - pre_x_pos, 2) + pow(iter->second - pre_y_pos, 2)) < 3.0){
			// 	bounding_right_box.header.frame_id="velodyne";
			// 	bounding_right_box.header.stamp=ros::Time::now();
			// 	bounding_right_box.ns="cluster_bounding_boxes";
			// 	bounding_right_box.id=(iter - traffic_cone.begin());
			// 	bounding_right_box.type=visualization_msgs::Marker::CUBE;
			// 	bounding_right_box.action=visualization_msgs::Marker::ADD;
			// 	bounding_right_box.pose.position.x=iter->first;
			// 	bounding_right_box.pose.position.y=iter->second;
			// 	bounding_right_box.pose.position.z=-0.37;
			// 	bounding_right_box.scale.x=0.150;
			// 	bounding_right_box.scale.y=0.150;
			// 	bounding_right_box.scale.z=0.450;
			// 	bounding_right_box.color.r=0.0;
			// 	bounding_right_box.color.g=0.0;
			// 	bounding_right_box.color.b=1.0;
			// 	bounding_right_box.color.a=1.0;
			// 	bounding_right_box.lifetime=ros::Duration(0.1);
			// 	cout<<" Blue"<<"\n";
			
			// 	bounding_box_right_array.markers.emplace_back(bounding_right_box);
			// }

			// else{
			// 	bounding_left_box.header.frame_id="velodyne";
			// 	bounding_left_box.header.stamp=ros::Time::now();
			// 	bounding_left_box.ns="cluster_bounding_boxes";
			// 	bounding_left_box.id=(iter - traffic_cone.begin());
			// 	bounding_left_box.type=visualization_msgs::Marker::CUBE;
			// 	bounding_left_box.action=visualization_msgs::Marker::ADD;
			// 	bounding_left_box.pose.position.x=iter->first;
			// 	bounding_left_box.pose.position.y=iter->second;
			// 	bounding_left_box.pose.position.z=-0.37;
			// 	bounding_left_box.scale.x=0.150;
			// 	bounding_left_box.scale.y=0.150;
			// 	bounding_left_box.scale.z=0.450;
			// 	bounding_left_box.color.r=1.0;
			// 	bounding_left_box.color.g=1.0;
			// 	bounding_left_box.color.b=0.0;
			// 	bounding_left_box.color.a=1.0;
			// 	bounding_left_box.lifetime=ros::Duration(0.1);
			// 	cout<<" Yellow"<<"\n";
			
			// 	bounding_box_left_array.markers.emplace_back(bounding_left_box);
			// }
			pre_x_pos=iter->first;
			pre_y_pos=iter->second;
		}
	}

	pub_bounding_left_box_.publish(bounding_box_left_array);
	pub_bounding_right_box_.publish(bounding_box_right_array);
	cout<<"\n";
	std::cout<<"============================"<<std::endl;
	
	raw_pointcloud->clear();
	cluster_indices.clear();
}