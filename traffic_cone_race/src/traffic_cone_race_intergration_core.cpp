#include "traffic_cone_intergration.h"

bool cmp_deg(pair<float, float> cone_0, pair<float, float> cone_1){
  float angle_0=RAD2DEG(atan(cone_0.second/cone_0.first));
  float angle_1=RAD2DEG(atan(cone_1.second/cone_1.first));
  
  return angle_0<angle_1;
}

bool cmp_dis(pair<float, float> cone_0, pair<float, float> cone_1){
  return sqrt(pow(cone_0.first, 2) + pow(cone_0.second, 2)) < sqrt(pow(cone_1.first, 2) + pow(cone_1.second, 2));
}

TrafficConeRace::TrafficConeRace(ros::NodeHandle &nh){
    sub_point_cloud_=nh.subscribe("/velodyne_points", 10, &TrafficConeRace::cloud_cb, this);

    pub_filtered_point_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 1000);
	pub_plane_=nh.advertise<sensor_msgs::PointCloud2>("/plane_point", 1000);
    pub_bounding_box_=nh.advertise<visualization_msgs::MarkerArray>("/object_bounding_boxes", 1000);
    way_point_marker_=nh.advertise<visualization_msgs::Marker>("/way_point", 1000);
    drive_value_=nh.advertise<race::drive_values>("/control_value", 1000);
    way_point_angle_=nh.advertise<visualization_msgs::Marker>("/way_point_angle", 1000);

    voxel_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	sor_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	raw_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	cropbox_filtered_pointcloud=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	inlierPoints=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
	inlierPoints_neg=pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);

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
    nh.getParam("cluster_tolerance", cluster_tolerance_);
	ROS_INFO("cluster_tolerance : %f", cluster_tolerance_);
	nh.getParam("min_cluster_size", min_cluster_size_);
	ROS_INFO("min_cluster_size_ : %d", min_cluster_size_);
	nh.getParam("max_cluster_size", max_cluster_size_);
	ROS_INFO("max_cluster_size : %d", max_cluster_size_);

    ros::spin();
}

TrafficConeRace::~TrafficConeRace() {
}

void TrafficConeRace::Spin(){
}

void TrafficConeRace::cloud_remove_plane(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr plane, const pcl::PointCloud<VPoint>::Ptr out_without_plane){
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

void TrafficConeRace::cloud_voxel(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::VoxelGrid<VPoint> voxel_filter;
	voxel_filter.setInputCloud(in);
	voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
	voxel_filter.filter(*out);
}

void TrafficConeRace::cloud_sor(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out){
	pcl::StatisticalOutlierRemoval<VPoint> sor;
	sor.setInputCloud(in);
	sor.setMeanK(num_neigbor_points_);
	sor.setStddevMulThresh(std_multiplier_);
	sor.filter(*out);
}

void TrafficConeRace::cloud_cropbox(const pcl::PointCloud<VPoint>::ConstPtr in, const pcl::PointCloud<VPoint>::Ptr out){
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

void TrafficConeRace::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr){
    int buffer_size=3000000;
	uint8_t buffer[buffer_size];
	ros::serialization::OStream stream(buffer, buffer_size);

	pcl::fromROSMsg(*in_cloud_ptr, *raw_pointcloud);
	
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

    pcl::search::KdTree<VPoint>::Ptr tree(new pcl::search::KdTree<VPoint>);
	tree->setInputCloud(inlierPoints_neg);

    std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<VPoint> ec;
	ec.setInputCloud(raw_pointcloud);
	ec.setClusterTolerance(cluster_tolerance_);
	ec.setMinClusterSize(min_cluster_size_); 
	ec.setMaxClusterSize(max_cluster_size_);
	ec.setSearchMethod(tree);
	ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<VPoint>::Ptr> cloud_clusters;
	
	visualization_msgs::MarkerArray bounding_box_array;
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
		
		if((cluster_depth<0.25 && cluster_width<0.25) && (cluster_height>0.30 && cluster_height<0.55)){
			if((cluster_centroid.x<8.0 && cluster_centroid.x>1.0) && abs(cluster_centroid.y)<2.3 && cluster_centroid.z<-0.2){ 
				// avg_depth : 0.147(0.09~0.20) avg_width : 0.10635(0.08~0.20) avg_height : 0.444(0.39~0.50)
				// avg_centroid_z : -0.368(-0.50~-0.29)
                closest_traffic_cone_.push_back({cluster_centroid.x, cluster_centroid.y});
				
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
	}
    // visualization_msgs::Marker way_point_marker;
    // visualization_msgs::Marker way_point_angle;
    // if(closest_traffic_cone_.size()>=3){
    //     float avg_x = 0, avg_y = 0;
    //     pair<float, float> positivePoint={0.0, 0.0};
    //     pair<float, float> negativePoint={0.0, 0.0};
    //     pair<float, float> thirdPoint={0.0, 0.0};

    //     sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp_deg);

    //     negativePoint=*(closest_traffic_cone_.begin());
    //     positivePoint=*(closest_traffic_cone_.end()-1);

    //     closest_traffic_cone_.erase(remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), positivePoint), closest_traffic_cone_.end());
    //     closest_traffic_cone_.erase(remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), negativePoint), closest_traffic_cone_.end());

    //     sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp_dis);

    //     thirdPoint=*(closest_traffic_cone_.begin());

    //     if(positivePoint.first>thirdPoint.first){
    //         float inclination = ((thirdPoint.second-positivePoint.second)/(thirdPoint.first-positivePoint.first));
    //         float curve_avg_x=(thirdPoint.first+positivePoint.first)/2;
    //         float curve_avg_y=(thirdPoint.second+positivePoint.second)/2;

    //         cout<<"##우회전##"<<"\n";
    //         cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
    //         cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";

    //         way_point_.push_back({-1.8*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x, 1.8/sqrt(pow(inclination, 2)+1)+curve_avg_y});
    //     }

    //     else if(negativePoint.first>thirdPoint.first){
    //         float inclination = ((thirdPoint.second-negativePoint.second)/(thirdPoint.first-negativePoint.first));
    //         float curve_avg_x=(thirdPoint.first+negativePoint.first)/2;
    //         float curve_avg_y=(thirdPoint.second+negativePoint.second)/2;

    //         cout<<"##좌회전##"<<"\n";
    //         cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
    //         cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";

    //         way_point_.push_back({1.8*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x, -1.8/sqrt(pow(inclination, 2)+1)+curve_avg_y});
    //     }
    //     // 7 : 5 -> 1.8

    //     else{
    //         avg_x = (positivePoint.first + negativePoint.first + thirdPoint.first) / 3;
    //         avg_y = (positivePoint.second + negativePoint.second) / 2;

    //         way_point_.push_back({avg_x, avg_y});

    //         cout<<"--Goal Point--"<<"\n";
    //         cout << avg_x << " " << avg_y << "\n";
    //         cout<<"---------------------"<<"\n";
    //     }

    //     race::drive_values drive_msg;
    //     float ldp_deg=-atan(way_point_[0].second/way_point_[0].first);
    //     float steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(way_point_[0].first - 0.50 , 2) + pow(way_point_[0].second, 2))));
    //     // vel(8) 0.30 : 곡률이 심한 커브에서 버거 | 0.40 : 조향각 약간 부족함. | 0.45 : 22시즌 카울 사용 시 적합 | 0.53 : 카울 미사용시 적합
    //     drive_msg.steering = steer;
    //     cout<<"Steering : "<<steer<<"\n";
    //     drive_msg.throttle = (abs(drive_msg.steering) < 15.0) ? 12 : 8;
    
    //     if(way_point_[0].first>0.2){
    //         way_point_marker.header.frame_id="velodyne";
    //         way_point_marker.header.stamp=ros::Time::now();
    //         way_point_marker.ns="cylinder";
    //         way_point_marker.id=0;
    //         way_point_marker.type=3;
    //         way_point_marker.action=0;
    //         way_point_marker.pose.position.x=way_point_[0].first;
    //         way_point_marker.pose.position.y=-way_point_[0].second;
    //         way_point_marker.pose.position.z=0.0;
    //         way_point_marker.pose.orientation.x=0.0;
    //         way_point_marker.pose.orientation.y=0.0;
    //         way_point_marker.pose.orientation.z=0.0;
    //         way_point_marker.pose.orientation.w=1.0;
    //         way_point_marker.scale.x=0.2;
    //         way_point_marker.scale.y=0.2;
    //         way_point_marker.scale.z=0.2;
    //         way_point_marker.color.a=1.0;
    //         way_point_marker.color.r=0.0;
    //         way_point_marker.color.g=1.0;
    //         way_point_marker.color.b=0.0;
    //         way_point_marker.lifetime=ros::Duration(0.2);
      
    //         way_point_angle.header.frame_id="velodyne";
    //         way_point_angle.header.stamp=ros::Time::now();
    //         way_point_angle.ns="checker";
    //         way_point_angle.id=0;
    //         way_point_angle.type=visualization_msgs::Marker::LINE_STRIP;
    //         way_point_angle.action=visualization_msgs::Marker::ADD;
    //         way_point_angle.pose.orientation.x=0.0;
    //         way_point_angle.pose.orientation.y=0.0;
    //         way_point_angle.pose.orientation.z=0.0;
    //         way_point_angle.pose.orientation.w=1.0;
    //         way_point_angle.scale.x=0.06;
    //         way_point_angle.color.a=1.0;
    //         way_point_angle.color.r=1.0;
    //         way_point_angle.color.g=1.0;
    //         way_point_angle.color.b=0.0;
            
    //         geometry_msgs::Point square_one, dir_steer;
    //         square_one.x=0.0;
    //         square_one.y=0.0;
    //         square_one.z=0.3;
    //         dir_steer.x=way_point_[0].first*0.3;
    //         dir_steer.y=-way_point_[0].second*0.3;
    //         dir_steer.z=0.3;
      
    //         way_point_angle.points.push_back(square_one);
    //         way_point_angle.points.push_back(dir_steer);
    //         way_point_angle.lifetime=ros::Duration(0.2);
    //     }
    //     drive_value_.publish(drive_msg);
    // }

    // pub_bounding_box_.publish(bounding_box_array);
    // way_point_marker_.publish(way_point_marker);
    // way_point_angle_.publish(way_point_angle);
	
	cropbox_filtered_pointcloud->clear();
	voxel_filtered_pointcloud->clear();
	sor_filtered_pointcloud->clear();
	raw_pointcloud->clear();
	inlierPoints->clear();
	inlierPoints_neg->clear();
    // cluster_indices.clear();
    closest_traffic_cone_.clear();
    way_point_.clear();
}