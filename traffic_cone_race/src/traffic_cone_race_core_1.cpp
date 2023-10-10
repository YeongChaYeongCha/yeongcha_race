#include <traffic_cone_race.h>

bool cmp(pair<float, float> cone_0, pair<float, float> cone_1){
  return sqrt(pow(cone_0.first, 2) + pow(cone_0.second, 2)) < sqrt(pow(cone_1.first, 2) + pow(cone_1.second, 2));
}

traffic_cone_race::traffic_cone_race(ros::NodeHandle &nh){
  cone_pos_=nh.subscribe("/object_bounding_boxes", 1000, &traffic_cone_race::cone_callback, this);
  way_point_marker_=nh.advertise<visualization_msgs::Marker>("/way_point", 1000);
  drive_value_=nh.advertise<race::drive_values>("/control_value", 1000);

  nh.getParam("throttle", throttle_);
	ROS_INFO("throttle : %d", throttle_);
  nh.getParam("drive_ratio", drive_ratio_);
	ROS_INFO("drive_ratio : %f", drive_ratio_);

  ros::spin();
}

traffic_cone_race::~traffic_cone_race(){}

void traffic_cone_race::Spin(){}

void traffic_cone_race::cone_callback(const visualization_msgs::MarkerArray &cone_boundingbox){
  //cout<<cone_boundingbox.markers.size()<<"\n";
  
  if(cone_boundingbox.markers.size()>=2){
    pair<float, float> nearestPositivePoint={numeric_limits<float>::max(), 0.0};
    pair<float, float> nearestNegativePoint={numeric_limits<float>::max(), 0.0};

    for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
      float cone_pos_x=iter->pose.position.x;
      float cone_pos_y=iter->pose.position.y;
      
      closest_traffic_cone_.push_back({cone_pos_x, cone_pos_y});
    }

    for (const auto& point : closest_traffic_cone_) {
      if (point.second > 0.0) { 
        float distance = sqrt(pow(point.first, 2) + pow(point.second, 2));
        if (distance < sqrt(pow(nearestPositivePoint.first, 2) + pow(nearestPositivePoint.second, 2))) {
          nearestPositivePoint = point;
        }
      }
      else if (point.second < 0.0) { 
        float distance = sqrt(pow(point.first, 2) + pow(point.second, 2));
        if (distance < sqrt(pow(nearestNegativePoint.first, 2) + pow(nearestNegativePoint.second, 2))) {
          nearestNegativePoint = point;
        }
      }
    }
    cout<<"가까운 점 좌표"<<"\n";
    cout<<nearestPositivePoint.first<<" "<<nearestPositivePoint.second<<"\n";
    cout<<nearestNegativePoint.first<<" "<<nearestNegativePoint.second<<"\n";
	  cout<<"---------------------"<<"\n";
    float avg_x = 0, avg_y = 0;

    avg_x = (nearestPositivePoint.first + nearestNegativePoint.first) / 2;
    avg_y = (nearestPositivePoint.second + nearestNegativePoint.second) / 2;

    way_point_.push_back({avg_x, -avg_y});

    race::drive_values drive_msg;
    if(dis!=0){
      // float steer=-RAD2DEG(atan((2*way_point_[0].second*1.04)/pow(dis,2)))*1.75;
      float steer=RAD2DEG(atan(way_point_[0].second/way_point_[0].first))*drive_ratio_;
      drive_msg.steering= (steer>30.0) ? 30.0 : (steer<-30.0) ? -30.0 : steer;
      // drive_msg.steering=-RAD2DEG(atan((2*way_point_[0].second*1.04)/pow(dis,2)));
      cout<<"Steering : "<<drive_msg.steering<<"\n";
    }
    else{
      drive_msg.steering=0;
      drive_msg.throttle=0;
    }
    drive_msg.throttle=throttle_;
    
    visualization_msgs::Marker way_point_marker;

    if(way_point_[0].first>0.2){
      way_point_marker.header.frame_id="velodyne";
      way_point_marker.header.stamp=ros::Time::now();
      way_point_marker.ns="cylinder";
      way_point_marker.id=0;
      way_point_marker.type=3;
      way_point_marker.action=0;
      way_point_marker.pose.position.x=way_point_[0].first;
      way_point_marker.pose.position.y=-way_point_[0].second;
      way_point_marker.pose.position.z=-0.25;
      way_point_marker.pose.orientation.x=0.0;
      way_point_marker.pose.orientation.y=0.0;
      way_point_marker.pose.orientation.z=0.0;
      way_point_marker.pose.orientation.w=1.0;
      way_point_marker.scale.x=0.2;
      way_point_marker.scale.y=0.2;
      way_point_marker.scale.z=0.2;
      way_point_marker.color.a=1.0;
      way_point_marker.color.r=0.0;
      way_point_marker.color.g=1.0;
      way_point_marker.color.b=0.0;
      way_point_marker.lifetime=ros::Duration(0.2);
    }

    drive_value_.publish(drive_msg);
    way_point_marker_.publish(way_point_marker);
    
    cout<<"======================="<<"\n";
    avg_x=0; avg_y=0;
  }

  closest_traffic_cone_.clear();
  way_point_.clear();
}

