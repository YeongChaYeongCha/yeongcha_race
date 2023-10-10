#include <traffic_cone_race.h>

bool cmp(pair<float, float> cone_0, pair<float, float> cone_1){
  float angle_0=RAD2DEG(atan(cone_0.second/cone_0.first));
  float angle_1=RAD2DEG(atan(cone_1.second/cone_1.first));
  
  return angle_0<angle_1;
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
  
  if(cone_boundingbox.markers.size()>=3){
    pair<float, float> positivePoint={0.0, 0.0};
    pair<float, float> negativePoint={0.0, 0.0};
    
    for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
      float cone_pos_x=iter->pose.position.x;
      float cone_pos_y=iter->pose.position.y;

      cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
      
      closest_traffic_cone_.push_back({cone_pos_x, -cone_pos_y});
    }
    cout<<"---------------------"<<"\n";

    sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp);
    negativePoint=*closest_traffic_cone_.begin();
    positivePoint=*(closest_traffic_cone_.end()-1);
    
    remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), positivePoint);
    remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), negativePoint);

    cout<<"가까운 점 좌표"<<"\n";
    cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
    cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
    cout<<closest_traffic_cone_[0].first<<" "<<closest_traffic_cone_[0].second<<"\n";
	  cout<<"---------------------"<<"\n";
    float avg_x = 0, avg_y = 0;

    avg_x = (positivePoint.first + negativePoint.first + closest_traffic_cone_[0].first) / 3;
    avg_y = (positivePoint.second + negativePoint.second) / 2;

    way_point_.push_back({avg_x, avg_y});

    cout << avg_x << " " << avg_y << "\n";
    cout<<"---------------------"<<"\n";

    race::drive_values drive_msg;
    if(dis!=0){
      float ldp_deg=-atan(way_point_[0].second/way_point_[0].first);
      // float steer=RAD2DEG(atan(way_point_[0].second/way_point_[0].first))*drive_ratio_;
      float steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(way_point_[0].first - 0.45 , 2) + pow(way_point_[0].second, 2)))); // vel(8) 0.30 : 곡률이 심한 커브에서 버거 | 0.40 : 조향각 약간 부족함. | 0.45 : 
      // drive_msg.steering= (steer>30.0) ? 30.0 : (steer<-30.0) ? -30.0 : steer;
      drive_msg.steering = steer;
      cout<<"Steering : "<<drive_msg.steering<<"\n";
    }
    else{
      drive_msg.steering=0;
      drive_msg.throttle=0;
    }
    drive_msg.throttle = (drive_msg.steering < 30.0) ? 10 : 7;
    
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





