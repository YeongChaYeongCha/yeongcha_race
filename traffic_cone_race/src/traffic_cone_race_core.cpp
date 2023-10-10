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
    for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
      float cone_pos_x=iter->pose.position.x;
      float cone_pos_y=iter->pose.position.y;
      
      // cout<<cone_pos_x<<" "<<cone_pos_y<<"\n";
      
      closest_traffic_cone_.push_back({cone_pos_x, cone_pos_y});
    }
    sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp);

    cout<<"인식된 라바콘 위치"<<"\n";
    for(const auto& cone : closest_traffic_cone_){
      cout<<cone.first<<" "<<cone.second<<"\n";
    }
	  cout<<"---------------------"<<"\n";

    float avg_x=0, avg_y=0;
  
    //가까운 좌우 라바콘(2개)의 평균 위치
    // auto it = closest_traffic_cone_.begin();
    // while (next(it)!=closest_traffic_cone_.end()) {
    //   if ((it->second>0 && next(it)->second<0) || (it->second<0 && next(it)->second>0)) {
    //     float avg_x=(it->first+next(it)->first)/2.0;
    //     float avg_y=(it->second+next(it)->second)/2.0;
    //     way_point_.push_back({avg_x, avg_y});
    //   }
    //   ++it;
    // }

    //가까운 라바콘(3개)의 평균 위치
    // auto it = closest_traffic_cone_.begin();
    // while(it!=closest_traffic_cone_.end() && distance(closest_traffic_cone_.begin(), it)<=2){
    //   cout<<avg_x<<" "<<avg_y<<"\n";
    //   avg_x+=it->first;
    //   avg_y+=it->second;
    //   it++;
    // }
    // avg_x/=3; avg_y/=3;
    // way_point_.push_back({avg_x, avg_y});

    //좌우 각각 라바콘 인식한 후 가장 가까운 라바콘 인식하여 way point 형성
    // auto it = closest_traffic_cone_.begin();
    // while (next(it)!=closest_traffic_cone_.end()) {
    //   if ((it->second>0 && next(it)->second<0) || (it->second<0 && next(it)->second>0)) {
    //     avg_x=(it->first+next(it)->first);
    //     avg_y=(it->second+next(it)->second);

    //     cout<<"way point 계산할 때 고려하는 라바콘"<<"\n";
    //     cout<<it->first<<it->second<<"\n";
    //     cout << next(it)->first << " " << next(it)->second << "\n";

    //     it = closest_traffic_cone_.erase(it);
    //     it = closest_traffic_cone_.erase(it);

    //     break;
    //   }
    //   else{
    //     ++it;
    //   }
    // }
    // avg_x+=closest_traffic_cone_.begin()->first;
    // cout << closest_traffic_cone_.begin()->first << " " << closest_traffic_cone_.begin()->second << "\n";
    // cout<<"------------------------\n";
    // avg_x/=3.0; avg_y/=2.0;
    // way_point_.push_back({avg_x, avg_y});

    auto it = closest_traffic_cone_.begin();
    cout<<"way point 계산할 때 고려하는 라바콘"<<"\n";
    while(it!=closest_traffic_cone_.end() && distance(closest_traffic_cone_.begin(), it)<=2){
      if(distance(closest_traffic_cone_.begin(), it)==0){
        avg_x+=it->first;
        cout<<it->first<<" "<<-it->second<<"\n";
      }
      else{
        avg_x+=it->first;
        avg_y+=it->second;
        cout<<it->first<<" "<<-it->second<<"\n";
      }
      it++;
    }
    avg_x/=3; avg_y/=2;
    way_point_.push_back({avg_x, -avg_y});

    cout<<"계산된 way point 좌표"<<"\n";
    for(int i=0; i<way_point_.size(); i++){
      cout<<way_point_[i].first<<" "<<way_point_[i].second<<"\n";  
    }
    
    cout<<"계산된 way point까지의 거리"<<"\n";
    dis=sqrt(pow(way_point_[0].first, 2)+pow(way_point_[0].second, 2));
    cout<<"dis : "<<dis<<"\n";
    
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
