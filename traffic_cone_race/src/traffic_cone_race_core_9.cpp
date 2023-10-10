#include <traffic_cone_race.h>

bool cmp(pair<float, float> cone_0, pair<float, float> cone_1){
  float angle_0=RAD2DEG(atan(cone_0.second/cone_0.first));
  float angle_1=RAD2DEG(atan(cone_1.second/cone_1.first));
  
  if(abs(angle_0-angle_1)<-45)
    return true;
  else
    return false;
}

traffic_cone_race::traffic_cone_race(ros::NodeHandle &nh){
  cone_left_pos_=nh.subscribe("/object_bounding_left_boxes", 1000, &traffic_cone_race::left_cone_callback, this);
  cone_right_pos_=nh.subscribe("/object_bounding_right_boxes", 1000, &traffic_cone_race::right_cone_callback, this);
  
  way_point_marker_=nh.advertise<visualization_msgs::Marker>("/way_point", 1000);
  drive_value_=nh.advertise<race::drive_values>("/control_value", 1000);
  way_point_angle_=nh.advertise<visualization_msgs::Marker>("/way_point_angle", 1000);

  ros::spin();
}

traffic_cone_race::~traffic_cone_race(){}

void traffic_cone_race::Spin(){}

void traffic_cone_race::left_cone_callback(const visualization_msgs::MarkerArray &left_cone_boundingbox){
  cout<<"Left_Cone_Size : "<<left_cone_boundingbox.markers.size()<<"\n";

  for(auto iter=left_cone_boundingbox.markers.begin(); iter!=left_cone_boundingbox.markers.end(); iter++){
    float cone_pos_x=iter->pose.position.x;
    float cone_pos_y=iter->pose.position.y;

    cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
		left_traffic_cone_.push_back({cone_pos_x, -cone_pos_y});
	}
  cout<<"-------------------"<<"\n";
  // if(cmp(*left_traffic_cone_.begin(), *(left_traffic_cone_.end()-1))==true&&(sqrt(pow((left_traffic_cone_.begin())->first,2)+pow(left_traffic_cone_.begin()->second,2))>sqrt(pow((left_traffic_cone_.begin()+1)->first,2)+pow((left_traffic_cone_.begin()+1)->second,2))) && left_traffic_cone_.size()>=2){
  //   cout<<"Swap"<<"\n";
  //   swap(left_traffic_cone_.begin()->first, (left_traffic_cone_.begin() + 1)->first);
  //   swap(left_traffic_cone_.begin()->second, (left_traffic_cone_.begin() + 1)->second);
  // }
  // cout<<"-------------------"<<"\n";
}

void traffic_cone_race::right_cone_callback(const visualization_msgs::MarkerArray &right_cone_boundingbox){
  cout<<"Right_Cone_Size : "<<right_cone_boundingbox.markers.size()<<"\n";

  for(auto iter=right_cone_boundingbox.markers.begin(); iter!=right_cone_boundingbox.markers.end(); iter++){
    float cone_pos_x=iter->pose.position.x;
    float cone_pos_y=iter->pose.position.y;

    cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
		right_traffic_cone_.push_back({cone_pos_x, -cone_pos_y});
	}
  // if(cmp(*right_traffic_cone_.begin(), *(right_traffic_cone_.end()-1))==true&&(sqrt(pow((right_traffic_cone_.begin())->first,2)+pow(right_traffic_cone_.begin()->second,2))>sqrt(pow((right_traffic_cone_.begin()+1)->first,2)+pow((right_traffic_cone_.begin()+1)->second,2))) && right_traffic_cone_.size()>=2){
  //   cout<<"Swap"<<"\n";cout<<"Swap"<<"\n";
  //   swap(right_traffic_cone_.begin()->first, (right_traffic_cone_.begin() + 1)->first);
  //   swap(right_traffic_cone_.begin()->second, (right_traffic_cone_.begin() + 1)->second);
  // }
  if(left_traffic_cone_.size()>0 || right_traffic_cone_.size()>0){
    process_data();
  }
  cout<<"-------------------"<<"\n";
}

void traffic_cone_race::process_data(){
  float steer;
  race::drive_values drive_msg;
  visualization_msgs::Marker way_point_marker;

  if(left_traffic_cone_.size()==0 && right_traffic_cone_.size()>=2){
    float inclination = (((right_traffic_cone_.end()-1)->second-(right_traffic_cone_.end()-2)->second)/((right_traffic_cone_.end()-1)->first-(right_traffic_cone_.end()-2)->first));
    float curve_avg_x=((right_traffic_cone_.end()-1)->first+(right_traffic_cone_.end()-2)->first)/2;
	  float curve_avg_y=((right_traffic_cone_.end()-1)->second+(right_traffic_cone_.end()-2)->second)/2;

    cout<<"##좌회전##"<<"\n";
    cout<<(right_traffic_cone_.end()-1)->first<<" "<<(right_traffic_cone_.end()-1)->second<<"\n";
    cout<<(right_traffic_cone_.end()-2)->first<<" "<<(right_traffic_cone_.end()-2)->second<<"\n";
    // cout<<"--Goal Point--"<<"\n";
    // cout << 1.5*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x << " " << -1.5/sqrt(pow(inclination, 2)+1)+curve_avg_y << "\n";
    // cout<<"---------------------"<<"\n";

    way_point_.push_back({1.6*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x, -1.6/sqrt(pow(inclination, 2)+1)+curve_avg_y});
  }

  else if(right_traffic_cone_.size()==0 && left_traffic_cone_.size()>=2){
    float inclination = (((left_traffic_cone_.end()-1)->second-(left_traffic_cone_.end()-2)->second)/((left_traffic_cone_.end()-1)->first-(left_traffic_cone_.end()-2)->first));
    float curve_avg_x=((left_traffic_cone_.end()-1)->first+(left_traffic_cone_.end()-2)->first)/2;
	  float curve_avg_y=((left_traffic_cone_.end()-1)->second+(left_traffic_cone_.end()-2)->second)/2;

    cout<<"##우회전##"<<"\n";
    cout<<(left_traffic_cone_.end()-1)->first<<" "<<(left_traffic_cone_.end()-1)->second<<"\n";
    cout<<(left_traffic_cone_.end()-2)->first<<" "<<(left_traffic_cone_.end()-2)->second<<"\n";
    // cout<<"--Goal Point--"<<"\n";
    // cout << -1.5*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x << " " << 1.5/sqrt(pow(inclination, 2)+1)+curve_avg_y << "\n";
    // cout<<"---------------------"<<"\n";

    way_point_.push_back({-1.6*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x, 1.6/sqrt(pow(inclination, 2)+1)+curve_avg_y});
  }

  // if(left_traffic_cone_.size()>0 || right_traffic_cone_.size()>0){
  else if(left_traffic_cone_.size()>0 || right_traffic_cone_.size()>0){
    if((left_traffic_cone_.end()-1)->first<right_traffic_cone_.begin()->first){
      float inclination = (((left_traffic_cone_.end()-1)->second-right_traffic_cone_.begin()->second)/((left_traffic_cone_.end()-1)->first-right_traffic_cone_.begin()->first));
      float curve_avg_x=((left_traffic_cone_.end()-1)->first+right_traffic_cone_.begin()->first)/2;
      float curve_avg_y=((left_traffic_cone_.end()-1)->second+right_traffic_cone_.begin()->second)/2;

      cout<<"##우회전##"<<"\n";
      cout<<(left_traffic_cone_.end()-1)->first<<" "<<(left_traffic_cone_.end()-1)->second<<"\n";
      cout<<right_traffic_cone_.begin()->first<<" "<<right_traffic_cone_.begin()->second<<"\n";
      // cout<<"--Goal Point--"<<"\n";
      // cout << -1.5*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x << " " << 1.5/sqrt(pow(inclination, 2)+1)+curve_avg_y << "\n";
      // cout<<"---------------------"<<"\n";

      way_point_.push_back({-1.6*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x, 1.6/sqrt(pow(inclination, 2)+1)+curve_avg_y});
    }
    else if((right_traffic_cone_.end()-1)->first<left_traffic_cone_.begin()->first){
      float inclination = (((right_traffic_cone_.end()-1)->second-left_traffic_cone_.begin()->second)/((right_traffic_cone_.end()-1)->first-left_traffic_cone_.begin()->first));
      float curve_avg_x=((right_traffic_cone_.end()-1)->first+left_traffic_cone_.begin()->first)/2;
      float curve_avg_y=((right_traffic_cone_.end()-1)->second+left_traffic_cone_.begin()->second)/2;

      cout<<"##좌회전##"<<"\n";
      cout<<(right_traffic_cone_.end()-1)->first<<" "<<(right_traffic_cone_.end()-1)->second<<"\n";
      cout<<left_traffic_cone_.begin()->first<<" "<<left_traffic_cone_.begin()->second<<"\n";
      // cout<<"--Goal Point--"<<"\n";
      // cout << -1.5*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x << " " << 1.5/sqrt(pow(inclination, 2)+1)+curve_avg_y << "\n";
      // cout<<"---------------------"<<"\n";

      way_point_.push_back({1.6*(inclination/sqrt(pow(inclination, 2)+1))+curve_avg_x, -1.6/sqrt(pow(inclination, 2)+1)+curve_avg_y});
    }
    else{
      pair<float, float> positivePoint={0.0, 0.0};
      pair<float, float> negativePoint={0.0, 0.0};

      negativePoint=*left_traffic_cone_.begin();
      positivePoint=*right_traffic_cone_.begin();
      cout<<"------------------"<<"\n";

      cout<<"--Targeting Rubber Cone--"<<"\n";
      cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
      cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
      cout<<"---------------------"<<"\n";
      float avg_x = 0, avg_y = 0;

      avg_x=(positivePoint.first+negativePoint.first)/2;
      avg_y=(positivePoint.second+negativePoint.second)/2;

      way_point_.push_back({avg_x, avg_y});
    }
  }
  float ldp_deg=-atan(way_point_[0].second/way_point_[0].first);
  steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(way_point_[0].first - 0.41 , 2) + pow(way_point_[0].second, 2))));
  // 09.22 연습주행 : 0.45보다 더 작은 수를 빼서 돌려봐줭(예 0.40)
  
  cout<<"--Goal Point--"<<"\n";
  cout<<way_point_[0].first<<" "<<way_point_[0].second<<"\n";
  cout<<"---------------------"<<"\n";

  // vel(8) 0.30 : 곡률이 심한 커브에서 버거 | 0.40 : 조향각 약간 부족함. | 0.45 : 22시즌 카울 사용 시 적합 | 0.53 : 카울 미사용시 적합
  drive_msg.steering = steer;
  cout<<"Steering : "<<steer<<"\n";
  drive_msg.throttle = (abs(drive_msg.steering) < 15.0) ? 8 : 6;
  if(way_point_[0].first>0.2){
      way_point_marker.header.frame_id="velodyne";
      way_point_marker.header.stamp=ros::Time::now();
      way_point_marker.ns="cylinder";
      way_point_marker.id=0;
      way_point_marker.type=3;
      way_point_marker.action=0;
      way_point_marker.pose.position.x=way_point_[0].first;
      way_point_marker.pose.position.y=-way_point_[0].second;
      way_point_marker.pose.position.z=0.0;
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
  cout<<"========================"<<"\n";
  drive_value_.publish(drive_msg);
  way_point_marker_.publish(way_point_marker);

  way_point_.clear();
  left_traffic_cone_.clear();
  right_traffic_cone_.clear();
}