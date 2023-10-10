#include <traffic_cone_race.h>

bool cmp_deg(pair<float, float> cone_0, pair<float, float> cone_1){
  float angle_0=RAD2DEG(atan(cone_0.second/cone_0.first));
  float angle_1=RAD2DEG(atan(cone_1.second/cone_1.first));
  
  return angle_0<angle_1;
}

bool cmp_dis(pair<float, float> cone_0, pair<float, float> cone_1){
  return sqrt(pow(cone_0.first, 2) + pow(cone_0.second, 2)) < sqrt(pow(cone_1.first, 2) + pow(cone_1.second, 2));
}

traffic_cone_race::traffic_cone_race(ros::NodeHandle &nh){
  cone_pos_=nh.subscribe("/object_bounding_boxes", 1000, &traffic_cone_race::cone_callback, this);
  way_point_marker_=nh.advertise<visualization_msgs::Marker>("/way_point", 1000);
  drive_value_=nh.advertise<race::drive_values>("/control_value", 1000);
  way_point_angle_=nh.advertise<visualization_msgs::Marker>("/way_point_angle", 1000);

  nh.getParam("curve_constant", curve_constant_);
	ROS_INFO("curve_constant : %f", curve_constant_);
  nh.getParam("max_speed", max_speed_);
	ROS_INFO("max_speed : %d", max_speed_);
  nh.getParam("min_speed", min_speed_);
	ROS_INFO("min_speed : %d", min_speed_);

  ros::spin();
}

traffic_cone_race::~traffic_cone_race(){}

void traffic_cone_race::Spin(){}

void traffic_cone_race::cone_callback(const visualization_msgs::MarkerArray &cone_boundingbox){
  
  if(cone_boundingbox.markers.size()>=3){
    float avg_x = 0, avg_y = 0;
    pair<float, float> positivePoint={0.0, 0.0};
    pair<float, float> negativePoint={0.0, 0.0};
    pair<float, float> thirdPoint={0.0, 0.0};
    
    for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
      float cone_pos_x=iter->pose.position.x;
      float cone_pos_y=iter->pose.position.y;

      cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
      
      closest_traffic_cone_.push_back({cone_pos_x, -cone_pos_y});
    }
    cout<<"---------------------"<<"\n";

    sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp_deg);

    negativePoint=*(closest_traffic_cone_.begin());
    positivePoint=*(closest_traffic_cone_.end()-1);

    closest_traffic_cone_.erase(remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), positivePoint), closest_traffic_cone_.end());
    closest_traffic_cone_.erase(remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), negativePoint), closest_traffic_cone_.end());

    sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp_dis);

    thirdPoint=*(closest_traffic_cone_.begin());

    cout<<"--Targeting Rubber Cone--"<<"\n";
    cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
    cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
    cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";
	  cout<<"---------------------"<<"\n";

    Eigen::Matrix2f way_point_mat;
    Eigen::Matrix2f avg_pos_mat;
    Eigen::Matrix2f inc_mat;

    if(positivePoint.first>thirdPoint.first){
      float inclination = ((thirdPoint.second-positivePoint.second)/(thirdPoint.first-positivePoint.first));
      avg_pos_mat<<1, (thirdPoint.first+positivePoint.first)/2, 1, (thirdPoint.second+positivePoint.second)/2;
      inc_mat<<-curve_constant_*(inclination/sqrt(pow(inclination, 2)+1)), curve_constant_/sqrt(pow(inclination, 2)+1), 1, 1;

      cout<<"##우회전##"<<"\n";
      cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
      cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";

      way_point_mat=avg_pos_mat*inc_mat;
      way_point_.push_back({way_point_mat(0,0), way_point_mat(1,1)});
    }

    else if(negativePoint.first>thirdPoint.first){
      float inclination = ((thirdPoint.second-negativePoint.second)/(thirdPoint.first-negativePoint.first));
      avg_pos_mat<<1, (thirdPoint.first+negativePoint.first)/2, 1, (thirdPoint.second+negativePoint.second)/2;
      inc_mat<<curve_constant_*(inclination/sqrt(pow(inclination, 2)+1)), -curve_constant_/sqrt(pow(inclination, 2)+1), 1, 1;

      cout<<"##좌회전##"<<"\n";
      cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
      cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";

      way_point_mat=avg_pos_mat*inc_mat;
      way_point_.push_back({way_point_mat(0,0), way_point_mat(1,1)});
    }

    // 7 : 5 -> 1.8

    else{
      avg_x = (positivePoint.first + negativePoint.first + thirdPoint.first) / 3;
      avg_y = (positivePoint.second + negativePoint.second) / 2;

      way_point_.push_back({avg_x, avg_y});

      cout<<"--Goal Point--"<<"\n";
      cout << avg_x << " " << avg_y << "\n";
      cout<<"---------------------"<<"\n";
    }

    race::drive_values drive_msg;
    float ldp_deg=-atan(way_point_[0].second/way_point_[0].first);
    // float steer=RAD2DEG(atan(way_point_[0].second/way_point_[0].first))*drive_ratio_;
    //09.22 연습주행 : 0.45보다 작게도 해볼 것
    float steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(way_point_[0].first - 0.50 , 2) + pow(way_point_[0].second, 2))));
    // vel(8) 0.30 : 곡률이 심한 커브에서 버거 | 0.40 : 조향각 약간 부족함. | 0.45 : 22시즌 카울 사용 시 적합 | 0.53 : 카울 미사용시 적합
    drive_msg.steering = steer;
    cout<<"Steering : "<<steer<<"\n";
    drive_msg.throttle = (abs(drive_msg.steering) < 15.0) ? max_speed_ : min_speed_;
    // drive_msg.throttle=7;
    
    visualization_msgs::Marker way_point_marker;
    visualization_msgs::Marker way_point_angle;

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

      way_point_angle.header.frame_id="velodyne";
      way_point_angle.header.stamp=ros::Time::now();
      way_point_angle.ns="checker";
      way_point_angle.id=0;
      way_point_angle.type=visualization_msgs::Marker::LINE_STRIP;
      way_point_angle.action=visualization_msgs::Marker::ADD;
      way_point_angle.pose.orientation.x=0.0;
      way_point_angle.pose.orientation.y=0.0;
      way_point_angle.pose.orientation.z=0.0;
      way_point_angle.pose.orientation.w=1.0;
      way_point_angle.scale.x=0.06;
      way_point_angle.color.a=1.0;
      way_point_angle.color.r=1.0;
      way_point_angle.color.g=1.0;
      way_point_angle.color.b=0.0;
      
      geometry_msgs::Point square_one, dir_steer;
      square_one.x=0.0;
      square_one.y=0.0;
      square_one.z=0.3;
      dir_steer.x=way_point_[0].first*0.3;
      dir_steer.y=-way_point_[0].second*0.3;
      dir_steer.z=0.3;

      way_point_angle.points.push_back(square_one);
      way_point_angle.points.push_back(dir_steer);
      way_point_angle.lifetime=ros::Duration(0.2);
    }

    drive_value_.publish(drive_msg);
    way_point_marker_.publish(way_point_marker);
    way_point_angle_.publish(way_point_angle);
    
    cout<<"======================="<<"\n";
  }

  closest_traffic_cone_.clear();
  way_point_.clear();
}