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
    pair<float, float> thirdPoint={0.0, 0.0};
    
    for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
      float cone_pos_x=iter->pose.position.x;
      float cone_pos_y=iter->pose.position.y;

      cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
      
      closest_traffic_cone_.push_back({cone_pos_x, -cone_pos_y});
    }
    cout<<"---------------------"<<"\n";

    sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp);
    
    if((sqrt(pow((closest_traffic_cone_.begin())->first,2)+pow(closest_traffic_cone_.begin()->second,2))>sqrt(pow((closest_traffic_cone_.begin()+1)->first,2)+pow((closest_traffic_cone_.begin()+1)->second,2))) && closest_traffic_cone_.size()>=5){
      cout<<"--First Case--"<<endl;
      cout<<"---------------------"<<"\n";
      negativePoint=*(closest_traffic_cone_.begin()+1);
      positivePoint=*(closest_traffic_cone_.end()-1);
      thirdPoint=*(closest_traffic_cone_.end()-2);
	  }
    else if((sqrt(pow((closest_traffic_cone_.end()-1)->first,2)+pow((closest_traffic_cone_.end()-1)->second,2))>sqrt(pow((closest_traffic_cone_.end()-2)->first,2)+pow((closest_traffic_cone_.end()-2)->second,2))) && closest_traffic_cone_.size()>=5){
      cout<<"--Second Case--"<<endl;
      cout<<"---------------------"<<"\n";
      negativePoint=*(closest_traffic_cone_.begin());
      positivePoint=*(closest_traffic_cone_.end()-2);
      thirdPoint=*(closest_traffic_cone_.begin()+1);
    }
    else{
      cout<<"--Third Case--"<<endl;
      cout<<"---------------------"<<"\n";
      negativePoint=*closest_traffic_cone_.begin();
      positivePoint=*(closest_traffic_cone_.end()-1);
      
      remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), positivePoint);
      remove(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), negativePoint);
      
      if(closest_traffic_cone_.size()>=4){
        thirdPoint=(sqrt(pow(closest_traffic_cone_.begin()->first,2)+pow(closest_traffic_cone_.begin()->second,2))>sqrt(pow((closest_traffic_cone_.end()-1)->first,2)+pow((closest_traffic_cone_.end()-1)->second,2))) ? *(closest_traffic_cone_.end()-1) : *(closest_traffic_cone_.begin());
      }
      else{
        thirdPoint=*closest_traffic_cone_.begin();
      }
    }

    cout<<"--Targeting Rubber Cone--"<<"\n";
    cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
    cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
    cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";
	  cout<<"---------------------"<<"\n";
    float avg_x = 0, avg_y = 0;

    avg_x = (positivePoint.first + negativePoint.first + thirdPoint.first) / 3;
    avg_y = (positivePoint.second + negativePoint.second) / 2;

    if(sqrt(pow(thirdPoint.first-avg_x,2)+pow(thirdPoint.second-avg_y,2))<0.6){
      if((sqrt(pow(positivePoint.first,2)+pow(positivePoint.second,2))<sqrt(pow(negativePoint.first,2)+pow(negativePoint.second,2)))){
        //좌회전
        cout<<"--Right--"<<"\n";
        avg_x=1.0; avg_y=-1.0;
        cout<<"---------------------"<<"\n";
      }
      else{
        //우회전
        cout<<"--Left--"<<"\n";
        avg_x=1.0; avg_y=1.0;
        cout<<"---------------------"<<"\n";
      }
    }

    way_point_.push_back({avg_x, avg_y});

    cout<<"--Goal Point--"<<"\n";
    cout << avg_x << " " << avg_y << "\n";
    cout<<"---------------------"<<"\n";

    race::drive_values drive_msg;
    float ldp_deg=-atan(way_point_[0].second/way_point_[0].first);
    // float steer=RAD2DEG(atan(way_point_[0].second/way_point_[0].first))*drive_ratio_;
    //09.22 연습주행 : 0.45보다 작게도 해볼 것
    float steer=0.0;
    if(sqrt(pow(avg_x,2)+pow(avg_y,2))>0.8){
      steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(way_point_[0].first - 0.45 , 2) + pow(way_point_[0].second, 2))));
    }
    else{
      cout<<"Revise Steer"<<"\n";
      steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / 10.0));
    }
    // vel(8) 0.30 : 곡률이 심한 커브에서 버거 | 0.40 : 조향각 약간 부족함. | 0.45 : 22시즌 카울 사용 시 적합 | 0.53 : 카울 미사용시 적합
    drive_msg.steering = steer;
    cout<<"Steering : "<<steer<<"\n";
    drive_msg.throttle = (abs(drive_msg.steering) < 15.0) ? 7 : 5;
    
    visualization_msgs::Marker way_point_marker;
    //visualization_msgs::Marker way_point_angle;

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

    drive_value_.publish(drive_msg);
    way_point_marker_.publish(way_point_marker);
    
    cout<<"======================="<<"\n";
    avg_x=0; avg_y=0;
  }

  closest_traffic_cone_.clear();
  way_point_.clear();
}







