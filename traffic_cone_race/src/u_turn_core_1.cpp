#include <traffic_cone_race_u_turn.h>

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
  drum_cone_condition_=nh.subscribe("/drum_object_condition", 1000, &traffic_cone_race::drum_cone_callback, this);
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

void traffic_cone_race::drum_cone_callback(const std_msgs::Bool &drum_cone_condition){
  drum_con_=drum_cone_condition.data;
}

void traffic_cone_race::cone_callback(const visualization_msgs::MarkerArray &cone_boundingbox){
  //cout<<cone_boundingbox.markers.size()<<"\n";
  race::drive_values drive_msg;
  visualization_msgs::Marker way_point_marker;
  
  if(cone_boundingbox.markers.size()>=2){
    pair<float, float> positivePoint={0.0, 0.0};
    pair<float, float> negativePoint={0.0, 0.0};
    pair<float, float> thirdPoint={0.0, 0.0};
    pair<float, float> goal_point={0.0, 0.0};
    
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

    //라바콘 U턴(우회전)
    Eigen::Matrix2f way_point_mat;
    Eigen::Matrix2f avg_pos_mat;
    Eigen::Matrix2f inc_mat;

    
    if(positivePoint.first>negativePoint.first){
      float inclination = ((negativePoint.second-positivePoint.second)/(negativePoint.first-positivePoint.first));
      avg_pos_mat<<1, (negativePoint.first+positivePoint.first)/2, 1, (negativePoint.second+positivePoint.second)/2;
      inc_mat<<-2.8*(inclination/sqrt(pow(inclination, 2)+1)), 2.8/sqrt(pow(inclination, 2)+1), 1, 1;

      cout<<"##우회전##"<<"\n";
      cout<<positivePoint.first<<" "<<positivePoint.second<<"\n";
      cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";

      way_point_mat=avg_pos_mat*inc_mat;
      goal_point.first=way_point_mat(0,0);
      goal_point.second=way_point_mat(1,1);

      float ldp_deg=-atan(goal_point.second/goal_point.first);
      float steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(goal_point.first - 0.45 , 2) + pow(goal_point.second, 2))));
      // drive_msg.steering=steer;
      drive_msg.steering=40.0;
    }
    // if(negativePoint.first>thirdPoint.first){
    //   float inclination = ((thirdPoint.second-negativePoint.second)/(thirdPoint.first-negativePoint.first));
    //   avg_pos_mat<<1, (thirdPoint.first+negativePoint.first)/2, 1, (thirdPoint.second+negativePoint.second)/2;
    //   inc_mat<<2.1*(inclination/sqrt(pow(inclination, 2)+1)), -2.1/sqrt(pow(inclination, 2)+1), 1, 1;

    //   cout<<"##좌회전##"<<"\n";
    //   cout<<negativePoint.first<<" "<<negativePoint.second<<"\n";
    //   cout<<thirdPoint.first<<" "<<thirdPoint.second<<"\n";

    //   way_point_mat=avg_pos_mat*inc_mat;
    //   goal_point.first=way_point_mat(0,0);
    //   goal_point.second=way_point_mat(1,1);

    //   float ldp_deg=-atan(goal_point.second/goal_point.first);
    //   float steer = -RAD2DEG(atan((2 * sin(ldp_deg) * 1.04) / sqrt(pow(goal_point.first - 0.45 , 2) + pow(goal_point.second, 2))));
    //   steering.data=steer; 
    // }

    cout<<"Steering : "<<drive_msg.steering<<"\n";
    drive_msg.throttle=5;

    if(goal_point.first>0.2){
      way_point_marker.header.frame_id="velodyne";
      way_point_marker.header.stamp=ros::Time::now();
      way_point_marker.ns="cylinder";
      way_point_marker.id=0;
      way_point_marker.type=3;
      way_point_marker.action=0;
      way_point_marker.pose.position.x=goal_point.first;
      way_point_marker.pose.position.y=-goal_point.second;
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
  }
  // else{
  //   drive_msg.steering = 0.0;
  //   drive_msg.throttle = 10;
  // }
  else if(drum_con_==true && count_bool_==false){
    cout<<"----U-Turn End----"<<"\n";
    count_bool_=true;
    for(int i=0; i<100; i++){
      drive_msg.throttle = 5;
      drive_msg.steering = 50.0;
      drive_value_.publish(drive_msg);
      usleep(50000);
    }
  }
  else{
    drive_msg.steering = 0.0;
    drive_msg.throttle = 5;
  }
  drive_value_.publish(drive_msg);
  way_point_marker_.publish(way_point_marker);
  
  cout<<"======================="<<"\n";

  closest_traffic_cone_.clear();
}






