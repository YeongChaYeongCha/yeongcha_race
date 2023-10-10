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
  car_steer_=nh.advertise<std_msgs::Float64>("/yeonghun_steering", 100);
  u_turn_condition_=nh.advertise<std_msgs::Bool>("/yeonghun_boolean", 100);
  way_point_marker_=nh.advertise<visualization_msgs::Marker>("/way_point", 1000);

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
  std_msgs::Bool u_turn;
  std_msgs::Float64 steering;
  visualization_msgs::Marker way_point_marker;

  if(cone_boundingbox.markers.size()>=2){
    pair<float, float> positivePoint={0.0, 0.0};
    pair<float, float> negativePoint={0.0, 0.0};
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

    if(RAD2DEG(atan(positivePoint.second/positivePoint.first))<50.0 && positivePoint.first>negativePoint.first){
      u_turn.data=true;
      steering.data=50.0;
    }
    else{
      u_turn.data=false;
    }
    
    cout<<"Steering : "<<steering.data<<"\n";

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
    cout<<"======================="<<"\n";
  }
  
  else if(drum_con_==true && cone_boundingbox.markers.size()==0 && count_bool_==false){
    cout<<"----U-Turn End----"<<"\n";
    for(int i=0; i<60; i++){
      u_turn.data=true;
      count_bool_=true;
      steering.data=60.0;
      car_steer_.publish(steering);
      u_turn_condition_.publish(u_turn);
      usleep(50000);
    }
  }
  if(steering.data!=0.0){
    car_steer_.publish(steering);
  }
  way_point_marker_.publish(way_point_marker);
  u_turn_condition_.publish(u_turn);

  closest_traffic_cone_.clear();
}







