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
    pair<float, float> firNegativePoint={0.0, 0.0};
    pair<float, float> secNegativePoint={0.0, 0.0};
    pair<float, float> thirNegativePoint={0.0, 0.0};
    
    for(auto iter=cone_boundingbox.markers.begin(); iter!=cone_boundingbox.markers.end(); iter++){
      float cone_pos_x=iter->pose.position.x;
      float cone_pos_y=iter->pose.position.y;

      cout<<cone_pos_x<<" "<<-cone_pos_y<<"\n";
      
      closest_traffic_cone_.push_back({cone_pos_x, -cone_pos_y});
    }
    cout<<"---------------------"<<"\n";

    sort(closest_traffic_cone_.begin(), closest_traffic_cone_.end(), cmp);
    firNegativePoint=*closest_traffic_cone_.begin();
    secNegativePoint=*(closest_traffic_cone_.begin()+1);
    thirNegativePoint=*(closest_traffic_cone_.begin()+2);
    
    cout<<firNegativePoint.first<<" "<<firNegativePoint.second<<"\n";
    cout<<secNegativePoint.first<<" "<<secNegativePoint.second<<"\n";
    cout<<thirNegativePoint.first<<" "<<thirNegativePoint.second<<"\n";

    //float fir_inclination_angle=RAD2DEG(atan(firNegativePoint.second-secNegativePoint.second/firNegativePoint.first-secNegativePoint.first));
    //float sec_inclination_angle=RAD2DEG(atan(secNegativePoint.second-thirNegativePoint.second/secNegativePoint.first-thirNegativePoint.first));

    race::drive_values drive_msg;
    //drive_msg.steering = 0.5*(fir_inclination_angle+sec_inclination_angle);
    drive_msg.steering=RAD2DEG(atan((firNegativePoint.second-secNegativePoint.second)/(firNegativePoint.first-secNegativePoint.first)));
    cout<<"Steering : "<<drive_msg.steering<<"\n";
    drive_msg.throttle=throttle_;
    
    //visualization_msgs::Marker way_point_marker;

    /*
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
    */

    drive_value_.publish(drive_msg);
    //way_point_marker_.publish(way_point_marker);
    
    cout<<"======================="<<"\n";
  }

  closest_traffic_cone_.clear();
  //way_point_.clear();
}





