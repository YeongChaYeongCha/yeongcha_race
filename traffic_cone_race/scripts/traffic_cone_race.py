#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import LineString, Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from race.msg import drive_values

chg_start=False
prev_streering=0.0
pp_streering=0.0
min_dis=0
points_min_x=0.0
points_min_y=0.0
points_max_x=0.0
points_max_y=0.0
points=[]
traffic_cone_pose=[]
intersections=[]
close_goal_point=[]

# def cal_dis(x, y):
# 	global closest_goal_point, min_dis, sec_x, sec_y

# 	#print(x, y)
	
# 	dis=math.sqrt(x**2 + y**2)
# 	if dis<min_dis:
# 		closest_goal_point=[]
# 		closest_goal_point.append([x, y])
# 		closest_goal_point.append([sec_x, sec_y])
# 		sec_x=x
# 		sec_y=y
# 	min_dis=dis
	
def cal_steering():
	global min_dis, closest_goal_point
	wheel_base = 1.04	
      	
	if len(close_goal_point)>1:
	        #cal_steering = np.arctan2(2*closest_goal_point[0][1]*wheel_base, min_dis*min_dis)
		#print("---------",cal_steering)
		print(close_goal_point[0])
		print(min_dis)
		if min_dis!=0:
			return -math.atan((2*close_goal_point[0][1]*wheel_base) / (min_dis*min_dis))*(180/math.pi)
		else:
			return -math.atan((2*close_goal_point[0][1]*wheel_base) / 1)*(180/math.pi)
		
def plot_voronoi(points):
	points_array=np.array(points, dtype=np.float64)
	#print(len(points_array))
	vor=Voronoi(points_array)
	lines=[LineString(vor.vertices[line]) for line in vor.ridge_vertices if -1 not in line]
	intersection_points=[lines[i].intersection(lines[j]) for i in range(len(lines)) for j in range(i+1, len(lines))]

	intersection_points=[point for point in intersection_points if isinstance(point, Point)]

	unique_intersection_points=set()
	for point in intersection_points:
		if isinstance(point, Point):
			point_tuple=(point.x, point.y)
			unique_intersection_points.add(point_tuple)

	return unique_intersection_points

def BoundingBox_callback(msg):
	global points_min_x, points_max_x, points_min_y, points_max_y, intersections
	print(len(msg.markers))	
	for traffic_cone in msg.markers:
		traffic_cone_pose.append((traffic_cone.pose.position.x, traffic_cone.pose.position.y))
	
	points_min_x=min(traffic_cone_pose[i][0] for i in range(len(msg.markers)))
	points_min_y=min(traffic_cone_pose[i][1] for i in range(len(msg.markers)))
	points_max_x=max(traffic_cone_pose[i][0] for i in range(len(msg.markers)))
	points_max_y=max(traffic_cone_pose[i][1] for i in range(len(msg.markers)))

	intersections=plot_voronoi(traffic_cone_pose)

	#print(traffic_cone_pose)
	#print(intersections)
	#print("------------------------------------")

rospy.init_node("traffic_cone_drive", anonymous=False)

pub_way_point=rospy.Publisher("/way_point", MarkerArray, queue_size=1000)
pub_drive_val=rospy.Publisher("/control_value", drive_values, queue_size=1000)
rospy.Subscriber("/object_bounding_boxes", MarkerArray, BoundingBox_callback)

rate=rospy.Rate(5)

way_point_markerarray=MarkerArray()
steering=0.0

while not rospy.is_shutdown():
	way_point_markerarray.markers = []
	
	intersections = [intersection for intersection in intersections if (intersection[0] >= 1.25 and intersection[0] <= 6.0) and abs(intersection[1]) <= 1.5]


	for i, intersection in enumerate(intersections):
		# if intersection[0]<0.7:
		# 	intersections.discard(i)

					

		#if((0.6<=intersection[0]<=points_max_x) and (points_min_y/2<=intersection[1]<=points_max_y/2)):
			#print(intersection)
			#print(type(intersection))

			#cal_dis(intersection[0], intersection[1])

		way_point_marker=Marker()
		way_point_marker.header.frame_id="velodyne"
		way_point_marker.header.stamp=rospy.Time.now()
		way_point_marker.ns="cylinder"
		way_point_marker.id=i
		way_point_marker.type=Marker.CYLINDER
		way_point_marker.action=Marker.ADD
		way_point_marker.pose.position.x=intersection[0]
		way_point_marker.pose.position.y=intersection[1]
		way_point_marker.pose.position.z=-0.25
		way_point_marker.pose.orientation.x=0.0
		way_point_marker.pose.orientation.y=0.0
		way_point_marker.pose.orientation.z=0.0
		way_point_marker.pose.orientation.w=1.0
		way_point_marker.scale.x=0.2
		way_point_marker.scale.y=0.2
		way_point_marker.scale.z=0.2
		way_point_marker.color.a=1.0
		way_point_marker.color.r=0.0
		way_point_marker.color.g=1.0
		way_point_marker.color.b=0.0
		way_point_marker.lifetime = rospy.Duration(0.2)
		way_point_markerarray.markers.append(way_point_marker)
		# else:
		# 	continue
		# 	intersections.discard(i)
    
	close_goal_point = sorted(intersections, key=lambda point: math.sqrt(point[0]**2 + point[1]**2))
	if len(close_goal_point)>1:
		min_dis=math.sqrt(close_goal_point[0][0]**2 + close_goal_point[0][1]**2)

	#print("-----Close Goal Point-----")
	#print(close_goal_point)
	
	if len(close_goal_point)>0:
		if 0.6<=close_goal_point[0][0]<=points_max_x:
			#print(closest_goal_point);
			goal_point_marker=Marker()
			goal_point_marker.header.frame_id="velodyne"
			goal_point_marker.header.stamp=rospy.Time.now()
			goal_point_marker.ns="cylinder"
			goal_point_marker.id=0
			goal_point_marker.type=Marker.CYLINDER
			goal_point_marker.action=Marker.ADD
			goal_point_marker.pose.position.x=close_goal_point[0][0]
			goal_point_marker.pose.position.y=close_goal_point[0][1]
			goal_point_marker.pose.position.z=-0.25
			goal_point_marker.pose.orientation.x=0.0
			goal_point_marker.pose.orientation.y=0.0
			goal_point_marker.pose.orientation.z=0.0
			goal_point_marker.pose.orientation.w=1.0
			goal_point_marker.scale.x=0.2
			goal_point_marker.scale.y=0.2
			goal_point_marker.scale.z=0.2
			goal_point_marker.color.a=1.0
			goal_point_marker.color.r=0.0
			goal_point_marker.color.g=0.0
			goal_point_marker.color.b=1.0
			goal_point_marker.lifetime = rospy.Duration(0.3)
			way_point_markerarray.markers.append(goal_point_marker)
	
	if len(close_goal_point)>1:
		# steer=cal_steering()

		# print(abs(steer-prev_streering))
		# print(abs(steer-pp_streering))

		# if 1.5<abs(steer-prev_streering)<3.5 and 1.5<abs(steer-prev_streering)<3.5:
		# 	print("$$$$$$$$$$$$$$$$$$")
		# 	print("$$$$$$$$$$$$$$$$$$")
		# 	print("$$$$$$$$$$$$$$$$$$")
		# 	chg_start=True
		# if chg_start==True:
		# 	steer*=1.1
		# 	chg_start=False

		# steering=np.clip(steer, -30, 30)
		# pp_streering=prev_streering
		# prev_streering=steering
		steering=np.clip(cal_steering(), -30, 30)
		steering*=1.23
	
	print("-----Steering-----")
	print(steering)
	print("==============================")
	drive_msg=drive_values()
	drive_msg.steering=steering
	drive_msg.throttle=7
	pub_drive_val.publish(drive_msg)
	#min_dis=0
	    
	pub_way_point.publish(way_point_markerarray)

	intersections=[]
	traffic_cone_pose=[]
	rate.sleep()
