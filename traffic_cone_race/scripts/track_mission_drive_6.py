#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from jsk_recognition_msgs.msg import BoundingBoxArray
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import LineString, Point
from visualization_msgs.msg import MarkerArray, Marker
from race.msg import drive_values
import numpy as np
import math

totall_object_num=0
stop_sign=False
points=[]
intersections=[]
closest_goal_point=[]
min_dis=0
sec_x=0
sec_y=0

def cal_dis(x, y):
	global closest_goal_point, min_dis, sec_x, sec_y

	#print(x, y)
	
	dis=math.sqrt(x**2 + y**2)
	if dis<min_dis:
		closest_goal_point=[]
		closest_goal_point.append([x, y])
		closest_goal_point.append([sec_x, sec_y])
		sec_x=x
		sec_y=y
	min_dis=dis
	
def cal_steering():
	global min_dis, closest_goal_point
	wheel_base = 1.04	
      	
	if len(closest_goal_point)>1:
	        #cal_steering = np.arctan2(2*closest_goal_point[0][1]*wheel_base, min_dis*min_dis)
		#print("---------",cal_steering)
		if min_dis!=0:
			print("--------")
			return math.atan((2*closest_goal_point[0][1]*wheel_base) / (min_dis*min_dis))*(180/math.pi)
		else:
			print("--------")
			return math.atan((2*closest_goal_point[0][1]*wheel_base) / 1)*(180/math.pi)
		
def plot_voronoi(points):
	points_array=np.array(points, dtype=np.float64)
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
	global points, totall_object_num, stop_sign, intersections
	global points_min_x, points_max_x, points_min_y, points_max_y
	points=[]
	object_num=0
	stop_sign=False
	totall_object_num=len(msg.boxes)
        

	object_x_pos=[]
	object_y_pos=[]
	object_z_pos=[]

	object_y_dem=[]
	object_z_dem=[]

	for i in range(totall_object_num):
		object_x_pos.append(msg.boxes[i].pose.position.x)
		object_y_pos.append(msg.boxes[i].pose.position.y)
		object_z_pos.append(msg.boxes[i].pose.position.z)

		object_y_dem.append(msg.boxes[i].dimensions.y)
		object_z_dem.append(msg.boxes[i].dimensions.z)

		if (1.0<object_x_pos[i]<10.0) and (abs(object_y_pos[i])<=3.5 and object_z_pos[i]>-1.5):
			if 0.1<object_z_dem[i]<1.0:
				object_num+=1
				stop_sign=True
				#rospy.loginfo("%d. X : %f  |  Y : %f", object_num, object_x_pos[i], object_y_pos[i])
				points.append((object_x_pos[i], object_y_pos[i]))
	#print("points : ", len(points))
	if len(points)>4:
		#print("points : ")
		#print(points)
		points_min_x=min(points[i][0] for i in range(len(points)))
		points_min_y=min(points[i][1] for i in range(len(points)))
		points_max_x=max(points[i][0] for i in range(len(points)))
		points_max_y=max(points[i][1] for i in range(len(points)))
		intersections=plot_voronoi(points)

rospy.init_node("object_detect_emergency_stop", anonymous=False)

rate=rospy.Rate(3)

traffic_cone_markerarray=MarkerArray()

while not rospy.is_shutdown():

	print("Totall Goal Point")
	for i, intersection in enumerate(intersections):
		if((points_min_x<=intersection[0]<=points_max_x) and (points_min_y<=intersection[1]<=points_max_y)):
			print(intersection[0], intersection[1])
			#print(type(intersection))

			cal_dis(intersection[0], intersection[1])

			traffic_cone_marker=Marker()
			traffic_cone_marker.header.frame_id="velodyne"
			traffic_cone_marker.header.stamp=rospy.Time.now()
			traffic_cone_marker.ns="cylinder"
			traffic_cone_marker.id=i
			traffic_cone_marker.type=Marker.CYLINDER
			traffic_cone_marker.action=Marker.ADD
			traffic_cone_marker.pose.position.x=intersection[0]
			traffic_cone_marker.pose.position.y=intersection[1]
			traffic_cone_marker.pose.position.z=-0.25
			traffic_cone_marker.pose.orientation.x=0.0
			traffic_cone_marker.pose.orientation.y=0.0
			traffic_cone_marker.pose.orientation.z=0.0
			traffic_cone_marker.pose.orientation.w=1.0
			traffic_cone_marker.scale.x=0.2
			traffic_cone_marker.scale.y=0.2
			traffic_cone_marker.scale.z=0.2
			traffic_cone_marker.color.a=1.0
			traffic_cone_marker.color.r=1.0
			traffic_cone_marker.color.g=0.0
			traffic_cone_marker.color.b=0.0

			traffic_cone_markerarray.markers.append(traffic_cone_marker)
		else:
			continue

	sec_x=0
	sec_y=0
	
	min_dis=0
	print("Closest Goal Point")
	
	if len(closest_goal_point)>0:
		print(closest_goal_point)
		goal_point_marker=Marker()
		goal_point_marker.header.frame_id="velodyne"
		goal_point_marker.header.stamp=rospy.Time.now()
		goal_point_marker.ns="cylinder"
		goal_point_marker.id=0
		goal_point_marker.type=Marker.CYLINDER
		goal_point_marker.action=Marker.ADD
		goal_point_marker.pose.position.x=closest_goal_point[0][0]
		goal_point_marker.pose.position.y=closest_goal_point[0][1]
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
		traffic_cone_markerarray.markers.append(goal_point_marker)
	
	pub_drive_val=rospy.Publisher("/control_value", drive_values, queue_size=1000)
	pub_traffic_cone=rospy.Publisher("/traffic_cone_marker", MarkerArray, queue_size=1000)
	rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, BoundingBox_callback)

	steering=cal_steering()
	print(steering)

	drive_msg=drive_values()
	drive_msg.steering=steering
	drive_msg.throttle=3

	print("--------------------------")
	
	pub_drive_val.publish(drive_msg)
	pub_traffic_cone.publish(traffic_cone_markerarray)
	intersections=[]
	#closest_goal_point=[]
	
	rate.sleep()
