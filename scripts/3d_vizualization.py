#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from tfpose_ros.msg import Person3D

def body_part_valid(body_part):
	if math.isnan(body_part.x) or math.isnan(body_part.y) or math.isnan(body_part.z):
		return False
	else:
		return True 

def body_part_point(body_part, symmetric):
	p = Point()
	if not symmetric:
		p.x = body_part.x;
		p.y = body_part.y;
		p.z = body_part.z;
	else:
		p.x = -body_part.x;
		p.y = body_part.y;
		p.z = body_part.z;		
	return p

def iterate_body_parts(marker, person, symmetric):
	if body_part_valid(person.nose):
		marker.points.append(body_part_point(person.nose, symmetric))
	if body_part_valid(person.neck):
		marker.points.append(body_part_point(person.neck, symmetric))
	if body_part_valid(person.right_shoulder):
		marker.points.append(body_part_point(person.right_shoulder, symmetric))
	if body_part_valid(person.right_elbow):
		marker.points.append(body_part_point(person.right_elbow, symmetric))
	if body_part_valid(person.right_wrist):
		marker.points.append(body_part_point(person.right_wrist, symmetric))
	if body_part_valid(person.left_shoulder):
		marker.points.append(body_part_point(person.left_shoulder, symmetric))
	if body_part_valid(person.left_elbow):
		marker.points.append(body_part_point(person.left_elbow, symmetric))
	if body_part_valid(person.left_wrist):
		marker.points.append(body_part_point(person.left_wrist, symmetric))
	if body_part_valid(person.right_hip):
		marker.points.append(body_part_point(person.right_hip, symmetric))
	if body_part_valid(person.right_knee):
		marker.points.append(body_part_point(person.right_knee, symmetric))
	if body_part_valid(person.right_ankle):
		marker.points.append(body_part_point(person.right_ankle, symmetric))
	if body_part_valid(person.left_hip):
		marker.points.append(body_part_point(person.left_hip, symmetric))
	if body_part_valid(person.left_knee):
		marker.points.append(body_part_point(person.left_knee, symmetric))
	if body_part_valid(person.left_ankle):
		marker.points.append(body_part_point(person.left_ankle, symmetric))
	if body_part_valid(person.right_eye):
		marker.points.append(body_part_point(person.right_eye, symmetric))
	if body_part_valid(person.left_eye):
		marker.points.append(body_part_point(person.left_eye, symmetric))
	if body_part_valid(person.right_ear):
		marker.points.append(body_part_point(person.right_ear, symmetric))
	if body_part_valid(person.left_ear):
		marker.points.append(body_part_point(person.left_ear, symmetric))
	return marker

def drawer(person):
	rospy.loginfo("Person received")

	# Original person
	marker = Marker()

	marker.header.frame_id = "/camera_depth_optical_frame"
	marker.id = person.person_id
	marker.ns = "joints"
	marker.header.stamp = rospy.Time()

	marker.type = Marker.SPHERE_LIST
	marker.action = Marker.ADD
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05

	marker.color.a = 1.0
	marker.color.b = 0.0
	if person.person_id == 0:			
		marker.color.r = 0.0
		marker.color.g = 1.0
	else:			
		marker.color.r = 1.0
		marker.color.g = 0.0

	marker.lifetime = rospy.Duration(0.15)

	marker = iterate_body_parts(marker, person, False)

	point_pub.publish(marker)

	# Symmetric person
	marker_sym = Marker()

	marker_sym.header.frame_id = "/camera_depth_optical_frame"
	marker_sym.id = person.person_id
	marker_sym.ns = "joints"
	marker_sym.header.stamp = rospy.Time()

	marker_sym.type = Marker.SPHERE_LIST
	marker_sym.action = Marker.ADD
	marker_sym.scale.x = 0.05
	marker_sym.scale.y = 0.05
	marker_sym.scale.z = 0.05

	marker_sym.color.a = 1.0
	marker_sym.color.b = 0.0
	if person.person_id == 0:			
		marker_sym.color.r = 1.0
		marker_sym.color.g = 1.0
	else:			
		marker_sym.color.r = 1.0
		marker_sym.color.g = 0.0

	marker_sym.lifetime = rospy.Duration(0.15)

	marker_sym = iterate_body_parts(marker_sym, person, True)

	sym_pub.publish(marker_sym)

if __name__ == '__main__':
	# Initialize Node and Parameters
	rospy.init_node("point_vizualization_3d")
	point_cloud = rospy.get_param("~persons", "/tfpose_ros/transformed_poses_keypoints_3d")
	image = rospy.get_param("~points", "/tfpose_ros/points_3d")

	# Declare Subscribers
	#person_sub = rospy.Subscriber("/tfpose_ros/detected_poses_keypoints_3d", Person3D, drawer)
	person_sub = rospy.Subscriber("/tfpose_ros/transformed_poses_keypoints_3d", Person3D, drawer)

	# Pointcloud publisher topic /tfpose_ros/points_3d
	point_pub = rospy.Publisher('/tfpose_ros/points_3d', Marker, queue_size=0)
	sym_pub = rospy.Publisher('/tfpose_ros/points_3d_sym', Marker, queue_size=0)

	# Image publisher topic /tfpose_ros/lines_3d
	# lines_pub = rospy.Publisher('/tfpose_ros/lines_3d', Marker, queue_size=0)

	# Spin Forever
	rospy.spin()