#!/usr/bin/env python
import rospy
import pcl
import math

import message_filters
import ros_numpy as np
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from tfpose_ros.msg import Persons, Persons3D, Person3D, BodyPartElm3D
from tfpose_ros.srv import *

def getNANBodypart(part_id):
	body_part = BodyPartElm3D()
	body_part.x = float('NaN')
	body_part.y = float('NaN')
	body_part.z = float('NaN')
	body_part.part_id = part_id
	body_part.confidence = float('NaN')
	return body_part

def print_body_part(body_part_new):
	print("Index: " + str(body_part_new.part_id))
	print("X: " + str(body_part_new.x))
	print("Y: " + str(body_part_new.y))
	print("Z: " + str(body_part_new.z))

def get3dcoordinates(body_part, pc, width, height):
	body_part_new = BodyPartElm3D()

	# Include 2D confidence 3D detections message
	body_part_new.confidence = body_part.confidence
	body_part_new.part_id = body_part.part_id

	if math.isnan(body_part_new.confidence) or body_part_new.confidence <= 0 or (body_part.x == 0 and body_part.y == 0) or body_part.x > width or body_part.y > height \
	   or math.isnan(body_part.x) or math.isnan(body_part.y):
		body_part_new.x = float('NaN')
		body_part_new.y = float('NaN')
		body_part_new.z = float('NaN')
	else:
		# Get keypoint pixel coordinates
		x_pixel = int(body_part.x * width + 0.5)
		y_pixel = int(body_part.y * height + 0.5)

		# Vector for storing possible world coordinates of indices in the cluster
		possible_x = []
		possible_y = []
		possible_z = []

		# Range of pixel clustering
		index_range = 1

		# Get coordinates if are valid
		# TODO: Check x_search and y_search < 0 and > width/height
		for x_search in range(x_pixel - index_range, x_pixel + index_range + 1):
			for y_search in range(y_pixel - index_range, y_pixel + index_range + 1):
				if not math.isnan(pc['x'][y_search][x_search]) and not math.isnan(pc['y'][y_search][x_search]) and not math.isnan(pc['z'][y_search][x_search]):
					possible_x.append(pc['x'][y_search][x_search])
					possible_y.append(pc['y'][y_search][x_search])
					possible_z.append(pc['z'][y_search][x_search])

		if len(possible_x) == 0 or len(possible_y) == 0 or len(possible_z) == 0:
			body_part_new.x = float('NaN')
			body_part_new.y = float('NaN')
			body_part_new.z = float('NaN')
		else:
			# Make the mean for each coordinate
			body_part_new.x = sum(possible_x) / float(len(possible_x))
			body_part_new.y = sum(possible_y) / float(len(possible_y))
			body_part_new.z = sum(possible_z) / float(len(possible_z))

	#print_body_part(body_part_new)

	return body_part_new

def converter3d_callback(pointcloud, image):
	rospy.loginfo("Image and Pointcloud received")
	rospy.wait_for_service('get_persons')

	# Publish camera output for vizualization
	pc_pub.publish(pointcloud)
	img_pub.publish(image)

	# Preprocess pointcloud	(Turn into np array with height x width dimension, where each cell has x, y, z)
	pc = np.numpify(pointcloud)

	# REALLY AWFUL ALTERNATIVE TO PCL IN PYTHON
	# pc_generator = pc2.read_points(pointcloud, field_names=("x", "y", "z"))
	# pc_list = list(pc_generator)

	# Get 2D persons
	try:
		get_persons = rospy.ServiceProxy('get_persons', GetPersons)
		response = get_persons(image)

		persons = response.persons
		image_width = persons.image_w
		image_height = persons.image_h

		# Do all the stuff here
		for i, person in enumerate(persons.persons):
			person_new = Person3D()
			person_new.person_id = person.person_id
			person_new.nose = getNANBodypart(0)
			person_new.neck = getNANBodypart(1)
			person_new.right_shoulder = getNANBodypart(2)
			person_new.right_elbow = getNANBodypart(3)
			person_new.right_wrist = getNANBodypart(4)
			person_new.left_shoulder = getNANBodypart(5)
			person_new.left_elbow = getNANBodypart(6)
			person_new.left_wrist = getNANBodypart(7)
			person_new.right_hip = getNANBodypart(8)
			person_new.right_knee = getNANBodypart(9)
			person_new.right_ankle = getNANBodypart(10)
			person_new.left_hip = getNANBodypart(11)
			person_new.left_knee = getNANBodypart(12)
			person_new.left_ankle = getNANBodypart(13)
			person_new.right_eye = getNANBodypart(14)
			person_new.left_eye = getNANBodypart(15)
			person_new.right_ear = getNANBodypart(16)
			person_new.left_ear = getNANBodypart(17)

			for body_part in person.body_part:
				body_part_new = get3dcoordinates(body_part, pc, image_width, image_height)
				if body_part.part_id == 0:
					person_new.nose = body_part_new
				elif body_part.part_id == 1:
					person_new.neck = body_part_new
				elif body_part.part_id == 2:
					person_new.right_shoulder = body_part_new
				elif body_part.part_id == 3:
					person_new.right_elbow = body_part_new
				elif body_part.part_id == 4:
					person_new.right_wrist = body_part_new
				elif body_part.part_id == 5:
					person_new.left_shoulder = body_part_new
				elif body_part.part_id == 6:
					person_new.left_elbow = body_part_new
				elif body_part.part_id == 7:
					person_new.left_wrist = body_part_new
				elif body_part.part_id == 8:
					person_new.right_hip = body_part_new
				elif body_part.part_id == 9:
					person_new.right_knee = body_part_new
				elif body_part.part_id == 10:
					person_new.right_ankle = body_part_new
				elif body_part.part_id == 11:
					person_new.left_hip = body_part_new
				elif body_part.part_id == 12:
					person_new.left_knee = body_part_new
				elif body_part.part_id == 13:
					person_new.left_ankle = body_part_new
				elif body_part.part_id == 14:
					person_new.right_eye = body_part_new
				elif body_part.part_id == 15:
					person_new.left_eye = body_part_new
				elif body_part.part_id == 16:
					person_new.right_ear = body_part_new
				elif body_part.part_id == 17:
					person_new.left_ear = body_part_new
				
			keypoints_3d_pub.publish(person_new)


	except rospy.ServiceException, e:
		rospy.loginfo("GetPersons Service call failed: %s"%e)
		return

if __name__ == '__main__':
    # Initialize Node and Parameters
    rospy.init_node("point_extraction_3d")
    point_cloud = rospy.get_param("~point_cloud", "/camera/depth_registered/points")
    image = rospy.get_param("~rgb_image", "/camera/color/image_raw")

    # Declare Subscribers
    # Synchronize Point Cloud and Image Subscription Received Messages
    pc_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)
    img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)

    # Pointcloud publisher topic /tfpose_ros/input_pointcloud
    pc_pub = rospy.Publisher('/tfpose_ros/input_pointcloud', PointCloud2, queue_size=0)

    # Image publisher topic /tfpose_ros/input_rgb
    img_pub = rospy.Publisher('/tfpose_ros/input_rgb', Image, queue_size=0)

    # Keypoints in 3D topic /tfpose_ros/detected_poses_keypoints_3d
    keypoints_3d_pub = rospy.Publisher('/tfpose_ros/detected_poses_keypoints_3d', Person3D, queue_size=0);

    # ApproximateTime takes a queue size as its constructor argument
    ts = message_filters.ApproximateTimeSynchronizer([pc_sub, img_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(converter3d_callback)

    # Spin Forever
    rospy.spin()