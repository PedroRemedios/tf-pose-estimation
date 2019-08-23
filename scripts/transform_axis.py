#!/usr/bin/env python
import rospy, math
import pandas as pd
import numpy as np
import tf
import geometry_msgs.msg
from tfpose_ros.msg import Person3D, BodyPartElm3D

class Transform(object):
	def __init__(self):
		# Declare Subscribers
		self.person_sub = rospy.Subscriber("/tfpose_ros/detected_poses_keypoints_3d", Person3D, self.callback)

		# Pointcloud publisher topic /tfpose_ros/points_3d
		self.person_pub = rospy.Publisher("/tfpose_ros/transformed_poses_keypoints_3d", Person3D, queue_size=0)

		# Get transformation parameters from csv file (Fetched previously)
		df = pd.read_csv("~/ros/camera_transforms/transform.csv", header=None)
		self.tf = self.get_tf_transform(df.values)

	def callback(self, person):
		person_tf = Person3D()
		person_tf.person_id = person.person_id

		person_tf = self.iterate_body_parts(person_tf, person)

		self.person_pub.publish(person_tf)

	def get_tf_transform(self, med):
		rot_matrix = np.zeros((3, 3))
		tra_matrix = np.zeros((3, 1))
		tx, ty, tz = med[0][0], med[1][0], med[2][0]
		qx, qy, qz, qw = med[3][0], med[4][0], med[5][0], med[6][0]

		t = tf.TransformerROS(True, rospy.Duration(10.0))
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'camera_frame'
		m.child_frame_id = 'baxter_frame'
		m.transform.translation.x = tx
		m.transform.translation.y = ty
		m.transform.translation.z = tz
		m.transform.rotation.x = qx
		m.transform.rotation.y = qy
		m.transform.rotation.z = qz
		m.transform.rotation.w = qw

		t.setTransform(m)
		#t.lookupTransform('/camera_depth_optical_frame', '/baxter_frame', rospy.Time(0))

		return t

	def apply_tf_transform(self, tf, point):
		x = point[0][0]
		y = point[1][0]
		z = point[2][0]

		m2 = geometry_msgs.msg.PointStamped()
		m2.header.frame_id = 'camera_frame'
		m2.point.x = x
		m2.point.y = y
		m2.point.z = z		

		return tf.transformPoint('baxter_frame', m2).point

	def body_part_valid(self, body_part):
		if math.isnan(body_part.x) or math.isnan(body_part.y) or math.isnan(body_part.z):
			return False
		else:
			return True

	def transformed_body_part(self, body_part):
		# If it consists in NANs, don't transform
		if not self.body_part_valid(body_part):
			return body_part

		body_part_tf = BodyPartElm3D()
		body_part_tf.part_id = body_part.part_id
		body_part_tf.confidence = body_part.confidence

		# Apply transformation matrix to body part point
		point_array = np.array([body_part.x, body_part.y, body_part.z]).reshape(3, 1)
		point_tf = self.apply_tf_transform(self.tf, point_array)
		body_part_tf.x, body_part_tf.y, body_part_tf.z = point_tf.x, point_tf.y, point_tf.z		

		return body_part_tf

	def iterate_body_parts(self, new_person, person):
		new_person.nose = self.transformed_body_part(person.nose)
		new_person.neck = self.transformed_body_part(person.neck)
		new_person.right_shoulder = self.transformed_body_part(person.right_shoulder)
		new_person.right_elbow = self.transformed_body_part(person.right_elbow)
		new_person.right_wrist = self.transformed_body_part(person.right_wrist)
		new_person.left_shoulder = self.transformed_body_part(person.left_shoulder)
		new_person.left_elbow = self.transformed_body_part(person.left_elbow)
		new_person.left_wrist = self.transformed_body_part(person.left_wrist)
		new_person.right_hip = self.transformed_body_part(person.right_hip)
		new_person.right_knee = self.transformed_body_part(person.right_knee)
		new_person.right_ankle = self.transformed_body_part(person.right_ankle)
		new_person.left_hip = self.transformed_body_part(person.left_hip)
		new_person.left_knee = self.transformed_body_part(person.left_knee)
		new_person.left_ankle = self.transformed_body_part(person.left_ankle)
		new_person.right_eye = self.transformed_body_part(person.right_eye)
		new_person.left_eye = self.transformed_body_part(person.left_eye)
		new_person.right_ear = self.transformed_body_part(person.right_ear)
		new_person.left_ear = self.transformed_body_part(person.left_ear)
		return new_person


if __name__ == '__main__':
	# Node initialization and number of samples for median
	rospy.init_node("transform_axis")
	person_in = rospy.get_param("~person_in", "/tfpose_ros/detected_poses_keypoints_3d")
	person_out = rospy.get_param("~person_out", "/tfpose_ros/transformed_poses_keypoints_3d")

	# Axis transformation object
	tr = Transform()

	rospy.spin()