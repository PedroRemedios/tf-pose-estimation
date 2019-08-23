#!/usr/bin/env python
import rospy, sys
import numpy as np
import pandas as pd
import tf
import math
import geometry_msgs.msg

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class ArcodeCalibration(object):
	def __init__(self, rang, err):
		# Get transformation parameters from csv file (Fetched previously)
		df = pd.read_csv("~/ros/camera_transforms/transform_reference.csv", header=None)
		tf = self.get_tf_transform(df.values)

		dfi = pd.read_csv("~/ros/camera_transforms/ar_reference.csv", header=None)
		self.ar_reference = self.apply_tf_transform(tf, dfi.values)
		print("Point in reference axis:")
		print(self.ar_reference)

		dfii = pd.read_csv("~/ros/camera_transforms/ar_point.csv", header=None)
		self.ar_point = dfii.values

		dfiii = pd.read_csv("~/ros/camera_transforms/transform_point.csv", header=None)
		tf = self.get_tf_transform(dfiii.values)
		new_point = self.apply_tf_transform(tf, self.ar_point)
		print("Point in new transform:")
		print(new_point)

		print("Current distance between points: " + str(self.distance_between_points(self.ar_reference, new_point)))
		self.find_best_transform(dfiii.values)

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


	def find_best_transform(self, med):
		# Translation and Rotation parameters from the original transform
		tx, ty, tz = med[0][0], med[1][0], med[2][0]
		qx, qy, qz, qw = med[3][0], med[4][0], med[5][0], med[6][0]
		
		# Here we consider the following formulas from the quaternion rotation:
		#	qw = cos(theta/2)
		#	qx(qy, qz) = tx(ty, tz) * sen(theta/2)
		# Note: For simplification we have that theta = theta / 2
		theta = math.acos(qw)
		rx = qx/math.sin(theta)
		ry = qy/math.sin(theta)
		rz = qz/math.sin(theta)

		# Interval Parameters
		angle_range = 0.01
		num_points = 30
		mincoiso = 100
		qx_f, qy_f, qz_f, qw_f = med[3][0], med[4][0], med[5][0], med[6][0]

		# Iterate over rotation axis and translation in x
		for x in np.arange(max(rx - angle_range, -1), min(rx + angle_range, 1), 2 * angle_range / num_points):
			qx_new = x * math.sin(theta)

			for y in np.arange(max(ry - angle_range, -1), min(ry + angle_range, 1), 2 * angle_range / num_points):
				qy_new = y * math.sin(theta)

				for z in np.arange(max(rz - angle_range, -1), min(rz + angle_range, 1), 2 * angle_range / num_points):
					qz_new = z * math.sin(theta)

					# Quaternion and normalization
					norm = math.sqrt(qx_new**2 + qy_new**2 + qz_new**2 + med[6][0]**2)
					med[3][0], med[4][0], med[5][0], med[6][0] = qx_new/norm, qy_new/norm, qz_new/norm, med[6][0]/norm
					# New transformation and points
					tf = self.get_tf_transform(med)
					new_point = self.apply_tf_transform(tf, self.ar_point)

					num = self.distance_between_points(self.ar_reference, new_point)
					if num < mincoiso:
						mincoiso = num
						qx_f, qy_f, qz_f, qw_f = med[3][0], med[4][0], med[5][0], med[6][0]

		final_med = med
		med[3][0], med[4][0], med[5][0], med[6][0] = qx_f, qy_f, qz_f, qw_f
		pd.DataFrame(final_med).to_csv("~/ros/camera_transforms/transform.csv", header=False, index=False)
		print("Nice transformation saved in ros/camera_transforms/transform.csv")
		tf = self.get_tf_transform(final_med)
		new_point = self.apply_tf_transform(tf, self.ar_point)

		print("Point in reference axis:")
		print(self.ar_reference)
		print("Point in new transform:")
		print(new_point)

		print("New distance between points: " + str(self.distance_between_points(self.ar_reference, new_point)))
		print(final_med)

	def distance_between_points(self, reference, new):
		# Conditions that need to be close to zero (No distance between same point)
		zero_conditions = np.zeros((3, 1))
		zero_conditions[0][0] = abs(new.x - reference.x)
		zero_conditions[1][0] = abs(new.y - reference.y)
		zero_conditions[2][0] = abs(new.z - reference.z)

		return math.sqrt(zero_conditions[0][0]**2 + zero_conditions[1][0]**2 + zero_conditions[2][0]**2)		

if __name__ == '__main__':
	# Node initialization and number of samples for median
	rang = rospy.get_param("~rang", "0.1")
	err = rospy.get_param("~err", "0.015")
	rospy.init_node("ar_callibration")
	# Axis transformation object
	AR = ArcodeCalibration(rang, err)
