#!/usr/bin/env python
import rospy, sys
import numpy as np
import pandas as pd
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class GripperTransform(object):
	def __init__(self):
		# Get transformation parameters from csv file (Fetched previously)
		df = pd.read_csv("~/ros/camera_transforms/transform.csv", header=None)
		self.rot_matrix, self.tra_matrix = self.transform_matrixes(df.values)

		df = pd.read_csv("~/ros/camera_transforms/gripper_points.csv", header=None)
		self.left = df.values[[0, 1, 2], :]
		self.right = df.values[[3, 4, 5], :]

		gripper_transformed = self.transform_points(self.left, self.right)
		pd.DataFrame(gripper_transformed).to_csv("~/ros/camera_transforms/gripper_transformed1.csv", header=False, index=False)
		print("Camera gripper points saved in ros/camera_transforms/gripper_transformed1.csv")

	def transform_matrixes(self, med):
		rot_matrix = np.zeros((3, 3))
		tra_matrix = np.zeros((3, 1))
		tx, ty, tz = med[0][0], med[1][0], med[2][0]
		qx, qy, qz, qw = med[3][0], med[4][0], med[5][0], med[6][0]
		s = 1 / (qx**2 + qy**2 + qz**2 + qw**2)

		# Rotation elements
		rot_matrix[0][0] = 1 - 2 * s * (qy**2 + qz**2)
		rot_matrix[0][1] = 2 * s * (qx*qy - qz*qw)
		rot_matrix[0][2] = 2 * s * (qx*qz + qy*qw)

		rot_matrix[1][0] = 2 * s * (qx*qy + qz*qw)
		rot_matrix[1][1] = 1 - 2 * s * (qx**2 + qz**2)
		rot_matrix[1][2] = 2 * s * (qy*qz - qx*qw)

		rot_matrix[2][0] = 2 * s * (qx*qz - qy*qw)
		rot_matrix[2][1] = 2 * s * (qy*qz + qx*qw)
		rot_matrix[2][2] = 1 - 2 * s * (qx**2 + qy**2)

		# Translation elements
		tra_matrix[0][0] = -tx
		tra_matrix[1][0] = -ty
		tra_matrix[2][0] = -tz

		return rot_matrix, tra_matrix

	def transform_points(self, left, right):
		new_left = self.transform_point(left)
		new_right = self.transform_point(right)
		return np.vstack((new_left, new_right))

	def transform_point(self, point_array):
		# Apply transformation matrix to point
		point_tf = self.tra_matrix + point_array
		point_tf = np.matmul(self.rot_matrix, point_tf)	

		return point_tf


if __name__ == '__main__':
	# Node initialization and number of samples for median
	rospy.init_node("gripper_transform")

	# Axis transformation object
	rt = GripperTransform()