#!/usr/bin/env python
import rospy, sys
import numpy as np
import pandas as pd
import math

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class ArcodeCalibration(object):
	def __init__(self, rang, err):
		# Get transformation parameters from csv file (Fetched previously)
		df = pd.read_csv("~/ros/camera_transforms/transform.csv", header=None)
		self.rot_matrix, self.tra_matrix = self.transform_matrixes(df.values)

		dfi = pd.read_csv("~/ros/camera_transforms/ar_point.csv", header=None)
		self.ar_point = dfi.values

		dfii = pd.read_csv("~/ros/camera_transforms/ar_reference.csv", header=None)
		self.ar_reference = self.transform_point(dfii.values)

		self.find_best_transform(df.values, float(rang), float(err))

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


	def find_best_transform(self, med, rang, err):
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
		angle_range = 0.05
		origin_range = 0.05
		num_points = 40
		num_trans = 0

		# Iterate over rotation axis and translation in x
		for x in np.arange(max(rx - angle_range, -1), min(rx + angle_range, 1), 2 * angle_range / num_points):
			qx_new = x * math.sin(theta)

			for y in np.arange(max(ry - angle_range, -1), min(ry + angle_range, 1), 2 * angle_range / num_points):
				qy_new = y * math.sin(theta)

				for z in np.arange(max(rz - angle_range, -1), min(rz + angle_range, 1), 2 * angle_range / num_points):
					qz_new = z * math.sin(theta)

					for t in np.arange(tx - origin_range, tx + origin_range, 2 * origin_range / num_points):
						x_new = t

						# New transformation and points
						med[0][0], med[3][0], med[4][0], med[5][0] = x_new, qx_new, qy_new, qz_new
						self.rot_matrix, self.tra_matrix = self.transform_matrixes(med)
						new_point = self.transform_point(self.ar_point)

						# Conditions that need to be close to zero (Same y and z, and x middle point)
						zero_conditions = np.zeros((3, 1))
						zero_conditions[0][0] = abs(new_point[1][0] - self.ar_reference[1][0])
						zero_conditions[1][0] = abs(new_point[1][0] - self.ar_reference[1][0])
						zero_conditions[2][0] = abs(new_point[2][0] - self.ar_reference[2][0])

						if abs(zero_conditions[0][0]) <= err and abs(zero_conditions[1][0]) <= err and abs(zero_conditions[2][0]) <= err:
							pd.DataFrame(med).to_csv("~/ros/camera_transforms/calibration/nice_transform%d.csv" % num_trans, header=False, index=False)
							print("New nice transformation saved in ros/camera_transforms/calibration/nice_transform%d.csv" % num_trans)
							print("X: %.2f, Y: %.2f, Z: %.2f, tX: %.2f" % (qx_new, qy_new, qz_new, x_new))
							num_trans += 1

		if num_trans == 0:
			print('No nice transformations were found')
		else:
			print('Yey, %d nice transformations were found' % num_trans)


	def transform_points(self, left, right):
		new_left = self.transform_point(left)
		new_right = self.transform_point(right)
		return new_left, new_right

	def transform_point(self, point_array):
		point_tf = self.tra_matrix + point_array
		point_tf = np.matmul(self.rot_matrix, point_tf)	

		return point_tf


if __name__ == '__main__':
	# Node initialization and number of samples for median
	rang = rospy.get_param("~rang", "0.1")
	err = rospy.get_param("~err", "0.0008")
	rospy.init_node("ar_callibration")
	# Axis transformation object
	AR = ArcodeCalibration(rang, err)
