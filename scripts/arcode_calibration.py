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
		df = pd.read_csv("~/ros/camera_transforms/transform_reference.csv", header=None)
		self.rot_matrix, self.tra_matrix = self.transform_matrixes(df.values)

		dfi = pd.read_csv("~/ros/camera_transforms/ar_reference.csv", header=None)
		self.ar_reference = self.transform_point(dfi.values)
		print(self.ar_reference)

		dfii = pd.read_csv("~/ros/camera_transforms/ar_point.csv", header=None)
		self.ar_point = dfii.values

		dfiii = pd.read_csv("~/ros/camera_transforms/transform.csv", header=None)
		self.rot_matrix, self.tra_matrix = self.transform_matrixes(dfiii.values)
		new_point = self.transform_point(self.ar_point)
		print(new_point)
		zero_conditions = np.zeros((3, 1))
		zero_conditions[0][0] = abs(new_point[0][0] - self.ar_reference[0][0])
		zero_conditions[1][0] = abs(new_point[1][0] - self.ar_reference[1][0])
		zero_conditions[2][0] = abs(new_point[2][0] - self.ar_reference[2][0])

		print(math.sqrt(zero_conditions[0][0]**2 + zero_conditions[1][0]**2 + zero_conditions[2][0]**2))
		return
		self.error_find(dfiii.values)

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


	def find_best_transform(self, med, err, num_trans):
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
		origin_range = 0.05
		num_points = 100
		mincoiso = 100
		final_med = med

		# Iterate over rotation axis and translation in x
		for x in np.arange(max(rx - angle_range, -1), min(rx + angle_range, 1), 2 * angle_range / num_points):
			qx_new = x * math.sin(theta)

			for y in np.arange(max(ry - angle_range, -1), min(ry + angle_range, 1), 2 * angle_range / num_points):
				qy_new = y * math.sin(theta)

				for z in np.arange(max(rz - angle_range, -1), min(rz + angle_range, 1), 2 * angle_range / num_points):
					qz_new = z * math.sin(theta)

					# New transformation and points
					med[3][0], med[4][0], med[5][0] = qx_new, qy_new, qz_new
					self.rot_matrix, self.tra_matrix = self.transform_matrixes(med)
					new_point = self.transform_point(self.ar_point)

					# Conditions that need to be close to zero (Same y and z, and x middle point)
					zero_conditions = np.zeros((3, 1))
					zero_conditions[0][0] = abs(new_point[0][0] - self.ar_reference[0][0])
					zero_conditions[1][0] = abs(new_point[1][0] - self.ar_reference[1][0])
					zero_conditions[2][0] = abs(new_point[2][0] - self.ar_reference[2][0])

					num = math.sqrt(zero_conditions[0][0]**2 + zero_conditions[1][0]**2 + zero_conditions[2][0]**2)
					if num < mincoiso:
						mincoiso = num
						final_med = med
						print num

		pd.DataFrame(final_med).to_csv("~/ros/camera_transforms/calibration/nice_transform%d.csv" % num_trans, header=False, index=False)
		print("New nice transformation saved in ros/camera_transforms/calibration/nice_transform%d.csv" % num_trans)

		return 2


	def transform_points(self, left, right):
		new_left = self.transform_point(left)
		new_right = self.transform_point(right)
		return new_left, new_right

	def transform_point(self, point_array):
		point_tf = self.tra_matrix + point_array
		point_tf = np.matmul(self.rot_matrix, point_tf)	

		return point_tf

	def error_find(self, med):
		num_trans = 0
		num_trans_min = 2
		err_rang_min = 0.010
		err_rang_max = 0.050
		pace = 0.002
		for err in np.arange(err_rang_min, err_rang_max, pace):
			print err
			num_trans = self.find_best_transform(med, err, num_trans)
			if num_trans >= num_trans_min:
				break

		if num_trans == 0:
			print('No nice transformations were found')
		else:
			print('Yey, %d nice transformations were found' % num_trans)

if __name__ == '__main__':
	# Node initialization and number of samples for median
	rang = rospy.get_param("~rang", "0.1")
	err = rospy.get_param("~err", "0.015")
	rospy.init_node("ar_callibration")
	# Axis transformation object
	AR = ArcodeCalibration(rang, err)
