#!/usr/bin/env python
import rospy, sys
import numpy as np
import pandas as pd
from tqdm import tqdm
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class TransformRT(object):
	def __init__(self, med_num, first, transform, analysis):
		# 7xN matrix, where each column has the translation (x, y, z) and rotation (parameters) from camera do robot
		self.point_matrix_0 = np.array([])
		self.point_matrix_6 = np.array([])
		self.transform = transform
		self.first = first

		# 6xN matrix, with translation and axis rotation parameters
		#elf.analysis_matrix = np.array([])
		#self.analysis = analysis

		self.med_num = int(med_num)

		markers = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
		print("Getting transformation parameters...")

	def callback(self, am): 
		# Only has marker with id 0
		for marker in am.detections:
			t = marker.pose.pose.pose.position
			q = marker.pose.pose.pose.orientation

			"""rx, ry, rz = self.quaternion_to_axis_rotation(q.x, q.y, q.z, q.w)
			if self.analysis:
				if self.analysis_matrix.shape[0] != 0:
					self.analysis_matrix = np.insert(self.analysis_matrix, 0, [t.x, t.y, t.z, rx, ry, rz], axis=1)
				else:
					self.analysis_matrix = np.array([t.x, t.y, t.z, rx, ry, rz]).reshape(6, 1)
			"""
			#if self.transform:
			if marker.id[0] == 0:
				if self.point_matrix_0.shape[0] != 0:
					self.point_matrix_0 = np.insert(self.point_matrix_0, 0, [t.x, t.y, t.z], axis=1)
				else:
					self.point_matrix_0 = np.array([t.x, t.y, t.z]).reshape(3, 1)

			if marker.id[0] == 6:	
				if self.point_matrix_6.shape[0] != 0:
					self.point_matrix_6 = np.insert(self.point_matrix_6, 0, [t.x, t.y, t.z, q.x, q.y, q.z, q.w], axis=1)
				else:
					self.point_matrix_6 = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w]).reshape(7, 1)

		self.printProgressBar(self.point_matrix_6.shape[1], self.med_num)

		if self.transform and self.point_matrix_6.shape[1] == self.med_num:
			median_calibration = np.median(self.point_matrix_0, axis=1)
			median_transform = np.median(self.point_matrix_6, axis=1)
			if self.first:
				pd.DataFrame(median_calibration).to_csv("~/ros/camera_transforms/ar_reference.csv", header=False, index=False)
				pd.DataFrame(median_transform).to_csv("~/ros/camera_transforms/transform_reference.csv", header=False, index=False)
				print("Transformation parameters saved in ros/camera_transforms/transform_reference.csv")
			else:
				pd.DataFrame(median_calibration).to_csv("~/ros/camera_transforms/ar_point.csv", header=False, index=False)
				pd.DataFrame(median_transform).to_csv("~/ros/camera_transforms/transform.csv", header=False, index=False)
				print("Transformation parameters saved in ros/camera_transforms/transform.csv")
			rospy.signal_shutdown("")

		"""if self.analysis and self.analysis_matrix.shape[1] == self.med_num:
			pd.DataFrame(self.analysis_matrix).to_csv("~/ros/camera_transforms/analysis.csv", header=False, index=False)
			print("Analysis parameters saved in ros/camera_transforms/analysis.csv")
			rospy.signal_shutdown("")
		"""

	def quaternion_to_axis_rotation(self, qx, qy, qz, qw):
		r = R.from_quat([qx, qy, qz, qw])
		rx, ry, rz = r.as_euler('xyz', degrees=True)
		return rx, ry, rz

	def printProgressBar (self, iteration, total):
	    percent = ("{0:.1f}").format(100 * (iteration / float(total)))
	    filledLength = int(100 * iteration // total)
	    bar = '█' * filledLength + '-' * (100 - filledLength)
	    print('\r%s |%s| %s%% %s' % ("Transform Progress", bar, percent, "Complete"), end = '\r')
	    # Print New Line on Complete
	    if iteration == total: 
	        print()


if __name__ == '__main__':
	# Node initialization and number of samples for median
	rospy.init_node("transform_rt")
	med_num = rospy.get_param("~median_num", "100")
	first_tra = rospy.get_param("~first_tra", "True")

	# Axis transformation object
	rt = TransformRT(med_num, bool(first_tra), transform=True, analysis=False)

	rospy.spin()