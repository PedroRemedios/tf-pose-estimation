#!/usr/bin/env python
import rospy, sys
import numpy as np
import pandas as pd
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class TransformRT(object):
	def __init__(self, med_num, transform, analysis):
		# 7xN matrix, where each column has the translation (x, y, z) and rotation (parameters) from camera do robot
		self.point_matrix = np.array([])
		self.transform = transform

		# 6xN matrix, with translation and axis rotation parameters
		self.analysis_matrix = np.array([])
		self.analysis = analysis

		self.med_num = int(med_num)

		markers = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
		print("Getting transformation parameters...")

	def callback(self, am): 
		# Only has marker with id 0
		for marker in am.detections:
			t = marker.pose.pose.pose.position
			q = marker.pose.pose.pose.orientation

			rx, ry, rz = self.quaternion_to_axis_rotation(q.x, q.y, q.z, q.w)
			if self.analysis:
				if self.analysis_matrix.shape[0] != 0:
					self.analysis_matrix = np.insert(self.analysis_matrix, 0, [t.x, t.y, t.z, rx, ry, rz], axis=1)
				else:
					self.analysis_matrix = np.array([t.x, t.y, t.z, rx, ry, rz]).reshape(6, 1)

			if self.transform:	
				if self.point_matrix.shape[0] != 0:
					self.point_matrix = np.insert(self.point_matrix, 0, [t.x, t.y, t.z, q.x, q.y, q.z, q.w], axis=1)
				else:
					self.point_matrix = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w]).reshape(7, 1)

		if self.transform and self.point_matrix.shape[1] == self.med_num:
			median_matrix = np.median(self.point_matrix, axis=1)
			pd.DataFrame(median_matrix).to_csv("~/ros/camera_transforms/transform.csv", header=False, index=False)
			print("Transformation parameters saved in ros/camera_transforms/transform.csv")
			rospy.signal_shutdown("")

		if self.analysis and self.analysis_matrix.shape[1] == self.med_num:
			pd.DataFrame(self.analysis_matrix).to_csv("~/ros/camera_transforms/analysis.csv", header=False, index=False)
			print("Analysis parameters saved in ros/camera_transforms/analysis.csv")
			rospy.signal_shutdown("")

		print(self.point_matrix.shape[1])

	def quaternion_to_axis_rotation(self, qx, qy, qz, qw):
		r = R.from_quat([qx, qy, qz, qw])
		rx, ry, rz = r.as_euler('xyz', degrees=True)
		return rx, ry, rz


if __name__ == '__main__':
	# Node initialization and number of samples for median
	rospy.init_node("transform_rt")
	med_num = rospy.get_param("~median_num", "100")

	# Axis transformation object
	rt = TransformRT(med_num, transform=True, analysis=False)

	rospy.spin()