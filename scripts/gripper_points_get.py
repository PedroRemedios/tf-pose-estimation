#!/usr/bin/env python
import rospy, sys
import numpy as np
import pandas as pd
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class GripperPoints(object):
	def __init__(self, med_num):
		self.left = np.array([])
		self.right = np.array([])
		self.med_num = int(med_num)

		markers = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
		print("Getting gripper points in the camera axis...")

	def callback(self, am): 
		# We used small squares with marker 0 (Baxter's right gripper) and 2 (Baxter's left gripper)
		for marker in am.markers:
			t = marker.pose.pose.position

			if marker.id == 0:
				if self.right.shape[0] != 0:
					self.right = np.insert(self.right, 0, [t.x, t.y, t.z], axis=1)
				else:
					self.right = np.array([t.x, t.y, t.z]).reshape(3, 1)
			elif marker.id == 2:
				if self.left.shape[0] != 0:
					self.left = np.insert(self.left, 0, [t.x, t.y, t.z], axis=1)
				else:
					self.left = np.array([t.x, t.y, t.z]).reshape(3, 1)

		print(self.left.shape[1])
		print(self.right.shape[1])

		if self.left.shape[1] >= self.med_num and self.right.shape[1] >= self.med_num:
			left_matrix = np.median(self.left, axis=1).reshape(3, 1)
			right_matrix = np.median(self.right, axis=1).reshape(3, 1)
			gripper_points = np.vstack((left_matrix, right_matrix))
			pd.DataFrame(gripper_points).to_csv("~/ros/camera_transforms/gripper_points.csv", header=False, index=False)
			print("Camera gripper points saved in ros/camera_transforms/gripper_points.csv")
			rospy.signal_shutdown("")


if __name__ == '__main__':
	# Node initialization and number of samples for median
	rospy.init_node("gripper_get")
	med_num = rospy.get_param("~median_num", "100")

	# Axis transformation object
	rt = GripperPoints(med_num)

	rospy.spin()