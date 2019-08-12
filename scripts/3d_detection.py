#!/usr/bin/env python
import time
import os
import sys
import ast

from threading import Lock
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from tfpose_ros.msg import Persons, Person, BodyPartElm

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import model_wh, get_graph_path
from tfpose_ros.srv import *


def humans_to_msg(humans):
    persons = Persons()

    for human in humans:
        person = Person()
        person.person_id = human.id

        for k in human.body_parts:
            body_part = human.body_parts[k]

            body_part_msg = BodyPartElm()
            body_part_msg.part_id = body_part.part_idx
            body_part_msg.x = body_part.x
            body_part_msg.y = body_part.y
            body_part_msg.confidence = body_part.score
            person.body_part.append(body_part_msg)
        persons.persons.append(person)

    return persons


def find_persons_handler(request):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(request.image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
        return

    acquired = tf_lock.acquire(False)
    if not acquired:
        return

    try:
        humans = pose_estimator.inference(cv_image, resize_to_default=True, upsample_size=resize_out_ratio)
    finally:
        tf_lock.release()

    msg = humans_to_msg(humans)
    msg.image_w = request.image.width
    msg.image_h = request.image.height
    msg.header = request.image.header

    return GetPersonsResponse(msg)


if __name__ == '__main__':
    # Initialize Node and Parameters
    rospy.init_node('point_detection_3d')
    image = rospy.get_param("~camera", "/camera/color/image_raw")
    model = rospy.get_param('~model', 'mobilenet_thin')
    resolution = rospy.get_param('~resolution', '432x368')
    resize_out_ratio = float(rospy.get_param('~resize_out_ratio', '4.0'))

    tf_lock = Lock()

    try:
        w, h = model_wh(resolution)
        graph_path = get_graph_path(model)

        rospack = rospkg.RosPack()
        graph_path = os.path.join(rospack.get_path('tfpose_ros'), graph_path)
    except Exception as e:
        rospy.logerr('invalid model: %s, e=%s' % (model, e))
        sys.exit(-1)

    pose_estimator = TfPoseEstimator(graph_path, target_size=(w, h))
    cv_bridge = CvBridge()

    s = rospy.Service('get_persons', GetPersons, find_persons_handler)

    rospy.spin()
