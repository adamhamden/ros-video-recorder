#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from recorder.video_recorder import VideoRecorder
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os


class ROSVideoRecord:

    def __init__(self):

        self.node = rospy.init_node('video_recorder', anonymous=True)
        self.bridge = CvBridge()

        print("Image topic: "+rospy.get_param("image_topic"))
        print("Command topic: "+rospy.get_param("command_topic"))

        self.image_sub = rospy.Subscriber(rospy.get_param("image_topic"), Image, self.image_callback)
        self.command_sub = rospy.Subscriber(rospy.get_param("command_topic"), String, self.command_callback)
        self.is_recording = False
        self.recorder = None

    def image_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.is_recording:
            self.recorder.add_data(cv_image)

    def command_callback(self, data):

        if "stop" in data.data:
            self.stop_recording()

        if "start" in data.data:
            self.start_recording()

    def start_recording(self):

        if self.recorder:
            print("Recording already in progress")
            return

        print("Starting a new recorder object")
        self.recorder = VideoRecorder()
        self.recorder.start_recording(file_path="/root/shared/catkin_ws/src/ros-video-recorder/video_repo")
        self.is_recording = True

    def stop_recording(self):

        if not self.recorder:
            print("No recording in progress")
            return

        print("Ending recording")
        self.is_recording = False
        self.recorder.stop_recording(upload_credentials="/root/shared/catkin_ws/src/ros-video-recorder/aws_config.yaml")
        self.recorder = None


if __name__ == "__main__":

    ros_recorder = ROSVideoRecord()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()