#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from src.recorder.video_recorder import VideoRecorder
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ROSVideoRecord:

    def __init__(self):

        self.node = rospy.init_node('video_recorder', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.command_sub = rospy.Subscriber("/video/command", String, self.command_callback)
        self.is_recording = False
        self.recorder = None
        self.count = 0
        self.fourcc = cv2.VideoWriter_fourcc(*'mpeg')
        self.out = cv2.VideoWriter('sdfdfsdfdsd.mp4', self.fourcc, 20.0, (640, 480), True)

    def image_callback(self,data):
        self.count += 1
        if self.count > 100:
            self.out.release()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = cv2.imread('../1.jpg')
            self.out.write(cv_image)
        except CvBridgeError as e:
            print(e)

    def command_callback(self, data):
        print(data)
        if "stop" in data.data:
            self.out.release()
            self.stop_recording()

        if "start" in data.data:
            self.start_recording()

    def start_recording(self):

        if self.recorder:
            return

        self.recorder = VideoRecorder('mp4', 30, '720p')
        self.recorder.start_recording()
        self.is_recording = True

    def stop_recording(self):

        if not self.recorder:
            return

        self.is_recording = False
        self.recorder.stop_recording()


if __name__ == "__main__":

    ros_recorder = ROSVideoRecord()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()