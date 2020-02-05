#!/usr/bin/env python

import cv2
import tempfile
import datetime


class VideoRecorder:

    VIDEO_TYPE = {
        'avi': cv2.VideoWriter_fourcc(*'mpeg'),
        'mp4': cv2.VideoWriter_fourcc(*'mpeg')
    }

    STD_DIMENSIONS = {
        "480p": (640, 480),
        "720p": (1280, 720),
        "1080p": (1920, 1080),
        "4k": (3840, 2160),
    }

    def __init__(
            self,
            vid_type='mp4',
            frames_per_second=30,
            video_dimensions='480p'
    ):

        self.vid_type = vid_type
        self.frames_per_second = frames_per_second
        self.video_dimensions = video_dimensions
        self.is_recording = False
        self.filename = None
        self.out = None

    def start_recording(self, filename=None, filepath=""):

        if self.is_recording:
            print("Error: video recorder is currently recording")
            return

        self.filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + '.' + self.vid_type

        self.out = cv2.VideoWriter(self.filename, self.VIDEO_TYPE[self.vid_type], self.frames_per_second,
                                       self.STD_DIMENSIONS[self.video_dimensions], True)
        self.is_recording = True

    def add_data(self, image_frame):

        if not self.is_recording:
            print("Error: video recorder is currently not recording, cannot add frame")
            return

        self.out.write(image_frame)

    def stop_recording(self):

        if not self.is_recording:
            print("Error: video recorder is currently not recording")
            return

        self.out.release()
        self.out = None
        self.filename = None
        self.is_recording = False
