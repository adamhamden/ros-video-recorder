#!/usr/bin/env python3

import cv2
import tempfile
from src.recorder.recorder import Recorder


class VideoRecorder(Recorder):

    VIDEO_TYPE = {
        'avi': cv2.VideoWriter_fourcc(*'XVID'),
        # 'mp4': cv2.VideoWriter_fourcc(*'H264'),
        'mp4': cv2.VideoWriter_fourcc(*'XVID'),
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
        super(VideoRecorder, self).__init__()
        self.vid_type = vid_type
        self.frames_per_second = frames_per_second
        self.video_dimensions = video_dimensions
        self.filename = None
        self.out = None

    def start_recording(self, filename=None, filepath=""):

        if self.is_recording:
            print("Error: video recorder is currently recording")
            return

        if not filename:
            self.filename = tempfile.mktemp(prefix='this_is_a_unique_temp_audio_file_', suffix='.'+self.vid_type, dir=filepath)
        else:
            self.filename = filepath+"/"+filename

        self.out = cv2.VideoWriter(self.filename, self.VIDEO_TYPE[self.vid_type], self.frames_per_second,
                                       self.STD_DIMENSIONS[self.video_dimensions])
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
        self.is_recording = False
