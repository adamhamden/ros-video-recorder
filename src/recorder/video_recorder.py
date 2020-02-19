import cv2
import datetime
import os
import boto3
import botocore
from botocore import exceptions
import yaml

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
        self.file_path = None
        self.video_writer = None

    def start_recording(self, filename=None, file_path=""):

        if self.is_recording:
            print("Error: video recorder is currently recording")
            return

        if not filename:
            self.filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + '.' + self.vid_type
        else:
            self.filename = filename

        self.file_path = os.path.join(file_path, self.filename)

        self.video_writer = cv2.VideoWriter(self.file_path, self.VIDEO_TYPE[self.vid_type], self.frames_per_second,
                                       self.STD_DIMENSIONS[self.video_dimensions], True)
        self.is_recording = True

    def add_data(self, image_frame):

        if not self.is_recording:
            print("Error: video recorder is currently not recording, cannot add frame")
            return

        self.video_writer.write(image_frame)

    def upload_and_verify(self, credential_path=None):

        if not credential_path:
            print("Error: No AWS credentials provided, cannot upload file")
            return

        with open(credential_path) as file:
            data = yaml.safe_load(file)
            aws_access_key_id = data["aws_access_key_id"]
            aws_secret_access_key = data["aws_secret_access_key"]
            bucket_name = data["bucket_name"]

        print("Uploading " + self.filename)

        s3 = boto3.client('s3',
                                aws_access_key_id=aws_access_key_id,
                                aws_secret_access_key=aws_secret_access_key,
                            )

        try:
            s3.upload_file(self.file_path, bucket_name, self.filename)
        except botocore.exceptions.ClientError:
            print("Failed to upload file")
            return

        print("Successfully uploaded "+self.filename)

    def stop_recording(self, upload_to_s3=True, upload_credentials=None):

        if not self.is_recording:
            print("Error: video recorder is currently not recording")
            return

        self.video_writer.release()

        if upload_to_s3:
            if not upload_credentials:
                print("Error: no credentials provided")
            else:
                self.upload_and_verify(upload_credentials)

        self.is_recording = False
