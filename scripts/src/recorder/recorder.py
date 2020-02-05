#!/usr/bin/env python3


class InRecordingException(Exception):

    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


class Recorder(object):

    def __init__(self):

        self.is_recording = False

    #@abstractmethod
    def start_recording(self, filename=None, filepath=""):
        self.is_recording = True

    #@abstractmethod
    def add_data(self, data):

        pass

    #@abstractmethod
    def stop_recording(self):

        self.is_recording = False
