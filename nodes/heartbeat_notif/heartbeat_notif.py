#!/usr/bin/env python3

from __future__ import print_function

import rospy
import yaml
import wave
import pyaudio
import rospkg
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from threading import Thread


class heartbeat_notif:
    SOUND_PATH = rospkg.RosPack().get_path('autoware_ut') + "/nodes/heartbeat_notif/sounds/"
    pyaudio = pyaudio.PyAudio()
    playing = False
    staled_counter = {}
    failure_trigger = None
    above_failure_trigger = 0
    seen_prefixes = []

    use_lidar_center = None
    use_lidar_front = None
    use_cameras = None
    use_gps = None

    def __init__(self):
        # We make sure all systems are operational before starting.
        time.sleep(10)
        rospy.loginfo(self.__class__.__name__ + " - node started")
        rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self.process_agg_diagnostics, queue_size=1)

        self.failure_trigger = self.target_frame = rospy.get_param("~failure_trigger", 2)
        self.use_lidar_center = self.target_frame = rospy.get_param("~use_lidar_center", True)
        self.use_lidar_front = self.target_frame = rospy.get_param("~use_lidar_front", True)
        self.use_cameras = self.target_frame = rospy.get_param("~use_cameras", True)
        self.use_gps = self.target_frame = rospy.get_param("~use_gnss", True)

    def process_agg_diagnostics(self, agg_diagnostics):
        """
        :type agg_diagnostics: DiagnosticArray
        """
        for status in agg_diagnostics.status:
            self.process_status(status)

    def process_status(self, status):
        """
        :type status: DiagnosticStatus
        """

        if status.level == 3:
            self.process_stale(status)
        elif status.level in [1, 2]:
            self.process_error(status)
        elif status.level == 0:
            self.process_ok(status)

    def process_error(self, status):
        # GPS Errors are managed by a specific node
        if self.use_lidar_front and status.name in ["/Lidars/lidar_front"] and not self.playing:
            self.play_audio_async("lidar_front_error.wav")
            rospy.logerr(self.__class__.__name__ + " - FRONT LIDAR ERROR")
        elif self.use_lidar_center and status.name in ["/Lidars/lidar_center"] and not self.playing:
            self.play_audio_async("lidar_center_error.wav")
            rospy.logerr(self.__class__.__name__ + " - LIDAR ERROR")
        elif self.use_cameras and status.name in ["/Cameras"] and not self.playing:
            self.play_audio_async("cameras_error.wav")
            rospy.logerr(self.__class__.__name__ + " - CAMERA ERROR")
        pass

    def process_stale(self, status):
        """
        Stale is when no diagnostics messages have been received for some time.
        :type status: DiagnosticStatus
        """

        # We will get a status for each prefix, we need to filter them to avoid redundancies.
        if status.name+"/" not in self.seen_prefixes:
            self.seen_prefixes.append(status.name+"/")

        prefixed = 0
        for seen in self.seen_prefixes:
            prefixed = prefixed + 1 if seen.startswith(status.name) else prefixed

        # Filter out redundancies.
        if prefixed > 1:
            return

        # We add the staled prefix to the counter if not present and we increment it, if it is.
        if status.name not in self.staled_counter:
            self.staled_counter[status.name] = 1
            return
        self.staled_counter[status.name] += 1

        # We count how many statuses are above the trigger.
        if self.staled_counter[status.name] == self.failure_trigger:
            self.above_failure_trigger += 1

        # If it has not been staled for long enough we exit.
        if self.staled_counter[status.name] < self.failure_trigger:
            return

        if self.above_failure_trigger > 1 and not self.playing:
            self.play_audio_async("multiple_failures.wav")
            rospy.logerr(self.__class__.__name__ + " - MULTIPLE FAILURES : " + ', '.join(self.staled_counter.keys()))
            return
        elif self.use_gps and status.name in ["/GPS"] and not self.playing:
            self.play_audio_async("gps_lost.wav")
            rospy.logerr(self.__class__.__name__ + " - GPS FAILURE")
            return
        elif self.use_lidar_front and status.name in ["/Lidars/lidar_front"] and not self.playing:
            self.play_audio_async("front_lidar_lost.wav")
            rospy.logerr(self.__class__.__name__ + " - FRONT LIDAR FAILURE")
            return
        elif self.use_lidar_center and status.name in ["/Lidars/lidar_center_publish", "/Lidars/lidar_center_publish/points topic status"] and not self.playing:
            self.play_audio_async("center_lidar_lost.wav")
            rospy.logerr(self.__class__.__name__ + " - CENTER LIDAR FAILURE")
            return
        elif self.use_cameras and status.name in ["/Cameras"] and not self.playing:
            self.play_audio_async("cameras_lost.wav")
            rospy.logerr(self.__class__.__name__ + " - CAMERA FAILURE")
            return
        elif status.name in ["/tfl_detection/Status"] and not self.playing:
            self.play_audio_async("trafficlight_stalled.wav")
            rospy.logerr(self.__class__.__name__ + " - TRAFFICLIGHT API FAILURE")
            return

    def process_ok(self, status):
        if status.name in self.staled_counter:
            if self.staled_counter[status.name] >= self.failure_trigger:
                self.above_failure_trigger -= 1

            del self.staled_counter[status.name]
            return

    def play_audio_async(self, filename):
        """
        Allows to play sound asynchronously
        :type filename: str
        :return:
        """
        t = Thread(target=self.play_audio, args=(filename,))
        t.start()

    def play_audio(self, filename):
        """
        Allows to play sound
        :type filename: str
        :return:
        """
        self.playing = True
        audio_file = wave.open(self.SOUND_PATH + filename, "rb")

        stream = self.pyaudio.open(
            format=self.pyaudio.get_format_from_width(audio_file.getsampwidth()),
            channels=audio_file.getnchannels(),
            rate=audio_file.getframerate(),
            output=True
        )
        data = audio_file.readframes(audio_file.getframerate())

        while data:
            stream.write(data)
            data = audio_file.readframes(audio_file.getframerate())

        stream.stop_stream()
        stream.close()
        self.playing = False
        return

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('heartbeat_notif', anonymous=True, log_level=rospy.INFO)
    node = heartbeat_notif()
    node.run()
