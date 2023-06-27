#!/usr/bin/env python

from autoware_msgs.msg import Signals,ExtractedPosition
import rospy
import time
import roslib
import paho.mqtt.client as paho
roslib.load_manifest('diagnostic_updater')
import diagnostic_msgs
import diagnostic_updater
import std_msgs.msg
import json

ib_light_to_op = {"RED" : 2,
        "GREEN": 0,
        "GREEN FLASH": 2,
        "AMBER": 1,
        "AMBER-RED": 2,
        "RED/AMB": 2,
        "AMB FLASH" : 0,
        "AMBER FLASH" : 0,
        "FLASH" : 0,
        "AMBERRED" : 2}

class IBLights:
    def __init__(self):
        tl_grouping_file = rospy.get_param('~tl_grouping_file')
        self.mqtt_host = rospy.get_param('~mqtt_host', 'mqtt.cloud.ut.ee')
        self.mqtt_port = rospy.get_param('~mqtt_port', 8883)
        self.mqtt_topic = rospy.get_param('~mqtt_topic')
        self.timeout = rospy.get_param('~timeout', 1.5) * 1000

        self.ib_id_to_op = None
        with open(tl_grouping_file) as json_file:
            self.ib_id_to_op = json.load(json_file)

        self.pub = rospy.Publisher('ib/roi_signal', Signals, queue_size=10)
        self.r = rospy.Rate(10) # 10hz
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("none")

        self.is_unknown_state = False
        self.unknown_state = None
        self.missing_tls = []
        self.num_of_timeouts = 0
        self.updater.add("Status", self.diag_task)

        self.count = 0
        self.total = 0.0
        self.response_times = {}
        self.tl_avail_dict = {k:0.0 for (k,v) in self.ib_id_to_op.items()}
        self.tl_avail_count = {k:0 for (k,v) in self.ib_id_to_op.items()}
        self.connection_up = False

        self.res = {}

        client = paho.Client()
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        client.on_connect = self.on_connect

        client.tls_set("/etc/ssl/certs/ca-certificates.crt")
        client.connect(self.mqtt_host, self.mqtt_port, keepalive=10)
        client.loop_start()

    def on_message(self, client, userdata, msg):
        rospy.logdebug('MQTT message recieved: %s, %s', msg.topic, str(msg.payload))
        # tl_id = msg.topic.split('/')[-1]
        tl_id = msg.topic
        self.res[tl_id] = json.loads(msg.payload)

    def on_disconnect(self, client, userdata, rc):
        self.connection_up = False
        rospy.logerr('Disconnected from MQTT server %s:%d, return code: %d', self.mqtt_host, self.mqtt_port, rc)

    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            rospy.logerr('Failed to connect to MQTT server %s:%d, return code: %d', self.mqtt_host, self.mqtt_port, rc)
            return
        self.connection_up = True
        rospy.loginfo('Connected to MQTT server %s:%d', self.mqtt_host, self.mqtt_port)
        client.subscribe(self.mqtt_topic)


    def run(self):
        while not rospy.is_shutdown():
            self.combine_and_publish()
            self.updater.update()
            self.r.sleep()

    def get_signals(self):
        signals = list()
        self.count += 1
        self.missing_tls = []
        self.response_times = {}
        for idx in self.ib_id_to_op.keys():
            if idx not in self.res:
                self.missing_tls.append(idx)
                continue 

            ct = int(time.time() * 1000) # current time
            ts = int(self.res[idx]['timestamp']) # message timestamp 
            dt = ct - ts # delay
            self.response_times[idx] = dt

            if dt > self.timeout:
                rospy.logwarn('Traffic light (id: %s) timed out', idx)
                del self.res[idx]
                self.num_of_timeouts += 1
                self.missing_tls.append(idx)
                continue

            self.tl_avail_count[idx] += 1
            state = self.res[idx]['status']
            if state not in ib_light_to_op.keys():
                rospy.logwarn("Unknown traffic light state: %s id: %s", state, idx)
                self.is_unknown_state = True
                self.unknown_state = state
                state = "RED"

            for op_id in self.ib_id_to_op[idx]:
                exp = ExtractedPosition()
                exp.linkId = op_id
                exp.radius = 10000
                exp.type = ib_light_to_op[state]
                signals.append(exp)
        self.tl_avail_dict = {k:str(float(self.tl_avail_count[k])/self.count * 100) + '%' for (k,v) in self.ib_id_to_op.items()}
        return signals

    def combine_and_publish(self):
        ib_signals = self.get_signals()
        msg = Signals()
        msg.Signals = ib_signals
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def diag_task(self, stat):
        if not self.connection_up:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                    "MQTT connection is down")
        elif self.res == {}:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 
                        "No traffic light data")
        elif len(self.missing_tls) > 0:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                    "Missing some traffic lights")
        elif self.is_unknown_state:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 
                        "Unknown traffic light state")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 
                        "OK!")
        stat.add("Connection up", self.connection_up)
        stat.add("Received num of TLs", len(self.res))
        stat.add("Unknown TL state", self.unknown_state)
        stat.add("Missing traffic light IDs", self.missing_tls)
        stat.add("TL response times", self.response_times)
        stat.add("% of traffic light availability", self.tl_avail_dict)
        
        if self.is_unknown_state:
            self.is_unknown_state = False
            self.unknown_state = None
        return stat

if __name__ == '__main__':
    rospy.init_node('ib_tls', log_level=rospy.INFO, anonymous=True)
    node = IBLights()
    node.run()
