#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import locale
from std_msgs.msg import Bool
from collections import defaultdict
import actionlib
import rospy
import random
from threading import Event, Thread, Lock
from sound_play.msg import SoundRequestAction, SoundRequestGoal
from sound_play.msg import SoundRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import yaml


class SpeakThread(Thread):
    def __init__(self, rate=1.0, wait=True, blacklist=None, language=None):
        super(SpeakThread, self).__init__()
        self.event = Event()
        self.rate = rate
        self.wait = wait
        self.lock = Lock()
        self.stale = []
        self.error = []
        self.history = defaultdict(lambda: 10)
        self.blacklist = blacklist
        self.language = language or "ja-JP"

        self.speak_action = actionlib.SimpleActionClient(
            "sound_play", SoundRequestAction)
        if not self.speak_action.wait_for_server(rospy.Duration(5.0)):
            rospy.logdebug("Failed to find action server '{}'".format(
                rospy.resolve_name("sound_play")))
            self.speak_action = None
            self.speak_pub = rospy.Publisher("robotsound", SoundRequest, queue_size=1)
            rospy.sleep(1)
            if self.speak_pub.get_num_connections() == 0:
                rospy.logwarn("No speech to text service found: {}".format(
                    rospy.resolve_name("robotsound")))

    def stop(self):
        self.event.set()

    def sort(self, error):
        ret = {}
        for s in error:
            ns = s.name.split()[0]
            if any(filter(lambda n: n in ns, self.blacklist)):
                continue
            rm_ns = filter(lambda n: n in ns, ret.keys())
            if rm_ns:
                for k in rm_ns:
                    del ret[k]
            ret[ns] = s
        return ret.values()

    def add(self, stale, error):
        with self.lock:
            self.stale = self.sort(stale)
            self.error = self.sort(error)

    def pop(self):
        with self.lock:
            for e in self.stale + self.error:
                if e.name not in self.history.keys():
                    self.history[e.name] += 1
                    return e
            return None

    def sweep(self):
        for k in self.history.keys():
            self.history[k] -= 1
            if self.history[k] == 0:
                del self.history[k]

    def clear(self):
        self.history = defaultdict(lambda: 10)

    def speak(self, text):
        req = SoundRequest(
            command=SoundRequest.PLAY_ONCE,
            sound=SoundRequest.SAY,
            arg=text,
            arg2=self.language)
        if "volume" in req.__slots__:
            req.volume = 1.0
        if self.speak_action is not None:
            self.speak_action.send_goal(
                SoundRequestGoal(sound_request=req))
            if self.wait:
                self.speak_action.wait_for_result(rospy.Duration(20.0))
        elif self.speak_pub.get_num_connections() > 0:
            self.speak_pub.publish(req)
        else:
            rospy.logerr("No speech to text service found: {}".format(
                rospy.resolve_name("robotsound")))
            rospy.loginfo(text)

    def run(self):
        while not self.event.wait(self.rate):
            e = self.pop()
            if e:
                rospy.logdebug("audible warning speaking: %s" % e)
                sentence = e.name + ' ' + e.message
                sentence = sentence.replace('/', ' ')
                self.speak(sentence)
            self.sweep()


class AudibleWarning(object):
    def __init__(self):
        speak_rate = rospy.get_param("~speak_rate", 1.0)
        speak_wait = rospy.get_param("~speak_wait", True)
        blacklist = rospy.get_param("~blacklist", list())
        language = rospy.get_param("~language", "ja-JP")
        self.speak_thread = SpeakThread(speak_rate, speak_wait, blacklist, language)

        # run-stop
        self.run_stop = True
        if rospy.resolve_name("runstop") != "runstop":
            rospy.logwarn("Topic 'runstop' is not remapped."
                          "The robot will speak warnings at anytime")
            self.run_stop = False
        else:
            self.sub_run_stop = rospy.Subscriber(
                "runstop", Bool,
                self.runstop_cb, queue_size=1)

        # diag
        self.sub_diag = rospy.Subscriber("/diagnostics_agg", DiagnosticArray,
                                         self.diag_cb, queue_size=1)
        self.speak_thread.start()

    def on_shutdown(self):
        self.speak_thread.stop()
        self.speak_thread.join()

    def runstop_cb(self, msg):
        self.run_stop = msg.data

    def diag_cb(self, msg):
        if self.run_stop:
            self.speak_thread.clear()
            return

        error = filter(lambda s: s.level == DiagnosticStatus.ERROR, msg.status)
        stale = filter(lambda s: s.level == DiagnosticStatus.STALE, msg.status)
        self.speak_thread.add(stale, error)

if __name__ == '__main__':
    rospy.init_node("audible_warning")
    w = AudibleWarning()
    rospy.on_shutdown(w.on_shutdown)
    rospy.spin()
