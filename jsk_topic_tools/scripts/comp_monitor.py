#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
import subprocess as sp
from jsk_topic_tools.msg import ComputerStatus

class CompMonitor(object):
    def __init__(self):
        self.usage_old = None
        self.pub_stat = rospy.Publisher(
            "~output", ComputerStatus, queue_size=1)
        self.pub_timer = rospy.Timer(
            rospy.Duration(rospy.get_param("~publish_rate", 1.0)),
            self.timer_callback)

    def timer_callback(self, event=None):

        cpu_stats = self.get_cpu_stats()

        msg = ComputerStatus()
        try:
            now = event.current_real
        except:
            now = rospy.Time.now()
        msg.header.stamp = now
        msg.num_cpu_cores = len(cpu_stats)
        msg.cpu_usage = [s[0] for s in cpu_stats]
        msg.cpu_user = [s[1] for s in cpu_stats]
        msg.cpu_nice = [s[2] for s in cpu_stats]
        msg.cpu_system = [s[3] for s in cpu_stats]
        msg.cpu_idle = [s[4] for s in cpu_stats]

        self.pub_stat.publish(msg)

    def get_cpu_stats(self):
        try:
            p = sp.Popen('mpstat -P ALL 1 1',
                         stdout=sp.PIPE, stderr=sp.PIPE,
                         shell=True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                rospy.logerr("Failed to monitor cpu status")
                return

            rows = stdout.split(os.linesep)
            col_names = rows[2].split()
            idle_col = -1 if (len(col_names) > 2 and col_names[-1] == '%idle') else -2

            stats = []
            for index, row in enumerate(rows):
                if index < 3:
                    continue

                # Skip row containing 'all' data
                if row.find('all') > -1:
                    continue

                lst = row.split()
                if len(lst) < 8:
                    continue

                ## Ignore 'Average: ...' data
                if lst[0].startswith('Average'):
                    continue

                idle = lst[idle_col].replace(',', '.')
                user = lst[3].replace(',', '.')
                nice = lst[4].replace(',', '.')
                system = lst[5].replace(',', '.')

                usage = float(user) + float(nice)
                if usage > 1000: # wrong reading, use old reading instead
                    rospy.logwarn('Read cpu usage of %f percent. Reverting to previous reading of %f percent'%(usage, self.usage_old))
                    usage = self.usage_old
                self.usage_old = usage

                stats.append((float(usage), float(user), float(nice),
                              float(system), float(idle)))
            return stats
        except Exception, e:
            rospy.logerr(e)
            import traceback
            rospy.logerr(traceback.format_exc())



if __name__ == '__main__':
    rospy.init_node("comp_monitor")
    c = CompMonitor()
    rospy.spin()
