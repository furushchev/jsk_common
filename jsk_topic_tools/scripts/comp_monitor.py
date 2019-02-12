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
        self.header = None
        self.pub_stat = rospy.Publisher(
            "~output", ComputerStatus, queue_size=1)
        self.pub_timer = rospy.Timer(
            rospy.Duration(rospy.get_param("~publish_rate", 1.0)),
            self.timer_callback)

    def timer_callback(self, event=None):

        cpu_stats, avg_stats = self.get_cpu_stats()

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
        msg.cpu_all_usage = avg_stats[0]
        msg.cpu_all_user = avg_stats[1]
        msg.cpu_all_nice = avg_stats[2]
        msg.cpu_all_system = avg_stats[3]
        msg.cpu_all_idle = avg_stats[4]

        self.pub_stat.publish(msg)

    def get_cpu_stats(self):
        try:
            p = sp.Popen('mpstat -P ALL 1 1',
                         stdout=sp.PIPE, stderr=sp.PIPE,
                         shell=True, env={'LC_ALL': 'C'})
            stdout, stderr = p.communicate()
            retcode = p.returncode

            if retcode != 0:
                rospy.logerr("Failed to monitor cpu status")
                return

            rows = stdout.split(os.linesep)

            # find header
            if not self.header:
                header = {'%usr': 2,
                          '%nice': 3,
                          '%sys': 4,
                          '%idle': -1}
                for r in rows:
                    if r.find('%usr') >= 0:
                        for i, n in enumerate(r.strip().split()):
                            header[n] = i
                        break
                else:
                    rospy.logwarn('Header not found. Using default header')

                self.header = header

            stats = []
            avg_stats = ()
            for index, row in enumerate(rows):
                if index < 3:
                    continue

                lst = row.split()
                if len(lst) < 8:
                    continue

                # Ignore 'Average: ...' data
                if lst[0].startswith('Average'):
                    continue

                idle = lst[self.header['%idle']].replace(',', '.')
                user = lst[self.header['%usr']].replace(',', '.')
                nice = lst[self.header['%nice']].replace(',', '.')
                system = lst[self.header['%sys']].replace(',', '.')

                usage = float(user) + float(nice)
                if usage > 1000: # wrong reading, use old reading instead
                    rospy.logwarn('Read cpu usage of %f percent. Reverting to previous reading of %f percent'%(usage, self.usage_old))
                    usage = self.usage_old
                self.usage_old = usage
                if row.find('all') >= 0:
                    avg_stats = (float(usage), float(user), float(nice),
                                 float(system), float(idle))
                    rospy.loginfo('usage=%.2f user=%.2f nice=%.2f system=%.2f idle=%.2f', *avg_stats)
                else:
                    stats.append((float(usage), float(user), float(nice),
                                  float(system), float(idle)))
            return stats, avg_stats
        except Exception, e:
            rospy.logerr(e)
            import traceback
            rospy.logerr(traceback.format_exc())



if __name__ == '__main__':
    rospy.init_node("comp_monitor")
    c = CompMonitor()
    rospy.spin()
