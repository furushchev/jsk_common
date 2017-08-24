#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
try:
    from colorama import Fore, Style, init
except:
  print "Please install colorama by pip install colorama"
  sys.exit(1)


keep_flag = True
results = {}
data_column = {
    "FullCapacity": "Full Charge Capacity (mAh)",
    "RemainingCapacity": "Remaining Capacity",
    "Status": "Batery Status",
    "CycleCount": "Cycle Count",
    "ManufactureDate": "Manufacture Date",
}

def callback(data):
    global keep_flag, results
    for s in data.status:
        if s.name.startswith("/Power System/Smart Battery"):
            if s.name not in results:
                results[s.name] = { "HardwareID": s.hardware_id }
            for col, label in data_column.items():
                for kv in s.values:
                    if kv.key.startswith(label):
                        try:
                            results[s.name][col] = int(kv.value)
                        except:
                            results[s.name][col] = kv.value
                        continue
    keep_flag = False

def getColor(result):
    try:
        cap = result["FullCapacity"]
        if cap > 5500:
            return Fore.GREEN
        elif cap > 4000:
            return Fore.YELLOW
        else:
            return Fore.RED
    except:
        return ""

def output():
    global results
    sorted_keys = ["HardwareID"] + sorted(data_column.keys())
    sorted_names = sorted(results)
    fmt = "{:>31}" + ("|{:>15}" * len(sorted_keys))
    print fmt.format("Battery Name", *sorted_keys)
    for name in sorted_names:
        color = getColor(results[name])
        v = [results[name][k] if k in results[name] else "N/A" for k in sorted_keys]
        print color + fmt.format(name, *v) + Fore.RESET

if __name__ == '__main__':
    init()
    rospy.init_node('battery_summary')
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, callback)

    while not rospy.is_shutdown() and keep_flag:
        print "aggregating battery info..."
        rospy.sleep(1)

    output()
