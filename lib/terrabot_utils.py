from datetime import datetime

class Agenda:
    schedule = []
    index = 0
    time0 = None
    def finished(self): return (self.index >= len(self.schedule))
    # Apparently, append and += both change the variable itself - not good
    def add_to_schedule(self, x): self.schedule = self.schedule + [x]

def clock_to_seconds(dtime):
    return (((dtime.day-1)*24 + dtime.hour)*3600 +
            dtime.minute*60 + dtime.second + dtime.microsecond/1e6)

def dtime_to_seconds(dtime):
    return clock_to_seconds(datetime.strptime(dtime, "%d-%H:%M:%S"))

def clock_time(time):
    return datetime.fromtimestamp(time).strftime("%d-%H:%M:%S")

def time_since_midnight(time):
    dtime = datetime.fromtimestamp(time)
    return (dtime.hour*3600 + dtime.minute*60 + dtime.second +
            dtime.microsecond/1e6)

import rclpy
def get_ros_time(node):
    return node.get_clock().now().nanoseconds / 1e9

def set_use_sim_time(node, use_sim_time):
    sim_time = rclpy.parameter.Parameter('use_sim_time',
                                         rclpy.Parameter.Type.BOOL,
                                         use_sim_time)
    node.set_parameters([sim_time])
