from datetime import datetime
from terrabot_utils import dtime_to_seconds, clock_time, time_since_midnight
from terrabot_utils import Agenda

sensor_names = ['light', 'temperature', 'humidity', 'smoist', 'wlevel',
                'current']

class Baseline(Agenda):
    def __init__(self, filename, time0): # time0 is midnight of the first day
        self.time0 = time0
        last_time = time0
        if (not filename): return
        with open(filename) as f:
            for line in f.readlines():
                l = line.split('#')[0].strip(' \n')
                if (l.find('START AT') == 0) :
                    time = dtime_to_seconds(l[len("START AT ")])
                    actions = []
                    self.add_to_schedule([time, actions])
                    self.time0 = time
                    print("START: %s" %clock_time(time))
                    last_time = time
                elif (l.find('AT') == 0):
                    time = dtime_to_seconds(l[len("AT ")]) + time0
                    if (time < last_time):
                        print("Time must run forward: %s" %l)
                        quit()
                    last_time = time
                    actions = []
                    self.add_to_schedule([time, actions])
                elif (len(l) > 0):
                    # Should be "sensor = value(s)"
                    sensor = l[:l.find(" ")]
                    if (not sensor in sensor_names):
                        print("%s not a legal baseline sensor name" %sensor)
                        quit()
                    sensor_val = l[l.find("=")+1:]
                    actions.append([sensor, sensor_val])

    def update(self, time, sensor_values):
        if (not self.finished() and (time >= self.schedule[self.index][0])):
            print("Updating baseline at %s:" %clock_time(time))
            for a in self.schedule[self.index][1]: 
                sensor_values[a[0]] = eval(a[1])
                #print("  %s: %s" %(a[0], sensor_values[a[0]])) 
            self.index += 1
            return True
        else: return False

    def display(self):
        for s in self.schedule:
            # Not sure why this prints right - but it does
            print("%sAT %s" %("START " if (self.time0 == s[0]) else "",
                              clock_time(s[0])))
            for sv in s[1]:
                print("  %s = %s" %(sv[0], sv[1]))

if __name__ == '__main__':
    import sys, time
    if (len(sys.argv) == 2):
        now = time.time()
        time0 = now - time_since_midnight(now)
        baseline = Baseline(sys.argv[1], time0)
        baseline.display()
        #"""
        # Testing how baseline works
        t = baseline.time0; sensor_values = {}
        while not baseline.finished():
            baseline.update(t, sensor_values)
            t += 1
        #"""
    else:
        print("Need to provide baseline file to parse")
