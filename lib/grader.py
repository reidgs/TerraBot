import rospy
from datetime import datetime
from terrabot_utils import clock_time, time_since_midnight, clock_to_seconds
from terrabot_utils import Agenda

class Command:
    type = 'WAIT'
    constraint = None
    rel_time = None
    abs_time = None
    timeout = None

    def line_str(self):
        return "%s %s %s %s" %(self.type, self.constraint,
                               ("FOR" if self.rel_time != None else "UNTIL"),
                               (self.rel_time if self.rel_time != None else
                                self.abs_time))
def parse_line(line):
    cmd = Command()
    if (line.find("FOR") > 0):
        sl = line.split('FOR')
        cmd.constraint = sl[0].strip()
        cmd.rel_time = int(sl[1])
    elif (line.find("UNTIL") > 0):
        sl = line.split('UNTIL')
        cmd.constraint = sl[0].strip()
        cmd.abs_time = sl[1].strip()
    else:
        print("Unknown syntax: %s" %line)
        quit()
    return cmd

class Grader(Agenda):
    vars = { 'light'    : [10,100],  'temperature' : [20,20],
             'humidity' : [40,50],   'smoist'      : [350,350],
             'current'  : [0.0,0.0], 'wlevel'      : 150.0,
             'led'      : 0,         'wpump'       : False,
             'fan'      : False}
    baseline_file = None
    interf_file = None
    last_cmd = None

    def __init__(self, filename):
        with open(filename) as f:
            for line in f.readlines():
                line =line.split('#')[0].strip('\n')
                if (line.find("BASELINE") == 0):
                    self.baseline_file = line.split('=')[1].strip()
                elif (line.find("INTERFERENCE") == 0):
                    self.interf_file = line.split('=')[1].strip()
                elif (line.find("WAIT") == 0):
                    cmd = parse_line(line[len("WAIT"):])
                    cmd.type = 'WAIT'
                    self.add_to_schedule(cmd)
                elif (line.find("ENSURE") == 0):
                    cmd = parse_line(line[len("ENSURE"):])
                    cmd.type = 'ENSURE'
                    self.add_to_schedule(cmd)
                elif (len(line) > 0):
                    print("Unknown syntax: '%s' %d" %(line, len(line)))
                    quit()

    def next_cmd(self, time):
        self.index += 1
        if (not self.finished()):
            next_cmd = self.schedule[self.index]
            if (next_cmd.rel_time != None):
                next_cmd.timeout = time + next_cmd.rel_time
            else:
                dtime = datetime.strptime(next_cmd.abs_time, "%d-%H:%M:%S")
                next_cmd.timeout = self.time0 + clock_to_seconds(dtime)
            print("  Next timeout: %s" %clock_time(next_cmd.timeout))

    def run_command(self, time):
        curr_cmd = self.schedule[self.index]
        if (self.time0 == None):
            self.time0 = time - time_since_midnight(time)
            curr_cmd.cmd_start = self.time0

        # Set up the local variables:
        light = self.vars.get('light')
        temperature = self.vars.get('temp'); 
        humidity = self.vars.get('humid'); 
        smoist = self.vars.get('smoist'); 
        current = self.vars.get('cur'); 
        wlevel = self.vars.get('level'); 
        wpump = self.vars.get('wpump'); 
        led = self.vars.get('led'); 
        fan = self.vars.get('fan');

        if (curr_cmd != self.last_cmd):
            print("  %s" %curr_cmd.line_str())
            self.last_cmd = curr_cmd
        if (curr_cmd.type == 'WAIT' and len(curr_cmd.constraint) == 0 ):
            if (time > curr_cmd.timeout):
                print("SUCCESS: %s"%curr_cmd.line_str())
                self.next_cmd(time)
                return 1

        elif (curr_cmd.type == 'WAIT'):
            res = eval(curr_cmd.constraint)
            if res: #success!
                print("SUCCESS: %s"%curr_cmd.line_str())
                self.next_cmd(time)
                return 1
            elif (time > curr_cmd.timeout):
                print("FAIL: %s"%curr_cmd.line_str())
                self.next_cmd(time)
                return -1

        elif (curr_cmd.type == 'ENSURE'):
            res = eval(curr_cmd.constraint)
            if (time > curr_cmd.timeout) and res:
                print("SUCCESS: %s"%curr_cmd.line_str())
                self.next_cmd(time)
                return 1
            elif not res: #fail
                print("FAIL: %s"%curr_cmd.line_str())
                self.next_cmd(time)
                return -1

    def update(self, var_name, value):
        self.vars[var_name] = value

    def display(self):
        if (self.baseline_file):
            print("BASELINE: '%s'" %self.baseline_file)
        if (self.interf_file):
            print("INTERFERENC: '%s'" %self.interf_file)
        for cmd in self.schedule:
            print(cmd.line_str())

if __name__ == '__main__':
    import sys, time
    if (len(sys.argv) == 2):
        grader = Grader(sys.argv[1])
        grader.display()
        #"""
        # Testing how grader works
        now = time.time()
        time0 =  now - time_since_midnight(now)
        grader.vars = {'light' : [100, 100], 'temp' : [28, 28],
                       'humid' : [80, 80], 'smoist' : [500, 500],
                       'cur' : [24, 1000], 'level' : 125.5,
                       'wpump' : False, 'led' : False, 'fan' : False}
        t = time0
        while not grader.finished():
            grader.run_command(t)
            t += 1
        #"""
    else:
        print("Need to provide one grader file to parse")
grader_vars = {}

bfile = ""
interf_file = ""

cmds = []
cmd_ind = 0
cmd_start = rospy.Time(0)

finished = False

def open_trace(path):
    global cmds, bfile, interf_file, grader_vars
    lines = open(path).readlines()
    bfile = lines[0].strip()
    interf_file = lines[1].strip()
    cmds = [line.strip().split(",") for line in lines[2:]]
