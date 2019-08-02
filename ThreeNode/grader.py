import rospy
import subprocess as sp

sensor_vars = {}
actuator_vars = {}

bfile = ""

cmds = []
cmd_ind = -1
cmd_start = rospy.Time(0)

finished = False

def open_trace(path):
    global cmds, bfile
    lines = open(path).readlines()
    bfile = lines[0].strip()
    cmds = [line.strip().split(",") for line in lines[2:]]
    return bfile

def run_command(time):
    global cmds,cmd_ind, finished
    cmd_ind += 1
    if cmds[cmd_ind][0] == 'QUIT':
        finished = True
    return cmds[cmd_ind]









