import rospy
import subprocess as sp

sensor_vars = {}
actuator_vars = {}

bfile = ""

comms = []
comm_ind = 0
comm_start = rospy.Time(0)

finished = False

def open_trace(path):
    global comms, bfile
    lines = open(path).readlines()
    bfile = lines[0].strip()
    comms = [line.strip().split(",") for line in lines[2:]]
    return bfile

def run_command(time):
    global comms,comm_ind, finished
    curr_comm = comms[comm_ind]
    if curr_comm[0] == 'QUIT':
        finished = True
    print(curr_comm)
    comm_ind += 1









