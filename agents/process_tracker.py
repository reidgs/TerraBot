import rospy, rosnode, rosgraph
import sys, select, time, socket
from os.path import exists
from lib.terrabot_utils import clock_time
#import send_email as semail

tracker_file = "tracker.txt"
period = 1

import argparse
parser = argparse.ArgumentParser(description = "Process Tracker")
parser.add_argument('-t', '--trackerfile', default=tracker_file,
                    help="File to save tracker output; default is %s" %tracker_file)
parser.add_argument('-p', '--period', type=int, default=period,
                    help="How often to check for processes; default is %d" %period)
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
args = parser.parse_args()

def init_ros ():
    if args.sim: rospy.set_param("use_sim_time", True)
    rospy.init_node("process_tracker", anonymous = True)
    # Wait for clock to start up correctly
    while rospy.get_time() == 0: rospy.sleep(0.1)
    print("Connected and ready for tracking")

# Remove the initial '/' and the last two parts preceded by '_'
def process_procname(procname):
    underline = procname.rfind('_')
    if (underline >= 0):
        underline = procname.rfind('_', 0, underline-1)
        procname = procname[:underline]
    return procname[1:] # Remove initial '/'

def process_tracker_file(tracker_file):
    if (not exists(tracker_file)):
        return set()
    else:
        line = None
        with open(tracker_file, "r") as tf:
            for line in tf:
                pass
        if (not line): return set() # No lines in file, yet
        # 'line' is the last line in the tracker file
        proc_names = line.replace('\n','').split(' ')[1:]
        print("Last set of processes: ", proc_names)
        return set(proc_names)

def process_current_procs():
    procs = [process_procname(proc) for proc in rosnode.get_node_names()]
    return set(procs)

def check_for_quit(period):
    if sys.stdin in select.select([sys.stdin],[],[],period)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q': exit()
    
def notify_change(previous_procs, current_procs):
    hostname = socket.gethostname()
    notification = "Change in processes for %s:\n" %hostname
    added = current_procs.difference(previous_procs)
    if (len(added) > 0):
        notification += "   Added: %s\n" %(' '.join(list(added)),)
    removed = previous_procs.difference(current_procs)
    if (len(removed) > 0): 
        notification += "   Removed: %s\n" %(' '.join(list(removed)),)
    print(notification, end="")
    '''
    semail.send('terrabot0@outlook.com', 'Simmons482',
                'reids@cs.cmu.edu', "Changes in %s processes" %hostname,
                notification)
    '''

def run_tracker(tf, previous_procs, period):
    init_ros()
    last_time = rospy.get_time()
    while rosgraph.is_master_online():
        current_procs = process_current_procs()
        if (previous_procs != current_procs):
            sorted_procs = sorted(list(current_procs),
                                  key=lambda s: s.casefold())
            '''
            print("%s %s" %(clock_time(rospy.get_time()),
                            ' '.join(sorted_procs)))
            '''
            tf.write("%s %s\n" %(clock_time(rospy.get_time()),
                                 ' '.join(sorted_procs)))
            tf.flush()
            notify_change(previous_procs, current_procs)    
            previous_procs = current_procs
        check_for_quit(period)

def wait_for_ros():
    print("%s Roscore died; Waiting to reconnect" %clock_time(rospy.get_time()))
    while not rosgraph.is_master_online():
        check_for_quit(1)

while True:
    with open(args.trackerfile, "a") as tf:
        run_tracker(tf, process_tracker_file(args.trackerfile), args.period)
    wait_for_ros()
    '''
    hostname = socket.gethostname()
    semail.send('terrabot0@outlook.com', 'Simmons482',
                'reids@cs.cmu.edu', "Roscore died on %s" %hostname,
                "Waiting to reconnect")
    '''
