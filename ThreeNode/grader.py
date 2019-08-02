import rospy

grader_vars = {}

bfile = ""

cmds = []
cmd_ind = 0
cmd_start = rospy.Time(0)

finished = False

def open_trace(path):
    global cmds, bfile, grader_vars
    lines = open(path).readlines()
    bfile = lines[0].strip()
    cmds = [line.strip().split(",") for line in lines[2:]]

    try:
        exec(open(bfile).read())
        grader_vars = init_actuators
    except:
        print('no baseline file found')
        exit()

    return bfile

def run_command(time):
    global cmds,cmd_ind,cmd_start,finished
    curr_cmd = cmds[cmd_ind]
    if curr_cmd[0] == 'START':
        cmd_ind += 1
        cmd_start = time
        return 0
    elif curr_cmd[0] == 'QUIT':
        cmd_ind += 1
        finished = True
        return 0
    elif curr_cmd[0] == 'WAIT':
        res = eval(curr_cmd[1])
        timeout = float(curr_cmd[2])

        if res: #success!
            print("success: %s"%str(curr_cmd))
            cmd_start = time
            cmd_ind += 1
            return 1
        elif (time - cmd_start).to_sec() > timeout:
            print("fail: %s"%str(curr_cmd))
            cmd_start = time
            cmd_ind += 1
            return -1

    elif curr_cmd[0] == 'ENSURE':
        res = eval(curr_cmd[1])
        timeout = float(curr_cmd[2])

        if (time - cmd_start).to_sec() > timeout and res:
            print("success: %s"%str(curr_cmd))
            cmd_start = time
            cmd_ind += 1
            return 1
        elif not res: #fail
            print("fail: %s"%str(curr_cmd))
            cmd_start = time
            cmd_ind += 1
            return -1

    cmd_ind += 0


