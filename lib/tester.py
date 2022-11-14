#import rospy
import copy, os, sys
from datetime import datetime
from terrabot_utils import clock_time, time_since_midnight, dtime_to_seconds
from terrabot_utils import Agenda
#from limits import optimal, limits

def parse_error(line):
    raise Exception("Unknown syntax: %s" % line)

def is_abs_time(time_str):
    try:
        datetime.strptime(time_str, "%d-%H:%M:%S")
        return True
    except ValueError: return False

class Constraint:
    condition = None
    rel_time = None
    abs_time = None
    timeout = 0

    def __init__(self, line): return None
    def _str_(self): return "[Generic constraint]"
    # Returns -1 (constraint is false), +1 (constraint is true), 0 (still TBD)
    def evaluate(self, time, vars): return 0

    def has_time(self): return self.abs_time != None or self.rel_time != None
    def set_timeout(self, time, time0):
        self.timeout = (time + self.rel_time if (self.rel_time != None) else
                        time if (self.abs_time == None) else
                        (time0 - time_since_midnight(time0) +
                         dtime_to_seconds(self.abs_time)))
    def evaluate_time(self, time): return time >= self.timeout
    def evaluate_condition(self, vars):
        for key in vars: locals()[key] = vars[key]
        return eval(self.condition)

    def time_str(self, rel_word, abs_word):
        return ("%s %d" %(rel_word, self.rel_time) if self.rel_time != None else
                "%s %s" %(abs_word, self.abs_time) if self.abs_time != None else "")
    def parse_time(self, line, rel_word, abs_word):
        if (line.find(rel_word) > 0):
            self.rel_time = int(line.split(rel_word)[1])
        elif (line.find(abs_word) > 0):
            self.abs_time = line.split(abs_word)[1].strip()
    def parse_condition_time(self, line):
        self.parse_time(line, "FOR", "UNTIL")
        self.condition = (line if not self.has_time() else
                          line.split("FOR" if self.rel_time != None else "UNTIL")[0])
        self.condition = self.condition.strip()

class WaitConstraint(Constraint):
    def __init__(self, line):
        self.parse_condition_time(line[len("WAIT"):])
    def __str__(self):
        return "[WAIT %s %s]" % (self.condition, self.time_str("FOR", "UNTIL"))
    def evaluate(self, time, vars):
        cres = (self.evaluate_condition(vars) if self.condition else None)
        tres = self.evaluate_time(time)
        return (1 if tres and cres == None else 1 if cres else -1 if tres else 0)

class EnsureConstraint(Constraint):
    def __init__(self, line):
        self.parse_condition_time(line[len("ENSURE"):])
    def __str__(self):
        return "[ENSURE %s %s]" %(self.condition, self.time_str("FOR", "UNTIL"))
    def evaluate(self, time, vars):
        cres = self.evaluate_condition(vars)
        tres = (self.evaluate_time(time) if self.has_time() else None)
        return (-1 if not cres else 1 if tres == None or tres else 0)

class SetConstraint(Constraint):
    var = None
    def __init__(self, line, vars):
        s = line[len("SET"):].split('=')
        self.condition = s[1].strip()
        self.var = s[0].strip()
        vars[self.var] = 0
    def __str__(self):
        return "[SET %s = %s]" %(self.var,  self.condition)
    def evaluate(self, time, vars):
        vars[self.var] = self.evaluate_condition(vars)
        return 1

class PrintConstraint(Constraint):
    condition = ""

    def __init__(self, line, vars):
        self.condition = line[len("PRINT"):]
    def __str__(self):
        return "[PRINT %s]" %self.condition
    def evaluate(self, time, vars):
        print("PRINT (%s): %s"
              %(clock_time(time), self.evaluate_condition(vars)))
        return 1

class DelayConstraint(Constraint):
    def __init__(self, line):
        self.parse_time(line[len("DELAY"):], 'FOR', 'UNTIL')
    def __str__(self):
        return "<DELAY %s>" %self.time_str("FOR", "UNTIL")
    def evaluate(self, time, vars): return int(self.evaluate_time(time))

class EndConstraint(Constraint):
    type = 'STOP'
    def __init__(self, line):
        self.type = line[0:4]
        self.parse_time(line[4:], 'AFTER', 'AT')
    def __str__(self):
        return "<%s %s>" %(self.type, self.time_str("AFTER", "AT"))
    def evaluate(self, time, vars): return int(self.evaluate_time(time))

class WheneverConstraint(Constraint):
    agenda = None
    conditionP = False
    parent = None
    def __init__(self, line):
        self.agenda = Agenda()
        trigger = line[len("WHENEVER"):].strip()
        if (is_abs_time(trigger)): self.abs_time = trigger
        else: self.condition = trigger
    def __str__(self):
        str = "[WHENEVER %s" %(self.condition if self.condition != None else
                               self.abs_time)
        for constraint in self.agenda.schedule:
            str = str + "\n  " + constraint.__str__()
        return str + "]"
    def brief(self):
        return "[WHENEVER %s ...]" %(self.condition if self.condition != None
                                     else self.abs_time)
    def evaluate(self, time, vars):
        if (self.condition != None):
            last_eval = self.conditionP
            self.conditionP = self.evaluate_condition(vars)
            return int(not last_eval and self.conditionP)
        else:
            self.conditionP = self.evaluate_time(time)
            return int(self.conditionP)

    def activate(self, time):
        child = copy.deepcopy(self)
        child.parent = self
        # Update the timeout so it doesn't keep triggering
        if (self.abs_time != None): self.timeout += 24*60*60 # Add a day

        if (child.agenda.time0 == None):
            child.agenda.time0 = time - time_since_midnight(time)
        first_constraint = child.agenda.schedule[0]
        first_constraint.set_timeout(time, child.agenda.time0)
        #print("First timeout: %s for %s" %(clock_time(first_constraint.timeout), first_constraint))
        return child

    def deactivate(self):
        self.parent.conditionP = False

    def evaluate_agenda(self, time, vars):
        curr_constraint = self.agenda.schedule[self.agenda.index]
        res = curr_constraint.evaluate(time, vars)
        #print("Test: %s %s: %s: %d" %(curr_constraint, curr_constraint.timeout, time, res))
        if (res == 1):
            self.next_constraint(time)
            # Keep testing, since subsequent clauses (e.g., SET) may already hold
            if (not self.agenda.finished()):
                res = self.evaluate_agenda(time, vars)
        return res

    def next_constraint(self, time):
        self.agenda.index += 1
        if (not self.agenda.finished()):
            next_constraint = self.agenda.schedule[self.agenda.index]
            next_constraint.set_timeout(time, self.agenda.time0)
            #print("Next timeout: %s for %s" %(clock_time(next_constraint.timeout), next_constraint))

    def print_status(self, res, time):
        if (res == 1):
            print("SUCCESS (%s): %s" %(clock_time(time), self.brief()))
            pass
        elif (res == -1):
            curr_constraint = self.agenda.schedule[self.agenda.index]
            print("FAILURE (%s): at %s in %s"
                  %(clock_time(time), curr_constraint, self.brief()))

class Tester:
    vars = { 'light'    : [10,100],  'temperature' : [20,20],
             'humidity' : [40,50],   'smoist'      : [350,350],
             'current'  : [0.0,0.0], 'wlevel'      : 150.0,
             'led'      : 0,         'wpump'       : False,
             'fan'      : False,     'camera'      : None,
             'ping'	: False,     'weight'      : 0 }
    baseline_file = None
    interf_file = None
    delay_time = None
    end_time = None
    constraints = []
    active = []

    def parse_file(self, filename):
        with open(filename) as f:
            for line in f.readlines():
                line = line.split('#')[0].strip(' \n\r')
                if (line.startswith("BASELINE")):
                    self.baseline_file = line.split('=')[1].strip()
                elif (line.startswith("INTERFERENCE")):
                    self.interf_file = line.split('=')[1].strip()
                elif (line.startswith("QUIT") or line.startswith("STOP")):
                    if (self.end_time):
                        print("WARNING: Quit/Stop constraint already specified; overriding")
                    self.end_time = EndConstraint(line)
                elif (line.startswith("DELAY")):
                    if (self.delay_time):
                        print("WARNING: Delay constraint already specified; overriding")
                    self.delay_time = DelayConstraint(line)
                elif (line.startswith("WAIT")):
                    self.add_to_whenever(WaitConstraint(line))
                elif (line.startswith("ENSURE")):
                    self.add_to_whenever(EnsureConstraint(line))
                elif (line.startswith("SET")):
                    self.add_to_whenever(SetConstraint(line, self.vars))
                elif (line.startswith("WHENEVER")):
                    self.constraints.append(WheneverConstraint(line))
                elif (line.startswith("PRINT")):
                    self.add_to_whenever(PrintConstraint(line, self.vars))
                elif (len(line) > 0):
                    parse_error("'%s' %d" %(line, len(line)))

    def add_to_whenever(self, cmd):
        if (len(self.constraints) == 0):
            parse_error("%s is not within a 'whenever' statement" % cmd)
        else:
            self.constraints[-1].agenda.add_to_schedule(cmd)

    def set_delay_time(self, time0):
        if (self.delay_time): self.delay_time.set_timeout(time0, time0)

    def ready_to_start(self, time):
        return (self.delay_time == None or self.delay_time.evaluate(time, []) == 1)

    def set_end_time(self, time0):
        if (self.end_time): self.end_time.set_timeout(time0, time0)

    def finished(self, time):
        return (self.end_time != None and self.end_time.evaluate(time, []) == 1)

    def end_status(self):
        return (None if not self.end_time else self.end_time.type)

    def evaluate_whenever(self, whenever, time):
        res = whenever.evaluate_agenda(time, self.vars)
        if (res == -1 or whenever.agenda.finished()):
            whenever.print_status(res, time)
            self.deactivate(whenever)

    def activate(self, whenever, time):
        active = whenever.activate(time)
        print("ACTIVATE (%s): %s" %(clock_time(time), active.brief()))
        self.active.append(active)

    def deactivate(self, whenever):
        whenever.deactivate()
        self.active.remove(whenever)

    def init_constraints(self, time0):
        #print("Start time: %s" %clock_time(time0))
        self.set_delay_time(time0)
        self.set_end_time(time0)
        for constraint in self.constraints:
            constraint.set_timeout(time0, time0)

    def process_constraints(self, time):
        # Wait for the delay, if any
        if (not self.ready_to_start(time)): return
        elif (self.delay_time):
            print("Tester: Initial delay achieved")
            self.delay_time = None

        # Trigger any of the 'whenever' constraints
        for whenever in self.constraints:
            if (not whenever.conditionP and
                whenever.evaluate(time, self.vars)):
                self.activate(whenever, time)

        # Process currently active 'whenever' constraints
        for whenever in self.active:
            self.evaluate_whenever(whenever, time)

    def display(self):
        if (self.baseline_file): print("BASELINE: '%s'" %self.baseline_file)
        if (self.interf_file): print("INTERFERENC: '%s'" %self.interf_file)
        if (self.delay_time):
            print("DELAY %s" %self.delay_time.time_str("FOR", "UNTIL"))
        if (self.end_time):
            print("%s %s" %(self.end_time.type, self.end_time.time_str("AFTER", "AT")))
        for cmd in self.constraints: print(cmd)

if __name__ == '__main__':
    import sys, time
    if (len(sys.argv) == 2):
        tester = Tester()
        tester.parse_file(sys.argv[1])
        #tester.display()
        now = time.time()
        time0 =  now - time_since_midnight(now)
        t = time0
        tester.init_constraints(time0)
        while not tester.finished(t):
            tester.process_constraints(t)
            t += 1
    else:
        print("Need to provide one grader file to parse")

