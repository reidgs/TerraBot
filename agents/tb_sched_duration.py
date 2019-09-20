from ortools.sat.python import cp_model
import matplotlib.pyplot as plt

# name is the name of the activity
# domain is a pair for the min/max times the activities can be scheduled
# duration can be an integer or an IntVar (giving a duration range)
# separation is a pair for min/max separation between activities
# resources is a dict of resource:value pairs
class Activity():
    name = ""
    scheduled = []
    starts = []
    ends = []
    intervals = []
    resources = {}

    def __init__(self, model, name, domain, duration, separation,
                 number, resources):
        min_separation = separation[0]; max_separation = separation[1]
        min_num = number[0]; max_num = number[1]
        self.name = name
        self.resources = resources
        self.scheduled = [model.NewBoolVar('%s-i%d-scheduled' %(name, i))
                          for i in range(max_num)]
        self.starts = [model.NewIntVar(domain[0], domain[1],
                                       '%s-s%d' %(name, i))
                       for i in range(max_num)]
        self.ends = [model.NewIntVar(domain[0], domain[1],
                                     '%s-e%d' %(name, i))
                       for i in range(max_num)]
        self.intervals = [model.NewOptionalIntervalVar(self.starts[i],
                                                       duration,
                                                       self.ends[i],
                                                       self.scheduled[i],
                                                       '%s-i%d' %(name, i))
                          for i in range(max_num)]
        for i in range(max_num-1):
            # Ensure that the intervals are scheduled in sequential order
            #  (i.e,. if interval i is scheduled, then interval i+1 is also)
            model.Add(self.scheduled[i]==1).OnlyEnforceIf(self.scheduled[i+1])

            # Adhere to the min and max separation constraints between
            #   scheduled activities
            # Insert your code here

            # Make sure that the scheduled intervals span the domain
            # Insert your code here

        # Adhere to the min & max number constraints
        # Insert your code here

def plot_activities(solver, activities):
    # Declaring a figure "gnt" 
    fig, gnt = plt.subplots() 
    fig.set_size_inches(12, max(1, 0.5*len(activities)))

    # Setting axes limits 
    gnt.set_xlim(0, 24*60)
    gnt.set_ylim(0, 3*len(activities)+1) 
  
    # Setting axes labels
    gnt.set_xlabel('Hours Since Midnight') 
    gnt.set_ylabel('Activity') 
  
    # Setting axes ticks and labels
    gnt.set_xticks([60*i for i in range(0,25)])
    gnt.set_xticklabels([i for i in range(0,25)])

    gnt.set_yticks([3*i-1 for i in range(1,len(activities)+1)])
    gnt.set_yticklabels([act.name for act in activities])
  
    # Setting graph attribute 
    gnt.grid(True) 

    colors = ('blue', 'red', 'orange', 'green', 'yellow')
    for a in range(len(activities)):
        act = activities[a]
        bars = []
        for i in range(len(act.intervals)):
            if (solver.Value(act.scheduled[i]) == 1):
                start = solver.Value(act.starts[i])
                bars.append([start, solver.Value(act.ends[i])-start])
        gnt.broken_barh(bars, [(3*a+1), 2], facecolors=colors[a%len(colors)],
                        edgecolor='black')

    plt.tight_layout()
    plt.ion()
    plt.show()

# Return False if the resource lists are inconsistent
#   (i.e., same resource used in conflicting ways)
def check_resources(res1, res2):
    for r in res1.items():
        r2 = res2.get(r[0])
        if (r2 != None and r[1] != r2): return False
    return True

# No overlap if the activities use the same resource in conflicting ways
def separate_resource_usage(model, act1, act2):
    if (not check_resources(act1.resources, act2.resources)):
        print("%s and %s should not overlap" %(act1.name, act2.name))
        # Ensure that intervals with conflicting resources do not overlap
        # Insert your code here

def print_scheduled(solver, acts):
    for i in range(len(acts.intervals)):
        if (solver.Value(acts.scheduled[i]) == 1):
            print('%s: %d..%d' %(acts.intervals[i].Name(),
                                 solver.Value(acts.starts[i]),
                                 solver.Value(acts.ends[i])))
        else:
            print('%s: Not scheduled' %acts.intervals[i].Name())

def main():
    model = cp_model.CpModel()

    # Define all your activities here
    # Lights on for >=14 hours
    # Lights off >= 7 hours from 8pm to 6am
    # Camera at least every 3 hours and at most 2 per 6 hours
    # Raising/lowering temp/humid/moisture - at least every 4 hours,
    #   but could be much more frequent then that, depending on how quickly
    #   things change
    # For instance - lowering humidity (turning fans on while not watering)
    #   occurs during the span midnight to midnight (0 - 1440), has a
    #   duration of 10 minutes, scheduled at least 1 hour apart (so,
    #   no more than 25 needed), but at least one every 4 hours
    lower_humid = Activity(model, 'lowerHumid', (0, 24*60), 10, (1*60, 4*60),
                           (6, 25), {'fan' : 'on', 'pump' : 'off'})

    activities = [lower_humid] # add all your activities here
    for a1 in activities:
        for a2 in activities:
            if (a1 != a2): separate_resource_usage(model, a1, a2)

    # Optional - choose which activities to maximize and which to minimize
    #model.Maximize(sum(lower_humid.scheduled))
    #model.Minimize(sum(lower_humid.scheduled))

    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    if (status == cp_model.INFEASIBLE):
        print("Infeasible schedule")
    else:
        for a in activities: print_scheduled(solver, a)
        plot_activities(solver, activities)

    print("Branches: %f" %solver.NumBranches())
    print("Wall time: %f" %solver.WallTime())

if __name__ == '__main__':
    main()
    input()
