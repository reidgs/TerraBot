#from __future__ import print_function
from ortools.sat.python import cp_model
import matplotlib.pyplot as plt


def main():
    # Hard-coded values
    actions = ["camera","lightsOn","lightsOff","raiseTemp","lowerTemp","raiseHum","lowerHum","raiseMoist","lowerMoist"]
    num_actions = len(actions)
    num_halfhours = 48
    all_halfhours = range(num_halfhours)
    
    # Creates the model.
    model = cp_model.CpModel()

    # Creates Task-Time variables for each action for each half hour
    #1 = scheduled, 0 = unscheduled
    task = {}
    for a in actions:
        for h in all_halfhours:
            if h%2 == 0:
                task[(a,h)] = model.NewBoolVar(a+'@'+str(h//2)+":00")
            else:
                task[(a,h)] = model.NewBoolVar(a+'@'+str(h//2)+":30")


    '''
    Action actuator constraints:
    LoweringTemperature: fan on (optional lights off)
    RaisingTemperature: lights on (optional fan off)
    LoweringHumidity: fan on, water off
    RaisingHumidity: fan off, water on
    LoweringSoilMoisture: fan on, water off
    RaisingSoilMoisture: fan off, water on
    LightsOn: lights on
    LightsOff: lights off
    Camera: lights off
    '''
    #Add more constraints for actions that can be completed simultaneously
    for h in all_halfhours:
        model.Add(sum([task[("lightsOn",h)],task[("lightsOff",h)]]) == 1)
        model.Add(sum([task[("lowerTemp",h)],task[("raiseTemp",h)]]) <= 1)

    #lights on for >=14 hours
    #lights off >= 7 hours from 8pm to 6am
    #insert here

    #camera at least every 3 hours and at most 2 per 6 hours
    # no camera at night
    minhalfhours = 3*2
    maxhalfhours = 6*2
    #insert here

    #check all other variables every 4 hours
    maintain = 4*2
    #insert here

    # Creates the solver and solve.
    solver = cp_model.CpSolver()
#    solver.parameters.linearization_level = 0
    status = solver.Solve(model)

    #print them all out
    if (status == cp_model.FEASIBLE):
        for h in all_halfhours:
            if h%2 == 0:
                print(str(h//2)+":00")
            else:
                print(str(h//2)+":30")
            for a in actions:
                try: #if a variable wasn't scheduled, just pass
                    if solver.Value(task[(a,h)]) == 1:
                        print(a)
                except:
                    pass
        activities = []
        for a in actions:  
            acts = [a] + [task[t] for t in task if t[0] == a]
            activities.append(acts)
        #plot them
        plot_activities(solver, activities)

#############################################################
#plotting tools
#############################################################
# name is of the form act@hh:mn
def time_from_name(name):
    l = name.split('@')[1].split(':')
    t = int(l[0])*60 + int(l[1])
    print("%s: %d" %(name, t))
    return t

def plot_activities(solver, activities):
    # Declaring a figure "gnt" 
    fig, gnt = plt.subplots() 
    fig.set_size_inches(12,0.5*len(activities))

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
    gnt.set_yticklabels([act[0] for act in activities])
  
    # Setting graph attribute 
    gnt.grid(True) 

    colors = ('blue', 'red', 'orange', 'green', 'yellow')
    for a in range(len(activities)):
        act = activities[a]
        bars = []
        for i in range(1, len(act)):
            print(act[i])
            if (solver.Value(act[i]) == 1):
                start = time_from_name(act[i].Name())
                bars.append([start, 30])
        gnt.broken_barh(bars, [(3*a+1), 2], facecolors=colors[a%len(colors)],
                        edgecolor='black')

    plt.tight_layout()
    plt.ion()
    plt.show()
    i = input("Ready?")#keep plot up until keyboard press

if __name__ == '__main__':
    main()


