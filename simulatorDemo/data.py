import math as m

max_light = 60.0 #max point of graph
daylight = 15.0/24 #percent of light in a day

initial_water = 50


#sunlight cycle
def light_data(time):
    global max_light
    global daylight
    amp = max_light/(2.0*daylight)
    return amp * m.sin(2*m.pi*time/86400) + max_light - amp



##  will need to change to consider
##  -plant growth vs water consumption
##  -evaporation? 
##  -really how much water is being consumed...

#evaporation/usage of initial water
def water_data(time):
    global initial_water
    rate = 1.0/3.0
    return 50.0 - rate*time

