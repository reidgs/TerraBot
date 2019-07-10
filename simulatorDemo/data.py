import math as m

maxLight = 60.0 #max point of graph
daylight = 15.0/24 #percent of light in a day

initialWater = 50


#sunlight cycle
def light_output(time):
    global maxLight
    global daylight
    amp = maxLight/(2*daylight)
    return amp * m.sin(2*m.pi*time/86400) + maxLight - amp



##  will need to change to consider
##  -plant growth vs water consumption
##  -evaporation? 
##  -really how much water is being consumed...

#evaporation/usage of initial water
def water_output(time):
    global initialWater
    rate = 1.0/3
    return 50 - rate*time









