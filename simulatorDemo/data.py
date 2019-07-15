import math as m

max_light = 60.0 #max point of graph
daylight = 15.0/24 #percent of light in a day
temp = 50
hum = 100
initial_water = 50
tds = 97
cur = 0
wpump_rate = 1.83


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
def level_data(time):
    global initial_water, wpump_rate
  #  rate = 1.0/3.0 
    return 50.0 - 0.05*time*wpump_rate/200.0

def tds_data(time): 
    global tds
    return tds

def temp_data(time):
    global temp
    return temp

def hum_data(time):
    global hum
    return hum

def cur_data(time):
    global cur
    return cur
