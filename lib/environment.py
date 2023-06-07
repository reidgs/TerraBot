from math import sqrt

### ENVIRONMENT ###

#Probably a plant class to store a plants' height, color, location, etc.
#{leaf, root health}
#{health -> affect growth function, tilt, color}
#func to forward time. health is 0<h<1, when time passes the plant will change according to h
#  like if h is close to 1, it will get greener, taller, maybe tilt up? (all depending on temp, light, smoist,... also)
# class Plant {height, color, tilt, health, position}
#update health based on temp, smoist, light,...
             
#Interactions:::: Important to get this digraph worked out
# plants <-- smoist, humidity, light, temperature
# plants --> smoist, humidity
# smoist <-- plants, temperature, wpump
# smoist --> plants, humidity, tankwater
# humidity <-- plants, smoist, tankwater, fan, temp
# humidity --> plants
# temperature <-- light?, fan
# temperature --> plants, smoist, humidity, tankwater
# light <-- led, time
# light --> plants , temp?
# current <-- led, wpump, fan
# volume <-- wpump
# volume --> wpump
# tankwater <-- smoist
# tankwater --> humidity
# led --> light
# wpump <-- volume
# wpump --> smoist, volume
# fan --> humidity, temperature

# Natural Constants #TODO tweak these

max_soilwater = 500        #ml The level at which the soil is fully saturated and will begin to overflow
flow_rate = 3.5             #ml/sec The rate at which the pump will pump water
pipe_capacity = 10          #ml The capacity of the pipe
drip_rate = 1.0             #ml/sec The rate at which water drips from the pipe when the pump is off
uptake_rate = 0.1           #ml/sec The rate at which water is absorbed from the pan to the soil
evap_rate = 1.2             #ml/sec The nominal rate at which water will evaporate
volume_rate = 1000.0 / 45   #ml/mm in the reservoir (used for sensing level)
light_diffuse = .7          #The percentage of sunlight that reaches the other side
max_daylight = 588          #The sunlight right at the window at midday
tank_width = .4             #m the width of the terrarium
led_power = 3.725           #units of light per LED level
room_temp = 20              #degrees C the room temperature out of the greenhouse
room_humidity = 40          # Humidity of air outside greenhouse
fan_cool_rate = .05 / 60    #deg C /min The rate at which temp decreases due to the fan

led_current = 3.2/255       #
pump_current = .2           # The current needed to support each device when it's on
fan_current = .06           #

base_weight = 500           # Weight of support panel, dry rockwool and pan, in grams

#Environment Parameters
params = { 'time' : 0, # This should start at 2000-01-01-00:00:00
           'start' : 0, # This is in seconds, starting at zero

           'humidity' : 50, # Percent
             'soilwater' : 550/2,
             'airwater' : None, # Calculate from humidity
             'temperature' : room_temp,
             
             'volume' : 3000.0,
             'tankwater' : 0.0,
             'pipewater' : 0.0,
             'panwater' : 0.0,
             'energy' : 0,
             
             'led' : 0,
             'wpump' : False,
             'fan' : False}


# Natural Helper Functions 

seconds_in_day = 3600 * 24
sunrise = 3600 * 7
sunset = 3600 * 19
midday = (sunrise + sunset) / 2
coeff1 = (-4.0 / ((sunrise - sunset)**2)) #This is for saving computation
def day_fraction(time):
    #Gives 0 if night, or 0<x<1 if day, where midday is 1 and dawn/dusk is 0, quadratically
    relative_time = time % seconds_in_day #The relative time in the day
    if relative_time <= sunrise or relative_time >= sunset: return 0
    return 1 + coeff1 * ((relative_time - midday)**2)
    
coeff2 = (light_diffuse - 1) / tank_width
def light_level(distance): 
    #Gives light level at a distance from the lit side of the tank TODO change to tanh? or maybe not
    sunlight = max_daylight * (1 + distance * coeff2) * day_fraction(params['time'])
    ledlight = led_power * params['led']
    return min(1000, sqrt(sunlight**2 + ledlight**2)) #This is done pythagorically, there may be a better way
                                                #I think its ok? for the light to be too high at the extreme
                                                #This is NOT Ok if temp is based on light 
    
def light_average():
    #A quick estimate of the average light level
    samples = 5
    return sum([light_level(tank_width * i / samples) for i in range(samples)]) / samples
    
def light_heat_rate():
    #The rate at which the temperature increases (deg C /sec) due to light
    return light_average() * .5 / (3600 * 180)
    
def temp_equil_rate():
    #The rate at which the temperature changes to equilibriate with outside the greenhouse
    #Newton's law of cooling
    return (room_temp - params['temperature']) * (.6 / 10000) #This constant is a guess atm
    
def temp_evap_multiplier():
    #most evaporative movements are multiplied by this to account for temperature
    return 1 + params['temperature'] / 200

def humidity_evap_multiplier():
    # Evaporation is inversely proportional to humidity of greenhouse
    #  Zero at 100% humidity, 1 at 50%, 2 at 0%
    return 2 * (1 - params['humidity'] / 100)

# TODO This should be based on plants size and health but, as a proxy,
#   make it relative to length that the plants have been growing
max_plant_area = 25 # cm^2
def estimated_plant_area():
    dt = (params['time'] - params['start'])
    return max_plant_area*dt/(14*86400.0) # Max out after 14 days

def transpiration_rate():
    #The rate at which water moves soil->air due to plants
    return (0.025/3600)*estimated_plant_area()
    
def soil_evaporation_rate():
    #The rate at which water moves soil->air due to direct evaporation
    #average about 1 ml/hr, based on soil water content
    base = (evap_rate / 3600) * (params['soilwater'] / max_soilwater)
    if params['fan']:
        base *= 3 #Fan increases evaporation speed
    return base * temp_evap_multiplier() * humidity_evap_multiplier()

def tank_evaporation_rate():
    #The rate at which water moves tank->air due to direct evaporation
    base = evap_rate / 3600
    return base * temp_evap_multiplier() * humidity_evap_multiplier()

def temp_to_pressure(temp):
    return 6.11*10**((7.5*temp)/(237.7+temp))

def airwater_to_humid(airwater, temp):
    Es = temp_to_pressure(temp)
    E = (temp + 273.15)*461.5*(airwater/3933.0)
    return 100*E/Es

def humid_to_airwater(humid, temp): # In percent
    Es = temp_to_pressure(temp)
    E = Es*humid/100.0
    return (3933.0*E)/(461.5*(temp + 273.15))

# if humid > 100, then calculate the excess water that should precipitate out
def update_airwater_humid(airwater, temp):
    humid = airwater_to_humid(airwater, temp)
    if (humid <= 100): return (humid, airwater, 0)
    else:
        sat_water = humid_to_airwater(100, temp)
        return (100, sat_water, airwater - sat_water)
    
def exit_rate():
    # The rate at which water leaves the greenhouse via air.
    # Based on difference in humidity between greenhouse and room,
    #  is _much_ faster with the fan
    base = params['airwater'] * (3.0 if params['fan'] else 0.1)/3600
    return base * (params['humidity']/room_humidity - 1)
        
def get_cur():
    return (5.0 + led_current * params['led'] +
           (pump_current if params['wpump'] else 0) +
           (fan_current if params['fan'] else 0))
     
def get_weight():
    # Water weighs about 1 g per 1 ml, add some for plants
    return (base_weight + params['soilwater'] + params['panwater'] +
            + params['tankwater'] + estimated_plant_area())

# Environment Runtime Functions
 
def forward_water_cycle(duration):

    if duration > 1 and params['wpump']:
        duration = .5

    #Pump water if in reservoir and pump on, accounting for pipe lag
    # With the addition of the weight sensors, had to change how water movement
    #  works - now, goes from the pipe to the pan and then to the soil
    if params['wpump']:
        vol = min(duration * flow_rate, params['volume'])
        params['volume'] -= vol
        params['pipewater'] += vol
        if params['pipewater'] > pipe_capacity:
            params['panwater'] += params['pipewater'] - pipe_capacity
            params['pipewater'] = pipe_capacity
            
    elif params['pipewater'] > 0: #Could be if. should dripping be ok when the pump is on?
        vol = min(duration * drip_rate, params['pipewater'])
        params['pipewater'] -= vol
        params['panwater'] += vol
    
        
    #Do water movement:
    #from soil to tankwater by overflow if soil fully saturated
    total_water = params['soilwater'] + params['panwater']
    if total_water > max_soilwater:
       params['tankwater'] += total_water - max_soilwater
       params['panwater'] = max(0, max_soilwater - params['soilwater'])
       params['soilwater'] = max_soilwater - params['panwater']
    #from pan to soil
    if params['panwater'] > 0:
        vol = min(duration * uptake_rate, params['panwater'])
        params['panwater'] -= vol
        params['soilwater'] += vol

    #from soil to air because of plants
    aw = params['airwater']
    vol = min(duration * transpiration_rate(), params['soilwater'])
    params['soilwater'] -= vol
    params['airwater'] += vol
    #from soil to air by evaporation
    vol = min(duration * soil_evaporation_rate(), params['soilwater'])
    params['soilwater'] -= vol
    params['airwater'] += vol
    #from tankwater to air by evaporation
    vol = min(duration * tank_evaporation_rate(), params['tankwater'])
    params['tankwater'] -= vol
    params['airwater'] += vol
    #from air to outside the greenhouse
    vol = duration * exit_rate()
    params['airwater'] = min(max(0.0, params['airwater'] - vol), 100)
    humid, params['airwater'], excess_water = \
           update_airwater_humid(params['airwater'], params['temperature'])
    params['humidity'] = humid
    params['tankwater'] += excess_water

    return duration
    
def forward_temperature(duration):
    
    #Cooling due to the fan
    if params['fan']:
        params['temperature'] -= duration * fan_cool_rate
    #Heating due to light
    params['temperature'] += duration * light_heat_rate()
    #Change to equilibriate with outside env TODO if duration is big, newtons law of cooling may get out of hand
    params['temperature'] += duration * temp_equil_rate()
    params['temperature'] = max(params['temperature'], room_temp)


def forward_time(duration): #Here is where most of the env mutation takes place
    #to change volume, smoist, humidity, tankwater
    duration = forward_water_cycle(duration)
    #actually move time
    params['time'] += duration #TODO I think weather will be handled here
    
    #Then also a growplants function (which probably just calls grow on each plant in params[plants])
    
    #Change temp based on fan, light, ..? (+ for high light, - for fan on)
    forward_temperature(duration)
    #add to used energy
    params['energy'] += get_cur() * 12 / 1000 * duration
    #print(light_average())
    #ordering here is interesting
    return duration

def init(bl):
    #print("initializing with " + str(bl.params))
    if bl is not None:
        for k,v in bl.params.items():
            if k in params:
                params[k] = v
            elif k == 'humidity':
                params['humidity'] = v
            elif k == 'smoist':
                params['soilwater'] = v / 2
            elif k == 'wlevel':
                params['volume'] = v * volume_rate
    params['airwater'] = humid_to_airwater(params['humidity'], params['temperature'])
