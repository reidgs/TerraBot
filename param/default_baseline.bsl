#These are the default values if not specified

start = 0              # [0, inf) seconds 
#start = 1-00:00:00    # day-hour:min:sec also works, day=1 is the first day

temperature = 20        
humidity = 50
smoist = 800
wlevel = 140
tankwater = 0

wpump = off
fan = off
led = 0


leaf_droop = 0          # [0, 1]
lankiness = 0           # [0, 1] (determines the lankiness of the plant) (only really matters if time is large enough) 
plant_health = 1        # [0, 1] (does not affect growth prior to simulation start)
