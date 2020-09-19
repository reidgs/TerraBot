#These are the default values if not specified

start = 0              # [0, inf) seconds 
#start = 1-00:00:00    # day-hour:min:sec also works, day=1 is the first day

temperature = 35 # starting too hot, should turn fan on
humidity = 90 # starting at too humid, should turn fan on
smoist = 300 # starting too dry, should turn water pump on
wlevel = 140
tankwater = 100

wpump = off
fan = off
led = 0


leaf_droop = 0          # [0, 1]
lankiness = 0           # [0, 1] (determines the lankiness of the plant) (only really matters if time is large enough) 
plant_health = 1        # [0, 1] (does not affect growth prior to simulation start)

