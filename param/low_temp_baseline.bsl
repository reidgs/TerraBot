#These are the default values if not specified

#start = 0              # [0, inf) seconds 
start = 1-00:00:00    # Starting at 4am, light behavior should be off

temperature = 20      # too cold, should turn LED on to heat up
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
