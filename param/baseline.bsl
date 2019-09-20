# Clock starts at 3am, first day; set initial sensor readings
START AT 1-03:00:00
light = [10, 100]
temperature = [20, 20]
humidity = [40, 50]
smoist = [350, 350]
wlevel = 130.0 # Nearly full reservoir

# Set humidity high, 4:30am, first day
AT 1-04:30:00
humidity = [80,80]

# Set lights low, 9am, first day
AT 1-09:00:00
light = [0, 0]
wlevel = 80.0 # Half full reservoir

# Set temperature high, 5:15pm, first day
AT 1-17:15:00
temperature = [31, 31]

# Set humidity low, 12:30am, second day
AT 2-00:30:00
humidity = [20,20]
