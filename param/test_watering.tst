#NOTE: if testing this on a high speedup, set the window size in greenhouse_behavior to 3. This is to ensure that you are not using super old measurements when calculating water level.  

# Wait a bit before starting, to give agent a chance to initialize
DELAY FOR 60

# Create an environment that is dry
BASELINE = baseline_water_1.bsl

WHENEVER 1-00:00:00
  SET wpump_today_saved = wpump_today
  SET wpump_today = 0
  ENSURE wpump_today_saved < 5.7 # cms, 1.9 per watering times 3

#count how many times the water pump turns on
WHENEVER wpump
  SET wlevel_start = wlevel
  #PRINT "W1: %s %s" %(wlevel_start, wpump_today)
  WAIT not wpump FOR 15
  ENSURE not wpump FOR 10
  SET wpump_today = wpump_today + (wlevel_start - wlevel)
  PRINT "W1: %s %s %s" %(wlevel, (wlevel_start - wlevel), wpump_today)
  ENSURE not wpump FOR 120

#make sure it doesn't turn on too many times in a day
#set to 5 because speedup might cause it to water a litte more than necessary
WHENEVER wpump
  ENSURE smoist[0] < 600 FOR 60

#if the wpump hasn't been turned on that day and the soil is dry
WHENEVER smoist[0] < 480 and wpump_today<1 and not wpump
  WAIT wpump FOR 28800

QUIT AT 7-23:59:59 # Run the test and simulator for 7 days
