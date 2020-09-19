# Simple trace file
BASELINE = baseline1.bsl


WHENEVER smoist[0] < 450 or smoist[1] < 450
  WAIT wpump FOR 86400 # Make sure the water pump was turned on today

# Don't let the pump overwater things
WHENEVER wpump
  ENSURE smoist[0] < 600 and smoist[1] < 600 FOR 3600

WHENEVER temperature[0] > 30
  WAIT fan FOR 1800 # Wait one day for fan to turn on

WHENEVER humidity > 80
  WAIT fan FOR 1800 

WHENEVER 1-22:00:00
  # Wait for 6 minutes for lights to go off after 10pm each day
  WAIT not led FOR 360
  # Ensure lights stay off until just before 7am the next day
  ENSURE not led UNTIL 2-03:59:59
