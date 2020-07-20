 # Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

WHENEVER 1-22:00:00
  # Wait for 6 minutes for lights to go off after 10pm each day
  WAIT not led FOR 360
  # Ensure lights stay off until just before 7am the next day
  ENSURE not led UNTIL 2-06:59:59

QUIT AT 3-23:59:59 # Run the test and simulator for 3 days
