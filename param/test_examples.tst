 # Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60
#DELAY UNTIL 1-07:00:00 # Wait until 7am before starting to test

BASELINE = smoist_up.bsl
#INTERFERENCE = smoist_up.inf

WHENEVER smoist[0] < 450
  WAIT wpump FOR 60 # Wait one minute for water pump to be on
  WAIT not wpump FOR 360 # Turn pump off before 6 minutes have elapsed
  # Wait an hour for both moisture sensors to be above threshold
  WAIT smoist[0] > 450 and smoist[1] > 450 FOR 3600

WHENEVER 1-22:00:00
  # Wait for 6 minutes for lights to go off after 10pm each day
  WAIT not led FOR 360
  # Ensure lights stay off until just before 7am the next day
  ENSURE not led UNTIL 2-06:59:59

# Count the number of pictures taken
WHENEVER camera
  SET num_pics = num_pics + 1
# Make sure that 3-5 pictures get taken every day
WHENEVER 1-00:00:00 # Every midnight
  SET daily_pics = num_pics
  WAIT UNTIL 1-23:59:59
  SET dpic = num_pics - daily_pics
  ENSURE 3 <= dpic and dpic <= 5

# Make sure the light level is sufficient whenever taking a picture
WHENEVER camera
  ENSURE light[0] > 20 and light[1] > 20

#STOP AT 5-00:00:00 # Stop testing midnight of the 5th day
#STOP AFTER 36000 # Stop testing after 10 hours
QUIT AT 3-23:59:59 # Run the test and simulator for 3 days
