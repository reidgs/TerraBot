# Simple trace file
BASELINE = baseline.txt

WHENEVER smoist[0] < 450 or smoist[1] < 450
  WAIT wpump FOR 60 # Wait one minute for water pump to be on
  WAIT not wpump FOR 360 # Turn pump off before 6 minutes have elapsed
  # Wait an hour for both moisture sensors to be above threshold
  WAIT smoist[0] > 450 and smoist[1] > 450 FOR 3600

# Don't let the pump overwater things
WHENEVER wpump
  ENSURE smoist[0] < 600 and smoist[1] < 600 FOR 3600
