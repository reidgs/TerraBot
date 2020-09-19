 # Wait a bit before starting, to give agent a chance to initialize
DELAY FOR 600

# Create an environment that is cold and dry
BASELINE = cold_and_dry.bsl

#Make sure you aren't overwatering
WHENEVER wpump
  ENSURE smoist[0] < 601 

#Make sure the pump is activated based on the schedule
WHENEVER smoist[0] < 500
  WAIT wpump FOR 18000


#make sure the pump doesn't turn on if the moisture level is high
WHENEVER smoist[0] > 650
  WAIT not wpump FOR 360

