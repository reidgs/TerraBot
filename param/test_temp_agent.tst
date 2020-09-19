BASELINE = high_temp_baseline.bsl

# Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

#when the temperature is high, make sure the fan turns on for at least a minute
WHENEVER temperature[0] > 29 or temperature[1] > 29
  WAIT temperature[0] < 29 FOR 14400

QUIT AT 2-23:59:59
