BASELINE = high_humid_baseline.bsl

# Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

#when the temperature is high, make sure the fan turns on for at least a minute
WHENEVER humidity[0] > 80 or humidity[1] > 80
  WAIT fan FOR 1800
  WAIT not fan FOR 1800
  WAIT humidity[0] < 87 FOR 1800

QUIT AT 2-23:59:59
