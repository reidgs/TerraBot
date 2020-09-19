# Testing that no behaviors make any changes to the actuators from 0-4am
BASELINE = low_temp_baseline.bsl

DELAY FOR 75

WHENEVER True
    ENSURE not led UNTIL 1-04:00:00 # no behaviors that turn the LED on should run until 4am

QUIT AT 1-04:00:00 # not sure how to get it to register "SUCCESS" but it stops execution without a "FAILURE" so I'm counting that as a success

