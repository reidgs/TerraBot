#NOTE: there is some 'buffer space' due to speedup, I ran the test at -s 350.
# Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

BASELINE = baseline2.bsl


#test 3 rounds of raiseTemp to see if it's correctly turning on LED and taking priority

#based on scheduled time, raiseTemp is checked every 3 hours (4h diff between each start),
#hence these WHENEVER values below.

WHENEVER 1-01:00:00
  SET raiseTemp = 1

WHENEVER 1-05:00:00
  SET raiseTemp = 1

WHENEVER 1-09:00:00
  SET raiseTemp = 1

#whenever it's the scheduled time for raiseTemp, ensure it is on for the entire hour
WHENEVER raiseTemp
  ENSURE led FOR 3300 #entire hour is 3600, we use 3300 for some 'buffer' time due to speedup at 350
  SET raiseTemp = 0

QUIT AT 1-10:30:30 #quit after 3 rounds are over
