 # Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

#WHENEVER True
  # Wait 3 minutes for a ping
#  WAIT ping FOR 180

# Reset the ping and wait until the next one
WHENEVER ping
  WAIT not ping FOR 30
  ENSURE not ping FOR 100
  WAIT ping FOR 70
#  WAIT ping FOR 100  # This should succeed
#  ENSURE not ping FOR 340 # This should (sometimes) fail

#QUIT AFTER 36000 # Run the test and simulator for 1 hour
