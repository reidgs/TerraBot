 # Wait a minute before starting, to give agent a chance to initialize
DELAY FOR 60

WHENEVER True
  # Wait 6 minutes for a ping
  WAIT ping FOR 360

# Reset the ping and wait until the next one
WHENEVER ping
  WAIT not ping FOR 10 # Don't want the current ping to confuse things
  ENSURE not ping FOR 120  # This should succeed
#  ENSURE not ping FOR 340 # This should (sometimes) fail

QUIT AFTER 3600 # Run the test and simulator for 1 hour
