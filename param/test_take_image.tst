#BASELINE= # Use a baseline file to start right before an image is to be taken

WHENEVER None != camera # Changed the order to distinguish this from above
  ENSURE light[0] > 20 or light[1] > 20 FOR 1 # Test right away

WHENEVER camera != None
  SET image = camera # Do this b/c 'camera' does not stay latched
  WAIT os.path.exists(image) FOR 30
  # Count the number of pictures taken
  SET num_pics = num_pics + 1

# Ensure sure that 1-3 pictures get taken every day
WHENEVER 1-00:00:00 # Every midnight
  SET daily_pics = num_pics
  WAIT UNTIL 1-23:59:59
  SET dpic = num_pics - daily_pics
  ENSURE 1 <= dpic and dpic <= 3
