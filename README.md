# Install ROS #

## First Create the Workspace ##
mkdir -p robotonist_ws/src
"catkin_make" in that directory (the parent not the src)
will make three seperate folders "build, devel, src"
"source devel/setup.bash" source the setup file
to make sure everything is working run "echo $ROS_PACKAGE_PATH"
Should return 
 /home/<usr>/<path>/robotonist_ws/src:/opt/ros/kinetic/share_
## Now we must create a catkin Package ##
To do this we must determine exactly what dependancies
catkin_create_pkg <package_name> [depend1] [depend2] ...
Now go back to the ws folder and rerun catkin_make
". devel/setup.bash" to source the setup file
Use rospack depends to determine all dependancies
## Now we create the script ##
create a publisher and a subscriber 
## Now we run the script ##
Make sure roscore is up and running
source the setup.bash file "source ./devel/setup.bash"
rosrun <package name> <file.py>
