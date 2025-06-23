## Installing and Running TerraBot in Docker ##
- [Building a Docker Image](#Build_Docker)
- [Running a Docker Container](#Run_Docker)
- [Running TerraBot and Agent Software](#Run_TerraBot)
- [Visualizing the TerraBot's Graphics](#Visualization)

### Build Docker ###
Building the system using docker is very simple:
* Download docker desktop ([docs.docker.com/get-started/introduction/get-docker-desktop/](https://docs.docker.com/get-started/introduction/get-docker-desktop/) for your machine
* cd to the TerraBot/docker directory
* If you have access to `make`, simply invoke `make build` (or `make rebuild` if you want to rebuild from scratch)
* Otherwise, look at the `build` or `rebuild` instructions in the `Makefile` and run those commands in a shell

In either case, you will produce a terrabot_image file that can then be run (see next section).

### Run Docker ###
The docker image includes Ubuntu, ROS, the TerraBot software, and the packages you will need to run the TerraBot and all the assignments in the course.  To run a docker container:
* From a shell, if on Windows, invoke `run_terrabot.bat`; if on Linux or Macs, invoke `sudo ./run_terrabot.sh` with password `TerraBot`
* The shell scripts start a container called `terrabot_container`, exposing port 5901 (which is used for graphics visualization).  It also maps a directory on your computer to the `User` directory in the container.  By default, the directory mapped is the one in which you invoked the shell script.  However, you can change that by providing a path to the directory you want to view within the container.  For instance:
  * `run_terrabot.sh "c:\Users\rsimmons\Desktop\ROS_HW"`
  * (note: this is equivalent to cd'ing to the ROS_HW directory and invoking the script from there)
* If a container is already running, the shell scripts will connect you to the existing container
* You can also invoke the container from the Docker Desktop, but you will have to add the command-line arguments manually.

Once you invoke the container, you will end up in a `bash` shell under Ubuntu.  From there, you can run the TerraBot software (see next section).

### Run TerraBot ###
There are two directories within the container.  The `TerraBot` directory contains all the software needed to run the TerraBot simulator; The `User` directory is whichever directory you mapped to when you invoked the shell scripts (default being the directory in which the shell script was invoked).  Note that if you edit files in the `User` directory, it will be changed on your computer; if you edit files in the `TerraBot` directory, those changes will last only as long as the container is running.

To run the TerraBot, do `python TerraBot\TerraBot.py -m sim`.  You can also use the `-s <x>` flag to speed up the simulator by a factor of "x" (e.g., `-s 100` runs the simulator at 100x real speed), and the `-g` flag to run graphics (see next section for how to view the graphics).  You will see `Waiting for nodes`, `Starting simulator`, followed by `System started`.  The software is running and growing plants (but they will die without an autonomous agent to take care of them).

For most of the assignments, you will need multiple terminals to be running.  There are several ways to accomplish this:
1. Invoke the shell scripts multiple times; the first invocation starts a new container and subsequent invocations connect a new terminal to that container.
2. Use `tmux`, which is a terminal multiplexer.  Use `ctl-b "` to split screens and `ctl-b o` to move between them.  One good reason to use `tmux` with the docker container is that you'll need to use it on the Raspberry Pi, so it is good to get familiar with using it.
3. You can connect to the docker container through Visual Studio.  Open the `Extensions` tab and install `Dev Containers` (need to do this just once).  Then, open the `Remote Explorer` tab and select the `terrabot_image (terrabot_container)`. At that point, you can navigate to and execute files (using the `Folders` tab) or open a terminal.  Again, if you edit files in the `User` directory, they get changed on your computer; if you edit files in the `TerraBot` directory, they will go away once the container ends.
4. You can open new terminals using `xterm &`, but to be seen you need to follow the instructions in the next section.

### Visualization ###
The TerraBot container runs lightweight versions of X (a VNC server) with a window manager, exporting the display on port 5901. To view the display, you need to run a remote viewer.  There are several available options. TigerVNC ([https://tigervnc.org/](https://tigervnc.org/)) is available for Windows, Macs, and Linux, but other options include RealVNC, TightVNC, and Remmina (for Linux).  You will need to download and install one of the options.

When you invoke the viewer, connect to `localhost:5901` - you should then be able to see any graphics that have been started in the container.  In particular, when you run `TerraBot.py` with the `-m sim -g` options, it starts up a graphical simulator that shows all the dynamic change to the (simulated) greenhouse.  Similarly, the `agents/plot_sensors.py` and `agents/time_series.py` both display graphics.  And, as mentioned above, you can start xterm windows and they can be interacted with (e.g., editing and running code).


