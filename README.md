# vrx_amore Introduction
This repository holds all of the codes and necessary packages for Team AMORE's Virtual RobotX (VRX) solution. This repo is designed to allow autonomous control of the Wave Adaptive Modular Vessel (WAM-V) unmanned surface vehicle (USV). This solution uses one ROS .launch file to launch the geonav_transform package, and 7 .cpp executables. The geonav_transform package is used for converting ECEF latitude longitude coordinates to a local ENU fixed frame that allows easy conversion to a NED frame to work out of, since that is the standard in the maritime industry. The 7 .cpp executables include mission_control, navigation_array, coordinate_converter, propulsion_system, path_planner, perception_array, and acoustics. The mission_control monitors the state of the current VRX task and sets the states of the other executables based on the overall system status. navigation_array publishes ECEF states to nav_odom so that geonav_transform is able to subscribe and publish its conversion to a local ENU frame to "geonav_odom". navigation_array then converts the ENU pose to a NED pose. coordinate_converter subscribes to all the VRX task topics that give important positions in ECEF and converts them to the local NED frame. Since conversions have to be done by publishing to the "nav_odom" node, this can't be done from multiple executables without close consideration to the priority of each executable, which is accomplished in the logic in mission_control. Perhaps different namespaces can be used for each executable to allow conversions of important positions of tasks and the USV pose simultaneously. propulsion_system handles outputting the angles and thrusts to the motors. Currently, the propulsion_system is using a PID heading position dual-azimuthing station-keeping controller. In the future, it is imagined to have a LL_state that tells which type of controller to use. Different options could use PID control theory to calculate control efforts for either heading position or heading speed and then use allocation to convert these efforts into thruster angles and thrust outputs dependent on the drive configuration. Possible drive configurations include differential, dual-azimuthing, and Ackermann. path_planner was designed to handle computing trajectories and feeding them to the propulsion_system. perception_array uses OpenCV to do image processing through the use of two singular lense cameras placed on the USV payload tray. acoustics interfaces with the acoustic hydrophones and performs the math necessary to estimate acoustic pinger positions with respect to the USV.

For permission purposes when installing, pulling, and pushing code to this repository, use the following username and password, unless you are a collaborator, then you can use your own github username and password.

USERNAME
```
lssuamore
```
PASSWORD
```
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
```

## Install Git Repository to Local Machine
NOTE: Beware that this repository was made to interface with the osrf/vrx git repository, which was made to work with ROS NOETIC. ROS NOETIC is made to be used with Ubuntu 20.04. Go to this link to follow tuturials for setiing up your host machine for working with the vrx simulation: https://github.com/osrf/vrx/wiki/tutorials. After setting up your machine with ROS NOETIC, Gazebo, and the vrx package, you are ready to install the vrx_amore solution.

Step 1: ```git clone``` the vrx_amore repository
```
git clone https://github.com/lssuamore/vrx_amore.git
```
Step 2: change directory to vrx_amore
```
cd vrx_amore/
```
Initialize and update submodules with the following two commands. If submodule commands don't work, delete empty folders and git clone the correct packages from github in their place.
```
git submodule init
```
```
git submodule update
```
Step 3: Delete the devel and build folders from the vrx_amore repository cloned to local machine through ```git clone``` if they came through while cloning.

Before catkin_make, update amore package CMakeLists.txt by commenting out all the programs (add_executable and target_link_libraries). This will allow the first catkin_make to build all the amore message types so that errors while catkin_makeing are avoided.
```
catkin_make
```
After catkin_make, update amore package CMakeLists.txt by uncommenting all the programs (add_executable and target_link_libraries). This will allow the next catkin_make to build all the programs then.
```
catkin_make
```
While using catkin_make, if the following error comes up:
```
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:75 (find_package):
  Could not find a package configuration file provided by ""
  with any of the following names:
```
Download the appropriate packages by finding the download on Google usually in the form:
```
sudo apt install ros-noetic-<>
```
Download the following libraries if necessary...
```
sudo apt-get install ros-noetic-xacro
```
```
sudo apt-get install ros-noetic-geographic-msgs
```
```
sudo apt-get install ros-noetic-cv-bridge
```
```
sudo apt-get install ros-noetic-image-transport
```
```
sudo apt-get install ros-noetic-pcl-ros
```
```
sudo apt-get install ros-noetic-tf2-geometry-msgs
```

## Update Local Machine with Git Repository
Step 1: Change local directory to vrx_amore.
```
cd vrx_amore/
```
Step 2: Make sure you are on the branch master using ```git status``` to ensure you pull the master code. If you are not on the branch master, run the following:
```
git checkout master
```
Step 3: Pull the repository to your local machine.
```
git pull
```

## Push Local Machine Code to Git Repository
Step 1: Use ```git checkout``` to switch to your own branch. The -b indicates we are creating a new branch.
```
git checkout -b <your name>
```
Step 2: Use ```git add``` to add changes to be tracked that you changed. The -A parameter specificlly sets everything.
```
git add -A
```
Step 3: Use ```git commit``` to commit the added changes. The -m allows the commit message to be created and attached to the commit.
```
git commit -m "Your message here"
```
Step 4: Use ```git push``` to push the most recent commit on the local branch <your name> to the origin, which is the URL to the vrx_amore repository on GitHub.
```
git push origin <your name>
```

For extra git commands, use ```git --help``` to list all the possible options.
