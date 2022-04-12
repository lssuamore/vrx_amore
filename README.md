# vrx_amore Repository introduction
This repository holds all of the codes and necessary packages for Team AMORE's VRX solution. The setup of the code uses one ROS .launch file to launch the geonav_transform package, and 5 .cpp executables. The geonav_transform package is used for converting ECEF latitude longitude coordinates to a local NED fixed frame to work out of. The 5 .cpp executables include mission_control, path_planner, navigation_array, perception_array, and propulsion_system. The mission_control monitors the state of the current VRX task and sets the states of the other executables. path_planner was designed to handle computing trajectories and feeding them to the propulsion_system. navigation_array publishes ECEF states to nav_odom so that geonav_transform is able to subscribe and publish its conversion to a local ENU frame to "geonav_odom". navigation_array subscribes to all the VRX task topics because the units of objects and poses given by VRX are in ECEF units and need conversion to the local NED working frame. All conversions happen in navigation_array because conversions have to be done by publishing to the "nav_odom" node and this shouldn't be done from multiple executables unless perhaps different namespaces are used. perception_array uses OpenCV to do image processing through the use of two singular lense cameras placed on the USV payload tray. propulsion_system handles outputting the angles and thrusts to the motors. Currently, the propulsion_system is only using a station-keeping controller designed for a dual-azimuthing controller. 

username: 
```
lssuamore
```
password:
```
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
```
## Installation Process: follow the following sequence of commands
```
git clone https://github.com/lssuamore/vrx_amore.git
```
```
cd vrx_amore/
```
Initialize and update submodules. If submodule commands don't work, delete empty folders and git clone the correct packages from github in their place. CURRENTLY THERE ARE NO SUBMODULES, A .gitsubmodules
```
git submodule init
```
```
git submodule update
```
The submodule commands should install opencv and geonav_transform. Might also need:
```
git clone PCL: https://github.com/ros-perception/perception_pcl.git 
```
Delete the devel and build folders
```
catkin_make
```

## Update local repository with vrx_amore master
```
cd vrx_amore/
```
Make sure you are on the master branch with the following:
```
git checkout master
```
Pull the repository to your computer with the following:
```
git pull
```
username:
```
lssuamore
```
password:
```
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
```

# Git Commands
git checkout is how you will switch branches. The -b indicates we are creating a new branch.
```
git checkout -b <your name>
```
The following command adds all changes to be tracked that you changed. The -A parameter is what specificlly sets everything. You can also use git rm and other similar commands. As always, see the git --help command for specifics past the tutorial.
```
git add -A
```
The commit command does what it sounds like. You are commiting to the code as it should have made some change. This can be as simple as finishing up for the night though, it doesn't need to be cohesive. This is not pushing any code github yet, this is just locally saved on your computer.
```
git commit -m "Your message here"
```
The push command sends your code to the github servers. origin just indicates that it is going where we got it from, no real need to work with this in what we develop with. Last you have to make sure you choose your branch to send it too. Again in this case the branch is your name, but if you ever forget what it is called, you can always use the git status or git branch command.
```
git push origin <your name>
```
