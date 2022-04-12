# vrx_amore Repository introduction
This repository holds all of the codes and necessary packages for Team AMORE's VRX solution. The setup of the code uses one ROS .launch file to launch the geonav_transform package, and 5 .cpp executables. The geonav_transform package is used for converting ECEF latitude longitude coordinates to a local NED fixed frame to work out of. The 5 .cpp executables include mission_control, path_planner, navigation_array, perception_array, and propulsion_system. The mission_control monitors the state of the current VRX task and sets the states of the other executables. path_planner was designed to handle computing trajectories and feeding them to the propulsion_system. navigation_array publishes ECEF states to nav_odom so that geonav_transform is able to subscribe and publish its conversion to a local ENU frame to "geonav_odom". navigation_array subscribes to all the VRX task topics because the units of objects and poses given by VRX are in ECEF units and need conversion to the local NED working frame. All conversions happen in navigation_array because conversions have to be done by publishing to the "nav_odom" node and this shouldn't be done from multiple executables unless perhaps different namespaces are used. perception_array uses OpenCV to do image processing through the use of two singular lense cameras placed on the USV payload tray. propulsion_system handles outputting the angles and thrusts to the motors. Currently, the propulsion_system is only using a station-keeping controller designed for a dual-azimuthing controller. 

For permission purposes when installing, pulling, and pushing code to this repository:

username: ```lssuamore```

password: ```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```

## Install Git Repository to Local Machine
Step 1: ```git clone``` the vrx_amore repository
```
git clone https://github.com/lssuamore/vrx_amore.git
```
Step 2: change directory to vrx_amore
```
cd vrx_amore/
```
Initialize and update submodules. If submodule commands don't work, delete empty folders and git clone the correct packages from github in their place. CURRENTLY, THERE ARE NO SUBMODULES AND THE NEXT TWO COMMANDS CAN BE SKIPPED.
```
git submodule init
```
```
git submodule update
```
Step 3: Delete the devel and build folders from the vrx_amore repository downloaded to local machine through ```git clone```
```
catkin_make
```

## Update Local Machine with Git Repository
Step 1: Change local directory to vrx_amore.
```
cd vrx_amore/
```
Step 2: Make sure you are on the master branch to ensure you pull the master code.
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
