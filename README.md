# vrx_amore
This repository holds all of the codes and packages for Team AMORE's VRX solution.
## Installation Process:

```
cd ~
```
```
git clone https://github.com/lssuamore/RobotX2022.git
```
username:
```
lssuamore
```
password:
```
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
```
```
cd RobotX2022/
```
Initialize and update submodules. If submodule commands don't work, delete empty folders and git clone the correct packages from github in their place.
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
