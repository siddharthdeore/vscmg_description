# VSCMG Description

Variable Speed Control Moment Gyroscope ros package. Complete thesis report available at [VSCMGThesisReport](https://github.com/siddharthdeore/VSCMGThesisReport).

# Build
Navigate to catkin workspace root directory and make.


```sh
cd catkin_ws
catkin_make
```

source workspace
```sh
source devel/setup.bash
```
run dummy joint state publisher
```sh
roslaunch vscmg_description dummy.launch
```

using adcs lib for physics
```sh
roslaunch vscmg_description run.launch
```
