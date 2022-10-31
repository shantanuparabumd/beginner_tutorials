# Programming Assignment: ROS Publisher/Subscriber
***
**Subject:** ENPM 808X  
**Name:** Shantanu Parab  
**UID:** 119347539  
**Directory ID:** sparab@umd.edu
***
### Instructions to Create workspace and build package
```bash
#Create a Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
#Clone Package
git clone git@github.com:shantanuparabumd/beginner_tutorials.git
# cd if you're still in the ``src`` directory with the ``beginner_tutorials`` clone
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
#Source your WS
. install/local_setup.bash
#Build Package
colcon build --packages-select beginner_tutorials
```

### Instruction to run nodes
Open a new terminal to run publisher node
```bash
cd ~/ros2_ws
. install/local_setup.bash
ros2 run beginner_tutorials talker
```
Open a new terminal to run subscriber node
```bash
cd ~/ros2_ws
. install/local_setup.bash
ros2 run beginner_tutorials listener
```