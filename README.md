# Control Kuka manipulator with robonomics.

## Liabiliy

```
chmod +x liability/msg.js
```

## Requirements
* ROS melodic, Gazebo (installation instraction [here](http://wiki.ros.org/melodic/Installation/Ubuntu))
* Some extra packages
```bash
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller
```
* IPFS 0.4.22 (download from [here](https://www.npackd.org/p/ipfs/0.4.22) and install)
```bash
tar -xvzf go-ipfs_v0.4.22_linux-386.tar.gz
cd go-ipfs/
sudo bash install.sh
ipfs init
```
* ipfshttpclient
```bash
pip3 install ipfshttpclient
```
* substrate-interface
```bash
pip3 install substrate-interface
```
* Robonomics node (binary file) (download latest release [here](https://github.com/airalab/robonomics/releases))
* IPFS browser extension (not necessary)
***
## Installation
Install Kuka manipulator and control packages
```bash
cd catkin_wc/src/
git clone https://github.com/orsalmon/kuka_manipulator_gazebo
git clone https://github.com/LoSk-p/kuka_controller
cd ..
catkin_make
```
***
## Running gazebo model
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch manipulator_gazebo manipulator_empty_world.launch
```
In a new window
```bash
source ~/catkin_ws/devel/setup.bash
rosrun manipulator_gazebo move_arm_server
```
![model](media/1.png)
***

## Running ipfs
Run ipfs daemon:
```bash
ipfs daemon
```
***


Now you can run control script:
```bash
source ~/catkin_ws/devel/setup.bash
rosrun kuka_controller move_arm_client.py
```


In the window with kuka_control package you will see:

![done](media/res.png)

Now you can find robot's telemetry in IPFS via this link with your hash `https://gateway.ipfs.io/ipfs/<hash>`.





