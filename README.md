# **skydrop_drone**

## **Introduction**
SkyDrop is an automated UAV delivery system designed for a specific apartment building. This system allows parcels to be delivered from the apartment gate/security room to the destination apartment balcony by a UAV. </br>
The complete system consist of an autonomous UAV system and a dashboard to send commands to the UAV. This repository contains the implementation related to the autonomous UAV system. </br>

**Technologies:** Gazebo Simulator | Ardupilot | ROS | Dronekit Python | C++

<!-- Brunner et al. -->
This work is based on the work done by Brunner et al., 2019[^1][^2]. In our project, we are trying to improve their work further more by considering the suggestions mentioned by them in their research paper along with our own ideas.</br>

[^1]: **Research Paper of Brunner et al. work:** [Link to the research paper](https://arxiv.org/pdf/1809.08022.pdf)</br>
Brunner, G., Szebedy, B., Tanner, S. and Wattenhofer, R., 2019, June. The urban last mile problem: Autonomous drone delivery to your balcony. In 2019 international conference on unmanned aircraft systems (icuas) (pp. 1005-1012). IEEE.

[^2]: **GitHub Repository of Brunner et al. work:** [Link to the repository](https://github.com/szebedy/autonomous-drone) 

## **Setting up the local environment**
The following tools and softwares should be installed in the local machine.
- Operating System: Ubuntu 18.04 LTS Bionic Beaver
- ROS Melodic Morenia (Desktop-Full Install) ([ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu))
- Gazebo 9.0 (Comes as the default Gazebo version when installing ROS Melodic Morenia)
- Ardupilot & SITL ([Ardupilot installation instructions](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux) | [Setting up SITL on Linux](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html))
- MAVROS ([MAVROS installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation))
- QGroundControl ([QGroundControl installation instructions](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html))
- Dronekit-python library ([Dronekit installation instructions](https://dronekit-python.readthedocs.io/en/latest/guide/quick_start.html#installation))

## **Learning materials for technologies**
- ROS: [ROS Wiki Tutorials](http://wiki.ros.org/ROS/Tutorials)
- Gazebo: [Gazebo Tutorials](https://gazebosim.org/tutorials)
- Dronekit: [Dronekit-python Documentation](https://dronekit-python.readthedocs.io/en/latest/)

## **Instructions to clone the project to local machine**
**Step 01:** Create a catkin workspace. (We can skip this step as we already created a workspace when we installed MAVROS).</br>
For more delatis visit [ROS wiki: create a ROS workspace using ```catkin_make```](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#:~:text=you%20installed%20ROS.-,Create%20a%20ROS%20Workspace,-catkin). </br>
Here we use ```catkin build``` instead of ```catkin_make```.
```
cd mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Step 02:** Clone the project inside the ```src``` directory of the catikin workspace.
```
cd ~/catkin_ws/src
git clone git@github.com:SkydropDroneDeliveries/skydrop_drone.git
```
**Step 03:** Rename the directory from skydrop_drone to skydrop.
```
mv skydrop_drone skydrop
```
**Step 04:** Set model path for Gazebo models.
```
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/skydrop/models" >> ~/.bashrc
```
**Step 05:** Build the project.
```
cd ~/catkin_ws
catkin build
```
**Step 06:** Update the global variables.
```
source ~/.bashrc
```

## **Instructions to run the project**
We need multiple terminals to run the project.</br>

**Terminal 01:** Launch the Gazebo world
```
roslaunch skydrop <requiredWorldsLaunchFileName>.launch 
``` 
Replace the \<requiredWorldsLaunchFileName\> with the launch file name of the world that you wish to open in Gazebo.
<!-- Once we are done with the project we can mention the final launch file name in the command -->

**Terminal 02:** Launch the Ardupilot SITL
```
roscd skydrop
cd scripts
./startsitl.sh
```

**Terminal 03:** Start MAVROS to get telemetry data from the Flight Control Unit (FCU). </br>
NOTE: Wait until ```startsitl.sh``` script completes its work before starting the MAVROS. 
```
roslaunch skydrop apm.launch
```

<!-- Terminal 04 - Run the script with the complete logic -->
<!-- Once the complete logic is integrated to a single script, add how to run the script here in the markdown -->

<!-- Add anything else as required -->