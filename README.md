# Getting Started

## Install ROS and Catkin

[Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages:

```bash
rosdep update
sudo apt-get update
```

Install [catkin](http://wiki.ros.org/catkin) the ROS build system:

```bash
sudo apt-get install -y ros-melodic-catkin python-catkin-tools
```

## Install MoveIt

```bash
sudo apt install -y ros-melodic-moveit
sudo apt install -y ros-melodic-moveit-*
```

## Create A Catkin Workspace

You will need to have a [catkin](http://wiki.ros.org/catkin) workspace setup:

```bash
mkdir -p ~/catkin_ws/src
```

## Download Simulation

Within your [catkin](http://wiki.ros.org/catkin) workspace, download the `panda_moveit_config` package:

```bash
cd ~/catkin_ws/src
git clone https://github.com/kamlesh364/panda_moveit_config.git
git clone https://github.com/kamlesh364/flywheel_pick_and_place.git
```

**Note:** For now we will use a pre-generated `panda_moveit_config` package but later we will learn how to make our own in the [MoveIt Setup Assistant tutorial](../setup_assistant/setup_assistant_tutorial.html).

## Build your Catkin Workspace

The following will install from Debian any package dependencies not already in your workspace:

```bash
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
```

The next command will configure your catkin workspace:

```bash
cd ~/catkin_ws
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Source the catkin workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

Optional: add the previous command to your `.bashrc`:

```bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

**Note:** Sourcing the `setup.bash` automatically in your `~/.bashrc` is
   not required and often skipped by advanced users who use more than one
   catkin workspace at a time, but we recommend it for simplicity.

# Pick and Place Demo

In MoveIt, grasping is done using the MoveGroup interface. In order to grasp an object we need to create `moveit_msgs::Grasp` msg which will allow defining the various poses and postures involved in a grasping operation.
Watch this video to see the output of this tutorial:

<div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
    <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</div>

## Running The Demo

Open two terminals. In the first terminal start RViz and wait for everything to finish loading:

```bash
roslaunch flywheen_pick_and_place demo_gazebo.launch
```

In the second terminal run the pick and place tutorial:

```bash
rosrun flywheen_pick_and_place pick_place_demo
```

You should see something similar to the video at the beginning of this tutorial.

## The Entire Code

The entire code can be seen [here](./src/pick_place_demo.cpp) in the flywheen_pick_and_place GitHub project.