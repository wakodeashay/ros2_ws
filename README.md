# Autonomous Navigation using Nav2 for tugbot robot

## Codemap
    .
    ├── README.md
    └── src
        ├── monitor_package
        │   ├── CMakeLists.txt
        │   ├── doc
        │   ├── include
        │   ├── launch
        │   ├── package.xml
        │   ├── script
        │   └── src
        ├── pointcloud_to_laserscan
        │   ├── CHANGELOG.rst
        │   ├── CMakeLists.txt
        │   ├── include
        │   ├── launch
        │   ├── LICENSE
        │   ├── package.xml
        │   ├── README.md
        │   └── src
        └── tugbot_ros2_pkgs
            ├── README.md
            ├── tugbot_description
            ├── tugbot_gazebo
            ├── tugbot_navigation2
            ├── tugbot_ros2_pkgs
            └── tugbot_slam

* monitor_package: Target tracking and gathering odom data for analysis. Setting initial pose and target location.
* pointcloud_to_laserspace (submodule): Used to convert pointcloud to laserscan. Humble branch of this [repo](https://github.com/ros-perception/pointcloud_to_laserscan).
* tugbot_ros2_pkgs (submodule): Hosts tugbot urdf, world, slam and navigation file. A fork of this [repo](https://github.com/porizou/tugbot_ros2_pkgs/tree/master).

## Machine used for testing:
* Processor: 13th Gen Intel® Core™ i7-13700H × 20
* RAM: 16 Gib
* OS Name: Ubuntu 22.04.5 LTS
* ROS Distro: Humble 
* Gazebo Distro: Harmonic

## Setup:
* Install ros2 humble using official guide + add ```/opt/ros/humble/setup.bash``` to .bashrc

* Init and update rosdep
    ```
    sudo rosdep init
    rosdep update
    ```

* Download and install nav2
    ```
    source /opt/ros/humble/setup.bash

    mkdir -p ~/nav2_ws/src && cd ~/nav2_ws

    git clone https://github.com/ros-navigation/navigation2.git --branch $ROS_DISTRO ./src/navigation2

    rosdep install -y \
    --from-paths ./src \
    --ignore-src

    colcon build \
    --symlink-install

    ```
Note: Certain package with dependency on gazebo classic may not build, but this error can be skipped.

* Install Gazebo harmonic
    ```
    sudo apt-get update
    sudo apt-get install curl lsb-release gnupg

    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install gz-harmonic

    sudo apt-get install ros-humble-ros-gzharmonic
    ```

* Install Cyclone DDS 
    ```
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    ```
and add following line to .bashrc.
    ```
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

* Git clone this repo iand initialize the submodules
    ```
    git clone git@github.com:wakodeashay/ros2_ws.git
    cd ros2_ws
    git submodule init
    git submodule update --init --recursive
    ```

* Build and source the build
    ```
    cd ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

### Running Default and Modified Planners
#### Setup (Assuming you already have a map)
* You will need 4 terminal windows.
* Terminal 1: Add command to launch navigation in the first using this command (for default planner).
    ```
    ros2 launch tugbot_navigation2 navigation2_default.launch.py 
    ```
    and following command for modified planner.
    ```
    ros2 launch tugbot_navigation2 navigation2_modified.launch.py 
    ```
* Terminal 2: Add command to launch monitor node
    ```
    ros2 launch tugbot_navigation2 navigation2_default.launch.py 
    ```
* Terminal 3: Add command to move actor (another tugbot) in the gazebo world
    ```
    gz topic -t "tugbot_actor/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.08}, angular: {z: 0.5}"
    ```
* Terminal 4: Add command to publish target
    ```
    ros2 topic pub --once /monitor_input_goal geometry_msgs/msg/Pose "{position: {x: -9.2, y: -6.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
    ```
#### Execute
* Running cmd in Terminal 1 will open rviz and gazebo window. Let them fully open and load.
* Run cmd in Terminal 2 and start gazebo simulation using play button placed left-bottom in gazebo window.
* The last command would set the initial state automatically.
* Start commands in Terminal 3 and 4 to start moving the actor and setting target/goal location respectively.

#### Videos (Screen Recording) of the simulation both the cases are uploaded [here](https://drive.google.com/drive/folders/1XH50mWInGyqgtR6C2Jc4bZj6_BHK3GXH?usp=sharing).

#### Approach
* Used existing tubot world + robot model for simulation. 
* Did many changes to the fork for it to work with Gazebo Harmonic. Changed topics +  added moving actors to name a few (Check commits for more details). 
* Created monitor_package with monitor_node to:
    * Set an initial pose and localize the robot in the world.
    * Send a navigation goal to a target location andprint "Reached" in the terminal. 
    * Prompt for the next goal, allowing the user to input another target location.
* Approach in planning
    * Increased loacl planner frequency to adapt to moving actors.
    * Rectified kinematic limits on tugbot.
    * Used Astar for planner server for smoother path.
    * Stopped publishing voxel layer to save computational efforts.

After plotting the log data from the 2 simulations, velocity plots were plotted and compared.
![aComparision Plot](/src/monitor_package/doc/comparison_plot.png). 
* Thruough the plots it was inferred that angular velocity evolution was better in modified planner, while linear velocity evolution remained more or less the same (Check variance values).
* Apart from that, the efficiency was found to increase, since the robot reached the target location faster in modified planner.

#### Challlenges faced
* Tried to tweak URDFs and world to remove unnecessary part to ease visualization, camera panning.
* Struggled with 2D pose estimate, but eventually realised how to do it.
* Tried putting non-zero bound on velocities to not allow robot from stopping completely, but resulted problems while reaching the target.


