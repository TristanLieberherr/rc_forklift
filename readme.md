# How to install

1. Clone the repo in your ROS workspace (http://wiki.ros.org/catkin/Tutorials/using_a_workspace)
2. Source your workspace
3. Install dependencies
  ```bash
  rosdep update
  rosdep install --from-paths ~/forklift_ws/src --ignore-src --rosdistro=${ROS_DISTRO}
  ```
4. Set the ROS_MASTER_URI variable to the master's hostname **on both computers**
  ```bash
  export ROS_MASTER_URI=http://{hostname}:11311 # {hostname} is the hostname of the master
  ```
5. Launch roscore on the master
  ```bash
  roscore
  ```
6. Launch the server nodes on the forklift
  ```bash
  roslaunch rc_forklift server.launch
  ```
7. Launch the client nodes on the master
  ```bash
  roslaunch rc_forklift client.launch
  ```
