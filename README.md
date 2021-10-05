# franka_pivot_control_ros
This is a ROS-Package to integrate franka_pivot_control into ROS.
Herefore the messages from pivot_control_messages_ros are used.

For DOF-Pose in this stage of the pivoting controller project 
we refer to the euler angle of the tool at the pivot point (entry hole) 
relative to the initial pose the tool started from.

## Starting
Using the default.launch-file.
A graphics surface can be started.

## ROS-Node
The following topics are by default remapped. This is to be refactored.
- franka_pivot_control subscribes the topics
  - `target_dof_pose`:
  the DOF-Pose (Pitch, Yaw, Roll, Trans-z) the tool/laparoscope is supposed to move to
  - `dof_boundaries`:
  the limits a DOF-Pose may have.
- franka_pivot_control publishes the topics
  - `current_dof_pose`:
  the calculated DOF-Pose the tool is currently in.
  - `pivot_error`:
  the calculated Distance from the tool(z)-axis to the pivot point
  - `franka_error`:
  error messages from libfranka
