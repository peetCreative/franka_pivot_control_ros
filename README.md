# franka_pivot_control_ros
This is a component in the research project [_EndoMersion_](https://github.com/peetcreative/endomersion).
As a ROS-Package, it integrates [franka_pivot_control](https://github.com/peetcreative/franka_pivot_control) into ROS.
Therefore the messages from [pivot_control_messages_ros](https://github.com/peetcreative/pivot_control_messages_ros) are used.

For DOF-Pose in this stage of the pivoting controller project 
we refer to the euler angle of the tool at the pivot point (entry hole) 
relative to the initial pose the tool started from.

## Starting
Using the default.launch-file a graphical UI can be started for testing.
However within _EndoMersion_ the pivot_control is meant to be controlled by any Node publishing target Poses.

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
