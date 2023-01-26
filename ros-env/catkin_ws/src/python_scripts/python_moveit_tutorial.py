import sys
import numpy as np
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    
    def dist(p, q):
        return sqrt(
            sum(
                (p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)
            )
        )

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    display_trajectory_publisher = rospy.Publisher(
        "move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    
    planning_frame = move_group.get_planning_frame()
    print(f"=================== Planning frame: {planning_frame}")
    
    eef_link = move_group.get_end_effector_link()
    print(f"=================== End effector link: {eef_link}")

    group_names = robot.get_group_names()
    print(f"=================== Available Planning Groups: {robot.get_group_names()}")
    
    print(f"=================== Printing robot state")
    print(robot.get_current_state())
    print("")
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)
    

    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    scale = 1.0
    waypoints = []
    
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1
    wpose.position.y += scale * 0.2
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.x += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.y -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    

if __name__ == "__main__":
    main()