import sys
import numpy as np

import moveit_commander
import rospy


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    

    rospy.init_node("joint_planner")
    
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    
    joint_goal = [0, np.deg2rad(90), 0, np.deg2rad(10), 0, np.deg2rad(0)]
    move_group.set_joint_value_target(joint_goal)
    
    move_group.go(wait=True)
    
    move_group.stop()
    
if __name__ == "__main__":
    main()