import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test_move_group', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()



group_name = "puddles_base"
move_group = moveit_commander.MoveGroupCommander(group_name)

print("{}".format(move_group.get_planning_frame()))

joint_goal = move_group.get_current_joint_values()

joint_goal = [0,0,-4,0,0,0,1]

move_group.set_planning_time(1)
move_group.set_workspace([-50,-50,-30,50,50,0])
print(joint_goal)
plan = move_group.plan(joint_goal)

print(plan)
move_group.execute(plan,wait=True)

move_group.stop()

