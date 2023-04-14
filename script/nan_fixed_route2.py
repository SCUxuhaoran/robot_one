#!/usr/bin/env python
# encoding: utf-8

 
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
 
# 巡逻点
list_pointx = [0.53, 1.15, 1.52, 1.80, 2.03, 2.50, 3.00, 3.60, 4.00, 3.65, 3.20, 2.80, 2.52, 1.92, 1.69, 1.05, 0.30, 0.00, 0.13 ]
list_pointy = [-0.1,-0.07,-0.60,-1.41,-2.59,-3.00,-3.00,-2.60,-1.70,-0.62,-0.10,-0.16,-0.60,-1.70,-2.74,-3.23,-3.01,-1.80,-0.76 ]
waypoints=[
    [( 3.00, -1.40, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.90, -0.50, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.00,  0.50, 0.0),( 0.0, 0.0, 180.0)],
    [( 2.10, -0.50, 0.0),( 0.0, 0.0, 180.0)]
]
def goal_pose(pose):
    goal_pose=MoveBaseGoal()
    goal_pose.target_pose.header.frame_id="map"
    goal_pose.target_pose.pose.position.x=pose[0][0]
    goal_pose.target_pose.pose.position.y=pose[0][1]
    goal_pose.target_pose.pose.position.z=pose[0][2]
 
    # r, p, y  欧拉角转四元数
    x,y,z,w=tf.transformations.quaternion_from_euler(pose[1][0],pose[1][1],pose[1][2])
 
    goal_pose.target_pose.pose.orientation.x=x
    goal_pose.target_pose.pose.orientation.y=y
    goal_pose.target_pose.pose.orientation.z=z
    goal_pose.target_pose.pose.orientation.w=w
    return goal_pose
 
 
if __name__ == "__main__": 
    #节点初始化
    rospy.init_node('patrol')
 
    #创建MoveBaseAction client
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #等待MoveBaseAction server启动
    client.wait_for_server()
 
    while not rospy.is_shutdown():
        for pose in waypoints:
            goal=goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
