#!/usr/bin/env python
# encoding: utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
 
# 巡逻点
list_pointx = [0.53, 1.15, 1.52, 1.80, 2.03, 2.50, 3.00, 3.60, 4.00, 3.65, 3.20, 2.80, 2.52, 1.92, 1.69, 1.05, 0.30, 0.00, 0.13 ]
list_pointy = [-0.1,-0.07,-0.60,-1.41,-2.59,-3.00,-3.00,-2.60,-1.70,-0.62,-0.10,-0.16,-0.60,-1.70,-2.74,-3.23,-3.01,-1.80,-0.76 ]

waypoints1=[
    [( 1.00, -0.65, 0.0),( 0.0, 0.0, 0.0)],
    [( 3.00, -0.65, 0.0),( 0.0, 0.0, 0.0)],
    [( 5.00, -0.65, 0.0),( 0.0, 0.0, 0.0)],
    [( 7.00, -0.65, 0.0),( 0.0, 0.0, 0.0)],
    [( 9.00, -0.65, 0.0),( 0.0, 0.0, 90.0)],
    [( 9.00,  0.00, 0.0),( 0.0, 0.0, 90.0)],
    [( 9.00,  0.65, 0.0),( 0.0, 0.0, 180.0)],
    [( 7.00,  0.65, 0.0),( 0.0, 0.0, 180.0)],
    [( 5.00,  0.65, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.00,  0.65, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.00,  0.65, 0.0),( 0.0, 0.0, -90.0)],
    [( 1.00,  0.00, 0.0),( 0.0, 0.0, -90.0)]
]

waypoints2=[
    [( 0.53, -0.10, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.15, -0.07, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.52, -0.60, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.80, -1.41, 0.0),( 0.0, 0.0, 180.0)],
    [( 2.03, -2.59, 0.0),( 0.0, 0.0, 180.0)],
    [( 2.50, -3.00, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.00, -3.00, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.60, -2.60, 0.0),( 0.0, 0.0, 180.0)],
    [( 4.00, -1.70, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.65, -0.62, 0.0),( 0.0, 0.0, 180.0)],
    [( 3.20, -0.10, 0.0),( 0.0, 0.0, 180.0)],
    [( 2.80, -0.16, 0.0),( 0.0, 0.0, 180.0)],
    [( 2.52, -0.60, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.92, -1.70, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.69, -2.74, 0.0),( 0.0, 0.0, 180.0)],
    [( 1.05, -3.23, 0.0),( 0.0, 0.0, 180.0)],
    [( 0.30, -3.01, 0.0),( 0.0, 0.0, 180.0)],
    [( 0.00, -1.80, 0.0),( 0.0, 0.0, 180.0)],
    [( 0.13, -0.76, 0.0),( 0.0, 0.0, 180.0)]
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
        for pose in waypoints1:
            goal=goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
