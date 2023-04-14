#!/usr/bin/env python
# encoding: utf-8
 
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

pose = [( 3.00, 3.00, 0.0),( 0.0, 0.0, 180.0)]

def goal_pose(pose):
    goal_pose=PoseWithCovarianceStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.pose.position.x=pose[0][0]
    goal_pose.pose.pose.position.y=pose[0][1]
    goal_pose.pose.pose.position.z=pose[0][2]
 
    # r, p, y  欧拉角转四元数
    x,y,z,w=tf.transformations.quaternion_from_euler(pose[1][0],pose[1][1],pose[1][2])
 
    goal_pose.pose.pose.orientation.x=x
    goal_pose.pose.pose.orientation.y=y
    goal_pose.pose.pose.orientation.z=z
    goal_pose.pose.pose.orientation.w=w

    #goal_pose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    goal_pose.pose.covariance[0] = 0.25
    goal_pose.pose.covariance[6*1+1] = 0.25
    goal_pose.pose.covariance[6*5+5] = 0.06853892326654787

    return goal_pose
 
 
if __name__ == "__main__": 
    #节点初始化
    rospy.init_node('pose_publisher') 
    #创建MoveBaseAction client
    #client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 2)
    #等待MoveBaseAction server启动
    #client.wait_for_server()
    position=goal_pose(pose)
    pose_pub.publish(position)

    while not rospy.is_shutdown():
        position=goal_pose(pose)
        pose_pub.publish(position)
        #client.wait_for_result()
        #ROS_INFO("Setting pose:")
        
