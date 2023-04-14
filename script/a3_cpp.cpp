#include "ros/ros.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <actionlib/client/simple_action_client.h>
#include "math.h"
#define PI 3.1415926

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_route_publish");
    ros::NodeHandle nh;
    //订阅move_base操作服务器
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ros::Publisher my_route_pub = nh.advertise<move_base_msgs::MoveBaseAction>("move_base", 10);
    ros::Rate loop_rate(1);
    //define 2d estimate pose
    double alpha = PI / 2; //radian value
    double x_pos = 3.00;
    double y_pos = 3.00;

    while (ros::ok())
    {
        move_base_msgs::MoveBaseAction pose_msg;

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.target_pose.pose.position.x = x_pos;
        pose_msg.target_pose.pose.position.y = y_pos;
        pose_msg.target_pose.pose.position.z = z_pos;

        pose_msg.target_pose.pose.orientation.z = sin(alpha / 2);
        pose_msg.target_pose.pose.orientation.w = cos(alpha / 2);

        my_route_pub.publish(pose_msg);
        ROS_INFO("Setting to :(%f,%f)", x_pos, y_pos);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
