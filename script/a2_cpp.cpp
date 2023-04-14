#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "cmath"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimate_2d");
    ros::NodeHandle nh;
    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    ros::Rate loop_rate(5);
    //define 2d estimate pose
    double alpha = M_PI / 4; //radian value
    double x_pos = 4.00;
    double y_pos = 4.00;

    while (ros::ok())
    {
        geometry_msgs::PoseWithCovarianceStamped pose_msg;

        pose_msg.header.stamp = ros::Time::now();
        //地图坐标系
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x_pos;
        pose_msg.pose.pose.position.y = y_pos;
        //协方差矩阵，可根据实际需要，调整参数。
        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853892326654787;
        pose_msg.pose.pose.orientation.z = sin(alpha);
        pose_msg.pose.pose.orientation.w = cos(alpha);

        initial_pose_pub.publish(pose_msg);
        ROS_INFO("Setting to :(%f,%f)", x_pos, y_pos);
        // 其实每次在通过2D pose estimate 指令点击RVIZ中的栅格地图时，在终端处也会出现同样的位置消息，即用C++程序取代RVIZ环境下的人工选取点。
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
