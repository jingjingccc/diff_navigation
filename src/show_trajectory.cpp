#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::Path path;
geometry_msgs::PoseStamped cur_pose;
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    cur_pose.pose.position.x = msg->pose.pose.position.x;
    cur_pose.pose.position.y = msg->pose.pose.position.y;
    cur_pose.pose.position.z = msg->pose.pose.position.z;
    cur_pose.pose.orientation.x = msg->pose.pose.orientation.x;
    cur_pose.pose.orientation.y = msg->pose.pose.orientation.y;
    cur_pose.pose.orientation.z = msg->pose.pose.orientation.z;
    cur_pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

bool start_record = false;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    path.poses.clear();
    start_record = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_trajectory");
    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("rpp_trajectory", 1);
    ros::Subscriber pose_sub = nh.subscribe("/ekf_pose", 1, poseCallback);
    ros::Subscriber goal_sub = nh.subscribe("/nav_goal", 10, goalCallback);

    path.header.frame_id = "map";
    cur_pose.header.frame_id = "map";

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!start_record)
        {
            // ROS_INFO("[%s]: Waiting for goal received", node_name.c_str());
            continue;
        }

        path.header.stamp = ros::Time::now();
        cur_pose.header.stamp = path.header.stamp;
        path.poses.push_back(cur_pose);
        path_pub.publish(path);

        loop_rate.sleep();
    }
}