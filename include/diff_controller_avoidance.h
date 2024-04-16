#include "ros/ros.h"
#include <iostream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// message
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "obstacle_detector/Obstacles.h"
using namespace std;

enum class MODE
{
    IDLE,
    PATH_RECEIVED,
    TRACKING,
    TRANSITION
};

enum class OBS_STATUS : int
{
    NO_IMPACT = 1,
    OBSERVATION = 2,
    ALERTING = 3,
    EMERGENCY = 4
};

class RobotPose
{
public:
    double x;
    double y;
    double theta;
};

class obstacleAvoidance
{
public:
    obstacleAvoidance(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

private:
    friend class pathTracking;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;

    ros::ServiceServer params_srv_;
    bool initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    ros::Subscriber obs_sub;
    void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr &obs_msg);

    obstacle_detector::Obstacles all_obstacles;
    obstacle_detector::Obstacles potential_obstacles;
    obstacle_detector::Obstacles obstaclesFilter(RobotPose cur, obstacle_detector::Obstacles obs);
};

class pathTracking
{
public:
    pathTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
    ~pathTracking();
    void initialize();
    bool initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    obstacleAvoidance *obstacleAvoidancer_;

    RobotPose goal_pose;
    RobotPose cur_pose;
    void pathRequest(RobotPose cur_, RobotPose goal_);
    std::vector<RobotPose> global_path;

    RobotPose rollingwindow(RobotPose cur);
    void diff_controller(RobotPose localgoal, RobotPose cur);
    double speedPlanning(double last_vel, int direction, double peak_vel);
    double angleLimiting(double theta);

    MODE mode;
    MODE past_mode;
    void switchMode(MODE next);

    // publisher
    ros::Publisher vel_pub;       // for chassis control
    ros::Publisher localgoal_pub; // for rviz display
    ros::Publisher center_pub;    // for rviz display

    // subscriber
    ros::Subscriber pose_sub;
    // void poseCallback(const nav_msgs::Odometry::ConstPtr &msg); //base_pose_ground_truth
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg); // ekf_pose
    ros::Subscriber goal_sub;
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // timer
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);

    RobotPose velocity;
    void publishVelocity(RobotPose vel_);

    double countdistance(RobotPose pose1, RobotPose pose2);
    bool if_xy_reached(RobotPose cur, RobotPose goal);
    bool if_theta_reached(RobotPose cur, RobotPose goal);
    bool xy_reached;
    double peak_v;
    double linear_brake_distance_;

    // param server
    bool p_active_;
    bool if_avoidance_enable_;
    double control_frequency_;
    double lookahead_distance_;
    // linear
    double xy_tolerance_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_kp_;
    double linear_brake_vel_;
    double linear_min_brake_distance_;
    double linear_brake_distance_ratio_;
    // angular
    double theta_tolerance_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_kp_;
    double angular_brake_distance_;
    double angular_min_brake_distance_;
    double angular_brake_distance_ratio_;
};
