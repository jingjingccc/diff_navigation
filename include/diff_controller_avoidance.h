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
#include "sensor_msgs/LaserScan.h"
#include "obstacle_detector/Obstacles.h"
using namespace std;

enum class MODE
{
    IDLE,
    PATH_RECEIVED,
    TRACKING,
    EMERGENCY
};

class RobotPose
{
public:
    double x;
    double y;
    double theta;
};

std::string node_name;
// tools
double angleLimiting(double theta);
double countdistance(RobotPose pose1, RobotPose pose2);

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
    void obstacleCallback(const sensor_msgs::LaserScan::ConstPtr &obs_msg);

    sensor_msgs::LaserScan all_obstacles;
    double obstaclesFilter(RobotPose cur, bool if_reversing, double turning_threshold);
    RobotPose obstacleHeuristic(RobotPose cur_vel, double lethal_distance_);

    ros::Time start_deadzone_time;

    // param
    // define avoidance behaviour
    double slow_down_distance_;
    double emergency_stop_distance_;
    double deadzone_timeout_;
    // proximity heuristic
    double heuristic_a_; // determine how aggresive the heuristic function be, 0.0< a <= 1.0
};

class pathTracking
{
public:
    pathTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
    ~pathTracking();
    void initialize();
    bool initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
    friend class obstacleAvoidance;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // publisher
    ros::Publisher vel_pub;             // for chassis control
    ros::Publisher lookahead_point_pub; // for rviz display
    ros::Publisher center_pub;          // for rviz display
    obstacleAvoidance *obstacleAvoidancer_;

    // subscriber
    ros::Subscriber pose_sub;
    // void poseCallback(const nav_msgs::Odometry::ConstPtr &msg); //base_pose_ground_truth
    // void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg); // ekf_pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg); // tracked_pose // carto_pose
    ros::Subscriber goal_sub;
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // timer and its switch case
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);
    MODE mode, past_mode;
    void switchMode(MODE next);
    void diff_controller(RobotPose cur);

    RobotPose goal_pose, cur_pose;
    void pathRequest(RobotPose cur_, RobotPose goal_);
    std::vector<RobotPose> global_path;
    RobotPose rollingwindow(RobotPose cur, double lookahead_dist_);

    // velocity profile
    RobotPose velocity;
    void publishVelocity(RobotPose vel_);
    double peak_v;
    double linear_brake_distance_;
    bool xy_reached, theta_reached;
    bool if_xy_reached(RobotPose cur, RobotPose goal);
    bool if_theta_reached(RobotPose cur, RobotPose goal);
    double speedPlanning(double last_vel, double peak_vel);
    void stationaryChassis();

    // transform tolerance
    ros::Time last_cur_time;
    bool robot_pose_available;

    double _;
    bool if_avoidance_enable_;
    // param server
    bool p_active_;
    double control_frequency_;
    double robot_pose_tolerance_;
    bool if_allow_reversing_;
    // lookahead
    double lookahead_time_;
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    // curvature heuristic
    double sharp_turn_threshold_;
    double min_circularmotion_radius_;
    double max_sharp_turn_vel_;
    // velocity profile linear
    double xy_tolerance_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_kp_;
    double linear_brake_vel_;
    double linear_min_brake_distance_;
    double linear_brake_distance_ratio_;
    // velocity profile angular
    double theta_tolerance_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_acceleration_heuristic_;
    double angular_kp_;
    double angular_brake_distance_;
};