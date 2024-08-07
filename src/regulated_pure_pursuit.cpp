#include "regulated_pure_pursuit.h"

pathTracking::pathTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    node_name = ros::this_node::getName();

    std_srvs::Empty e;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &pathTracking::initializeParams, this);
    initializeParams(e.request, e.response);

    initialize();
}

void pathTracking::initialize()
{
    // flag
    mode = MODE::IDLE;
    past_mode = MODE::IDLE;
    last_cur_time = ros::Time::now();
    robot_pose_available = false;

    while (!ros::service::waitForService("/move_base/GlobalPlanner/make_plan", ros::Duration(3.0))) /// move_base/GlobalPlanner/make_plan
    {
        ROS_INFO_STREAM("[" << node_name << "]: Waiting for service /move_base/GlobalPlanner/make_plan to become available");
    }

    ROS_INFO_STREAM("[" << node_name << "]: Initialized OK!");
    timer_.start();
}

pathTracking::~pathTracking()
{
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("if_allow_reversing");

    nh_local_.deleteParam("lookahead_time");
    nh_local_.deleteParam("min_lookahead_distance");
    nh_local_.deleteParam("max_lookahead_distance");

    nh_local_.deleteParam("sharp_turn_threshold");
    nh_local_.deleteParam("min_circularmotion_radius");
    nh_local_.deleteParam("min_sharp_turn_vel");

    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("linear_max_vel");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("linear_kp");
    nh_local_.deleteParam("linear_brake_vel");
    nh_local_.deleteParam("linear_min_brake_distance");
    nh_local_.deleteParam("linear_brake_distance_ratio");

    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("angular_max_vel");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("angular_acceleration_heuristic");
    nh_local_.deleteParam("angular_kp");
    nh_local_.deleteParam("angular_brake_distance");
}

bool pathTracking::initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    get_param_ok = nh_local_.param<double>("control_frequency", control_frequency_, 70);
    get_param_ok = nh_local_.param<double>("robot_pose_tolerance", robot_pose_tolerance_, 70);
    get_param_ok = nh_local_.param<bool>("if_allow_reversing", if_allow_reversing_, true);

    get_param_ok = nh_local_.param<double>("lookahead_time", lookahead_time_, 0.3);
    get_param_ok = nh_local_.param<double>("min_lookahead_distance", min_lookahead_distance_, 0.3);
    get_param_ok = nh_local_.param<double>("max_lookahead_distance", max_lookahead_distance_, 0.3);

    get_param_ok = nh_local_.param<double>("sharp_turn_threshold", sharp_turn_threshold_, 0.3);
    get_param_ok = nh_local_.param<double>("min_circularmotion_radius", min_circularmotion_radius_, 0.3);
    get_param_ok = nh_local_.param<double>("max_sharp_turn_vel", max_sharp_turn_vel_, 0.1);

    get_param_ok = nh_local_.param<double>("xy_tolerance", xy_tolerance_, 0.02);
    get_param_ok = nh_local_.param<double>("linear_max_vel", linear_max_vel_, 0.8);
    get_param_ok = nh_local_.param<double>("linear_acceleration", linear_acceleration_, 0.3);
    get_param_ok = nh_local_.param<double>("linear_kp", linear_kp_, 0.8);
    get_param_ok = nh_local_.param<double>("linear_brake_vel", linear_brake_vel_, 0.25);
    get_param_ok = nh_local_.param<double>("linear_min_brake_distance", linear_min_brake_distance_, 0.25);
    get_param_ok = nh_local_.param<double>("linear_brake_distance_ratio", linear_brake_distance_ratio_, 0.3);

    get_param_ok = nh_local_.param<double>("theta_tolerance", theta_tolerance_, 0.03);
    get_param_ok = nh_local_.param<double>("angular_max_vel", angular_max_vel_, 3.0);
    get_param_ok = nh_local_.param<double>("angular_acceleration", angular_acceleration_, 0.15);
    get_param_ok = nh_local_.param<double>("angular_acceleration_heuristic", angular_acceleration_heuristic_, 0.5);
    get_param_ok = nh_local_.param<double>("angular_kp", angular_kp_, 1.5);
    get_param_ok = nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.15);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            lookahead_point_pub = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
            center_pub = nh_.advertise<geometry_msgs::PointStamped>("/center_point", 10);
            // pose_sub = nh_.subscribe("/base_pose_ground_truth", 10, &pathTracking::poseCallback, this);
            // pose_sub = nh_.subscribe("/ekf_pose", 10, &pathTracking::poseCallback, this);
            pose_sub = nh_.subscribe("/carto_pose", 10, &pathTracking::poseCallback, this);
            goal_sub = nh_.subscribe("/nav_goal", 10, &pathTracking::goalCallback, this);

            timer_ = nh_.createTimer(ros::Duration(1 / control_frequency_), &pathTracking::timerCallback, this, false, false);
            timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
        }
        else
        {
            vel_pub.shutdown();
            lookahead_point_pub.shutdown();
            center_pub.shutdown();
            pose_sub.shutdown();
            goal_sub.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[" << node_name << "]: Set param OK!");
    }
    else
    {
        ROS_WARN_STREAM("[" << node_name << "]: Set param OK!");
    }
    return true;
}

void pathTracking::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(_, _, goal_pose.theta);
    ROS_INFO("[%s]: Received goal = (%f, %f, %f)", node_name.c_str(), goal_pose.x, goal_pose.y, goal_pose.theta);

    linear_brake_distance_ = countdistance(cur_pose, goal_pose) * linear_brake_distance_ratio_;
    if (linear_brake_distance_ < linear_min_brake_distance_)
        linear_brake_distance_ = linear_min_brake_distance_;

    pathRequest(cur_pose, goal_pose);

    if (global_path.size() <= 0)
    {
        switchMode(MODE::IDLE);
        ROS_ERROR_STREAM("[" << node_name << "]: Failed to make plan!");
    }
    else
    {
        switchMode(MODE::PATH_RECEIVED);
    }
}

// void pathTracking::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) // base_pose_ground_truth
// {
//     cur_pose.x = msg->pose.pose.position.x;
//     cur_pose.y = msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q);
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     cur_pose.theta = yaw;
// }

// void pathTracking::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) // nav_goal
// {
//     cur_pose.x = msg->pose.pose.position.x;
//     cur_pose.y = msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q);
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     cur_pose.theta = yaw;
// }

void pathTracking::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) // carto_pose
{
    if (ros::Time::now().toSec() - last_cur_time.toSec() > robot_pose_tolerance_)
    {
        ROS_WARN_STREAM("[" << node_name << "]: Received robot pose late...");
        robot_pose_available = false;
    }
    else
        robot_pose_available = true;

    cur_pose.x = msg->pose.position.x;
    cur_pose.y = msg->pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose.theta = yaw;

    last_cur_time = ros::Time::now();
}

void pathTracking::pathRequest(RobotPose cur_, RobotPose goal_)
{
    tf2::Quaternion q;

    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = "map";
    cur.pose.position.x = cur_.x;
    cur.pose.position.y = cur_.y;
    cur.pose.position.z = 0;
    q.setRPY(0, 0, cur_.theta);
    cur.pose.orientation = tf2::toMsg(q);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_.x;
    goal.pose.position.y = goal_.y;
    goal.pose.position.z = 0;
    q.setRPY(0, 0, goal_pose.theta);
    goal.pose.orientation = tf2::toMsg(q);

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan"); /// move_base/GlobalPlanner/make_plan
    if (!client)
    {
        ROS_WARN("[%s]: Couid not initialize get plan service from %s", node_name.c_str(), client.getService().c_str());
        return;
    }

    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    if (client.call(srv))
    {
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;
        global_path.clear();

        for (int i = 0; i < path_msg.poses.size(); i++)
        {
            RobotPose pose;
            pose.x = path_msg.poses[i].pose.position.x;
            pose.y = path_msg.poses[i].pose.position.y;
            tf2::fromMsg(path_msg.poses[i].pose.orientation, q);
            tf2::Matrix3x3(q).getRPY(_, _, pose.theta);
            global_path.push_back(pose);
        }
    }
    else
    {
        ROS_ERROR_STREAM("[" << node_name << "]: Failed to call service make_plan");
    }
}

void pathTracking::switchMode(MODE next)
{
    past_mode = mode;
    mode = next;
}

void pathTracking::timerCallback(const ros::TimerEvent &e)
{
    switch (mode)
    {
    case MODE::IDLE:
        ROS_INFO_STREAM_THROTTLE(5, "[" << node_name << "]: Mode: IDLE");
        stationaryChassis();
        break;
    case MODE::PATH_RECEIVED:
        ROS_INFO_STREAM_THROTTLE(5, "[" << node_name << "]: Mode: PATH_RECEIVED");
        xy_reached = false;
        theta_reached = false;
        peak_v = 0;
        switchMode(MODE::TRACKING);
        break;
    case MODE::TRACKING:
        ROS_INFO_STREAM_THROTTLE(5, "[" << node_name << "]: Mode: TRACKING");
        if (!robot_pose_available)
        {
            stationaryChassis();
            break;
        }
        if (xy_reached && theta_reached) // reached
        {
            stationaryChassis();
            switchMode(MODE::IDLE);
            break;
        }
        diff_controller(cur_pose); // track local goal with speed planning
        break;
    }
    publishVelocity(velocity);
}

RobotPose pathTracking::rollingwindow(RobotPose cur, double lookahead_dist_)
{
    RobotPose min_dis_point_, lookahead_point_;

    // find the closet point from robot to path
    // make sure there are at least two point in global path list
    // min_dis_point_ = global_path.at(1);
    while (global_path.size() > 2)
    {
        if (countdistance(cur, global_path.at(0)) >= countdistance(cur, global_path.at(1)))
            global_path.erase(global_path.begin());
        else
            break;
    }
    bool lookahead_point_found = false;
    for (int i = 0; i < global_path.size(); i++)
    {
        if (countdistance(cur, global_path.at(i)) < lookahead_dist_)
        {
            lookahead_point_ = global_path.at(i);
            lookahead_point_found = true;
        }
        else
        {
            break;
        }
    }

    if (!lookahead_point_found)
        lookahead_point_ = min_dis_point_;

    if (countdistance(cur_pose, goal_pose) < lookahead_dist_)
        lookahead_point_ = goal_pose;

    return lookahead_point_;
}

void pathTracking::diff_controller(RobotPose cur)
{
    /*=====================================================================================
                        calculte linear velocity with speed planning Vt
    =====================================================================================*/
    // linear x
    if (xy_reached || if_xy_reached(cur_pose, goal_pose))
    {
        xy_reached = true;
        velocity.x = 0;
        velocity.y = 0;
    }
    else
    {
        velocity.x = speedPlanning(velocity.x, peak_v);
        peak_v = (velocity.x > peak_v) ? velocity.x : peak_v;
    }

    /*=====================================================================================
                        calculate lookahead distance and lookahead point
    =====================================================================================*/
    double lookahead_distance_ = velocity.x * lookahead_time_;
    lookahead_distance_ = (lookahead_distance_ < min_lookahead_distance_) ? min_lookahead_distance_ : lookahead_distance_;
    lookahead_distance_ = (lookahead_distance_ > max_lookahead_distance_) ? max_lookahead_distance_ : lookahead_distance_;
    RobotPose lookahead_point = rollingwindow(cur, lookahead_distance_);

    /*=====================================================================================
                                count circular motion radius R
    =====================================================================================*/
    RobotPose robotLine, robotLine_perpendicular, center;
    // a1x + b1y = c1 // extend from the robot angle, and go through the cur_pose
    robotLine.x = tan(cur.theta);                                // a1
    robotLine.y = -1;                                            // b1
    robotLine.theta = robotLine.x * cur.x + robotLine.y * cur.y; // c1

    // a2x + b2y = c2 // perpendicular to robotLine, and go through the center point
    robotLine_perpendicular.x = -1 * robotLine.y;                                                          // a2
    robotLine_perpendicular.y = robotLine.x;                                                               // b2
    robotLine_perpendicular.theta = robotLine_perpendicular.x * cur.x + robotLine_perpendicular.y * cur.y; // c2

    // calculate the circular center, and the radius R
    center.x = (pow(cur.x, 2) + pow(cur.y, 2) - pow(lookahead_point.x, 2) - pow(lookahead_point.y, 2) + 2 * (lookahead_point.y - cur.y) * robotLine_perpendicular.theta / robotLine_perpendicular.y) / (2 * (cur.x - lookahead_point.x) - 2 * (cur.y - lookahead_point.y) * robotLine_perpendicular.x / robotLine_perpendicular.y);
    center.y = (robotLine_perpendicular.theta - robotLine_perpendicular.x * center.x) / robotLine_perpendicular.y;
    double circularMotion_R = countdistance(center, cur);

    /*=====================================================================================
                                    determine localgoal_theta
    =====================================================================================*/
    // calculate the slope that perpendicular to the line, localgoal to center point
    lookahead_point.theta = atan2(-1, (lookahead_point.y - center.y) / (lookahead_point.x - center.x));
    // use cross product to check localgoal.theta correct or 180 deg difference
    RobotPose fake_local, fake_cur;
    fake_local.x = cos(lookahead_point.theta);
    fake_local.y = sin(lookahead_point.theta);
    fake_cur.x = cos(cur.theta);
    fake_cur.y = sin(cur.theta);
    // (center point to cur point) cross (center point to localgoal point)
    bool inner_dir = ((lookahead_point.y - center.y) * (cur.x - center.x) - (lookahead_point.x - center.x) * (cur.y - center.y)) >= 0 ? true : false;
    // cur.theta vector cross localgoal.theta vector
    bool outer_dir = (fake_local.y * fake_cur.x - fake_local.x * fake_cur.y) >= 0 ? true : false;
    if (!xy_reached)
    {
        if (inner_dir != outer_dir)
            lookahead_point.theta = angleLimiting(lookahead_point.theta + M_PI);
    }
    else
    {
        // if xy has reached, just turn around to meet the goal angle
        lookahead_point.theta = goal_pose.theta;
    }

    // for rviz display "local_goal"
    geometry_msgs::PoseStamped local;
    local.header.stamp = ros::Time::now();
    local.header.frame_id = "map";
    local.pose.position.x = lookahead_point.x;
    local.pose.position.y = lookahead_point.y;
    local.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, lookahead_point.theta);
    local.pose.orientation.x = q.x();
    local.pose.orientation.y = q.y();
    local.pose.orientation.z = q.z();
    local.pose.orientation.w = q.w();
    lookahead_point_pub.publish(local);

    /*=====================================================================================
                            Apply Curvature Heuristic to linear velocity
       =====================================================================================*/
    bool apply_curvature_heuristic = false;
    if (!xy_reached && (lookahead_point.x != goal_pose.x && lookahead_point.y != goal_pose.y) && fabs(angleLimiting(lookahead_point.theta - cur.theta)) > sharp_turn_threshold_)
    {
        apply_curvature_heuristic = true;
        double original = velocity.x;
        double slow_linear_vel = velocity.x * (circularMotion_R / min_circularmotion_radius_);
        velocity.x = (slow_linear_vel < max_sharp_turn_vel_) ? max_sharp_turn_vel_ : slow_linear_vel;
        if (velocity.x > linear_max_vel_)
            velocity.x = linear_max_vel_;
        if (velocity.x < -linear_max_vel_)
            velocity.x = -linear_max_vel_;
        // ROS_INFO("Apply Curvature Heuristic: %f --> %f --> %f", original, slow_linear_vel, velocity.x);
    }

    /*=====================================================================================
            calculte velocity with speed planning linear.x and angular.z
    =====================================================================================*/
    if (xy_reached)
    {
        if (if_theta_reached(cur_pose, goal_pose))
        {
            velocity.theta = 0;
            theta_reached = true;
        }
        else
        {
            // use p control for decceleration when xy has reached
            double theta_err = fabs(angleLimiting(cur_pose.theta - goal_pose.theta));
            if (theta_err > angular_brake_distance_)
                velocity.theta = fabs(velocity.theta) + (angular_acceleration_ / control_frequency_);
            else
                velocity.theta = theta_err * angular_kp_;
        }
    }
    else
    {
        // use w = v / r for angular z
        if (apply_curvature_heuristic)
        {
            velocity.theta = fabs(velocity.theta) + (angular_acceleration_heuristic_ / control_frequency_);
        }

        velocity.theta = fabs(velocity.x / circularMotion_R);

        if (velocity.theta > angular_max_vel_)
            velocity.theta = angular_max_vel_;
    }

    /*=====================================================================================
                        determine robot go_forward_or_backward (Vx + or -)
    =====================================================================================*/
    int go_forward_or_backward = 1;
    if (if_allow_reversing_)
    {
        // extend the robot cur_pose length=1 with cur angle
        RobotPose fake_cur_point;
        fake_cur_point.x = cur.x + cos(cur.theta);
        fake_cur_point.y = cur.y + sin(cur.theta);

        // bring local_goal and cur_pose to the line (robotLine_perpendicular). If bigger than 0, on the right side of the line
        double local_ = robotLine_perpendicular.x * lookahead_point.x + robotLine_perpendicular.y * lookahead_point.y - robotLine_perpendicular.theta;
        double fake_cur_ = robotLine_perpendicular.x * fake_cur_point.x + robotLine_perpendicular.y * fake_cur_point.y - robotLine_perpendicular.theta;

        // if local_goal and cur_pose are on the same side, then go forward
        if ((local_ >= 0 && fake_cur_ >= 0) || (local_ <= 0 && fake_cur_ <= 0))
            go_forward_or_backward = 1;
        else
            go_forward_or_backward = -1;
    }
    // ROS_INFO("go_forward_or_backward  =  %d", go_forward_or_backward);

    /*=====================================================================================
                                     determine rotate direction
         =====================================================================================*/
    // use cross product to determine rotate direction
    // cur.theta vector cross localgoal.theta vector
    // (since localgoal.theta might changed, instead of using "outer_dir" directly, we calculate it again)
    int rotate_direction = 0;
    double cross = cos(cur.theta) * sin(lookahead_point.theta) - sin(cur.theta) * cos(lookahead_point.theta);
    if (cross >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;
    // ROS_INFO("rotate_direction  =  %d", rotate_direction);

    velocity.x *= go_forward_or_backward;
    velocity.theta *= rotate_direction;
    ROS_INFO("[%s]: Output velocity = (%f, %f, %f)", node_name.c_str(), velocity.x, velocity.y, velocity.theta);
}

double pathTracking::speedPlanning(double last_vel, double peak_vel)
{
    // only for linear velocity
    double output_vel_;
    double xy_err = countdistance(cur_pose, goal_pose);
    if (xy_err > linear_brake_distance_)
    {
        // acceleration
        double d_vel = linear_acceleration_ / control_frequency_;
        output_vel_ = abs(last_vel) + d_vel;
    }
    else
    {
        // deceleration
        double dcc = pow(peak_vel, 2) / 2 / linear_brake_distance_;
        output_vel_ = sqrt(2 * dcc * xy_err);
        if (output_vel_ < linear_brake_vel_ || xy_err < linear_min_brake_distance_)
            output_vel_ = xy_err * linear_kp_;
    }
    if (output_vel_ > linear_max_vel_)
        output_vel_ = linear_max_vel_;
    return output_vel_;
}

void pathTracking::stationaryChassis()
{
    velocity.x = 0;
    velocity.y = 0;
    velocity.theta = 0;
}

double pathTracking::angleLimiting(double theta)
{
    // limit the angle from -pi to pi
    while (theta > M_PI)
        theta -= 2 * M_PI;

    while (theta < -M_PI)
        theta += 2 * M_PI;

    return theta;
}

void pathTracking::publishVelocity(RobotPose vel_)
{
    geometry_msgs::Twist vel;
    vel.linear.x = vel_.x;
    vel.linear.y = 0;
    vel.angular.z = vel_.theta;
    vel_pub.publish(vel);
}

double pathTracking::countdistance(RobotPose pose1, RobotPose pose2)
{
    return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
}

bool pathTracking::if_xy_reached(RobotPose cur, RobotPose goal)
{
    if (xy_tolerance_ >= countdistance(cur, goal))
        return true;
    else
        return false;
}

bool pathTracking::if_theta_reached(RobotPose cur, RobotPose goal)
{
    if (theta_tolerance_ >= fabs(angleLimiting(cur.theta - goal.theta)))
        return true;
    else
        return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    pathTracking pathTracking_(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}