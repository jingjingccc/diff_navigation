
#include "diff_controller_avoidance.h"

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
    velocity.x = 0;
    velocity.y = 0;
    velocity.theta = 0;

    // flag
    mode = MODE::IDLE;
    past_mode = MODE::IDLE;
    last_cur_time = ros::Time::now();
    robot_pose_available = false;

    if (if_avoidance_enable_)
        obstacleAvoidancer_ = new obstacleAvoidance(nh_, nh_local_);

    while (!ros::service::waitForService("/move_base/GlobalPlanner/make_plan", ros::Duration(3.0))) /// move_base/GlobalPlanner/make_plan
    {
        ROS_INFO_STREAM("[" << node_name << "]: Waiting for service /move_base/GlobalPlanner/make_plan to become available");
    }

    ROS_INFO_STREAM("[" << node_name << "]: Initialized OK!");
    timer_.start();
}

pathTracking::~pathTracking()
{
    stationaryChassis();
    publishVelocity(velocity);

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
    get_param_ok = nh_local_.param<bool>("if_allow_reversing", if_allow_reversing_, false);
    get_param_ok = nh_local_.param<bool>("if_avoidance_enable", if_avoidance_enable_, false);

    get_param_ok = nh_local_.param<double>("lookahead_time", lookahead_time_, 0.3);
    get_param_ok = nh_local_.param<double>("min_lookahead_distance", min_lookahead_distance_, 0.3);
    get_param_ok = nh_local_.param<double>("max_lookahead_distance", max_lookahead_distance_, 0.3);

    get_param_ok = nh_local_.param<double>("sharp_turn_threshold", sharp_turn_threshold_, 0.3);
    get_param_ok = nh_local_.param<double>("min_circularmotion_radius", min_circularmotion_radius_, 0.3);
    get_param_ok = nh_local_.param<double>("min_sharp_turn_vel", min_sharp_turn_vel_, 0.1);

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
            // pose_sub = nh_.subscribe("/base_pose_ground_truth", 10, &pathTracking::poseCallback, this);
            // pose_sub = nh_.subscribe("/ekf_pose", 10, &pathTracking::poseCallback, this);
            pose_sub = nh_.subscribe("/carto_pose", 10, &pathTracking::poseCallback, this);
            goal_sub = nh_.subscribe("/nav_goal", 10, &pathTracking::goalCallback, this);
            reach_pub = nh_.advertise<std_msgs::Bool>("/mission_finish", 10);

            timer_ = nh_.createTimer(ros::Duration(1 / control_frequency_), &pathTracking::timerCallback, this, false, false);
            timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
        }
        else
        {
            vel_pub.shutdown();
            lookahead_point_pub.shutdown();
            pose_sub.shutdown();
            goal_sub.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[" << node_name << "]: pathTracking set param OK!");
    }
    else
    {
        ROS_WARN_STREAM("[" << node_name << "]: pathTracking set param OK!");
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
    ROS_INFO("[%s]: cur_pose = (%f, %f, %f)", node_name.c_str(), cur_pose.x, cur_pose.y, cur_pose.theta);

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
//     robot_pose_available = true;
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
    // ROS_INFO("pathRequeest: %f %f %f", goal_.x, goal_.y, goal_.theta);
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
    bool if_apply_reversing;
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
        if (if_avoidance_enable_)
            obstacleAvoidancer_->obstacle_detected = false;
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
            std_msgs::Bool mission_finish_flag;
            mission_finish_flag.data = true;
            reach_pub.publish(mission_finish_flag);
            break;
        }
        if_apply_reversing = diff_controller(cur_pose); // track local goal with speed planning
        break;
    }

    if (if_avoidance_enable_)
    {
        switch (mode)
        {
        case MODE::TRACKING:
        {
            double nearest_potential_obs = obstacleAvoidancer_->obstaclesFilter(cur_pose, velocity.x > 0 ? false : true, sharp_turn_threshold_);

            if (nearest_potential_obs != 100 && fabs(nearest_potential_obs) <= obstacleAvoidancer_->emergency_stop_distance_)
            {
                // ROS_INFO_STREAM("[" << node_name << "]: Obstacle detected... emergency stop...");
                ROS_INFO("!!! Obstacle detected... emergency stop... !!!");
                ROS_INFO("nearest_potential_obs: %f", nearest_potential_obs);
                stationaryChassis();                                         // switch to emergency mode
                obstacleAvoidancer_->start_deadzone_time = ros::Time::now(); // and, start counting...
                ROS_INFO("EMERGENCY DIRECTION: %f", nearest_potential_obs);
                if(nearest_potential_obs > 0)
                    switchMode(MODE::EMERGENCY_RIGHT);
                else
                    switchMode(MODE::EMERGENCY_LEFT);
                break;
            }
            
            nearest_potential_obs = fabs(nearest_potential_obs);
            bool if_need_slow_down = (!xy_reached && velocity.x != 0.0 && !if_apply_reversing && nearest_potential_obs > obstacleAvoidancer_->emergency_stop_distance_ && nearest_potential_obs < obstacleAvoidancer_->slow_down_distance_) ? true : false;
            if (obstacleAvoidancer_->obstacle_detected)
            {
                double orignal_vel = velocity.x;
                if (ros::Time::now().toSec() - obstacleAvoidancer_->start_slowdown_time.toSec() < obstacleAvoidancer_->min_slowdown_duration_)
                    velocity = obstacleAvoidancer_->obstacleHeuristic(velocity, nearest_potential_obs); // keep slow down
                else
                {
                    if (if_need_slow_down)
                        velocity = obstacleAvoidancer_->obstacleHeuristic(velocity, nearest_potential_obs);
                    else
                        obstacleAvoidancer_->obstacle_detected = false;
                }

                // just for check out
                if (orignal_vel != velocity.x)
                {
                    // ROS_INFO_STREAM("[" << node_name << "]: Obstacle detected... slow down...");
                    // ROS_INFO("(%f --> %f)", orignal_vel, velocity.x);
                }
            }
            else
            {
                if (nearest_potential_obs == 100)
                    obstacleAvoidancer_->obstacle_detected = false;
                else if (if_need_slow_down)
                {
                    obstacleAvoidancer_->start_slowdown_time = ros::Time::now();
                    obstacleAvoidancer_->obstacle_detected = true;
                }
            }
        }
        break;
        case MODE::EMERGENCY_LEFT:
        case MODE::EMERGENCY_RIGHT:
        { // if over dedzone_timeout, plan new Globalplanner
            if (ros::Time::now().toSec() - obstacleAvoidancer_->start_deadzone_time.toSec() < obstacleAvoidancer_->deadzone_timeout_)
            {
                velocity.theta = (mode == MODE::EMERGENCY_LEFT) ? -0.2 : 0.2;
                // stationaryChassis();
            }
            else
            {
                pathRequest(cur_pose, goal_pose);
                if (global_path.size() <= 0)
                {
                    switchMode(MODE::IDLE);
                    ROS_ERROR_STREAM("[" << node_name << "]:  (emergency mode) Failed to make plan!");
                }
                else
                {
                    obstacleAvoidancer_->obstacle_detected = false;
                    ROS_INFO_STREAM("[" << node_name << "]: Make plan to escape from deadzone!");
                    switchMode(MODE::TRACKING);
                }
            }
        }
        break;
        }
    }

    // last velocity check
    velocity.x = (velocity.x > 0 && velocity.x >= linear_max_vel_) ? linear_max_vel_ : velocity.x;
    velocity.x = (velocity.x < 0 && velocity.x <= -linear_max_vel_) ? -linear_max_vel_ : velocity.x;

    // ROS_INFO("Output velocity = (%f, %f, %f)", velocity.x, velocity.y, velocity.theta)
    publishVelocity(velocity);
}

RobotPose pathTracking::rollingwindow(RobotPose cur, double lookahead_dist_)
{
    RobotPose lookahead_point_;

    // find the closet point from robot to path
    // make sure there are at least two point in global path list
    while (global_path.size() > 2)
    {
        if (countdistance(cur, global_path.at(0)) >= countdistance(cur, global_path.at(1)))
            global_path.erase(global_path.begin());
        else
            break;
    }
    bool lookahead_point_found = false;
    int lookahead_idx = 0;
    for (int i = 0; i < global_path.size(); i++)
    {
        if (countdistance(cur, global_path.at(i)) < lookahead_dist_)
        {
            lookahead_point_ = global_path.at(i);
            lookahead_point_found = true;
            lookahead_idx = i;
        }
        else
            break;
    }

    if (!lookahead_point_found)
        lookahead_point_ = global_path.at(1);
    else if (global_path.size() >= 2 + lookahead_idx)
        global_path.erase(global_path.begin(), global_path.begin() + lookahead_idx);

    if (countdistance(cur_pose, goal_pose) < lookahead_dist_)
        lookahead_point_ = goal_pose;

    if (lookahead_point_.x == 0 && lookahead_point_.y == 0 && lookahead_point_.theta == 0)
    {
        ROS_INFO("lookahead point = %f, %f, %f", lookahead_point_.x, lookahead_point_.y, lookahead_point_.theta);

        ROS_WARN("=================");
        ROS_WARN("path size = %ld", global_path.size());
        ROS_WARN("=================");
    }
    return lookahead_point_;
}

bool pathTracking::diff_controller(RobotPose cur)
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
    // heuristic condition
    bool turning_over_threshold = (fabs(angleLimiting(lookahead_point.theta - cur.theta)) > sharp_turn_threshold_) ? true : false;
    bool goalpose_is_nearing = (countdistance(cur, goal_pose) < 0.8) ? true : false;
    if (!xy_reached && turning_over_threshold) // && !goalpose_is_nearing
    {
        // ROS_INFO("processeing for curvature heuristic");
        apply_curvature_heuristic = true;
        double slow_linear_vel = velocity.x * (circularMotion_R / min_circularmotion_radius_);
        // velocity.x = (slow_linear_vel < max_sharp_turn_vel_) ? max_sharp_turn_vel_ : slow_linear_vel;
        // velocity.x = (velocity.x > linear_max_vel_) ? linear_max_vel_ : velocity.x;
        if (slow_linear_vel < min_sharp_turn_vel_)
            velocity.x = min_sharp_turn_vel_;
        else
            velocity.x = slow_linear_vel;

        // double original = velocity.x;
        // ROS_INFO("curvature heuristic: %f ----*(%f)----> %f ----(min_sharp_turn) ---->%f", original, circularMotion_R / min_circularmotion_radius_, slow_linear_vel, min_sharp_turn_vel_);
        // ROS_INFO("curvature heuristic: %f ----*(%f)----> %f", original, circularMotion_R / min_circularmotion_radius_, velocity.x);
    }

    /*=====================================================================================
                            determine robot go_forward_or_backward (Vx + or -)
        =====================================================================================*/
    int go_forward_or_backward = 1;

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

    bool reversing_turning_first = false;

    if (!if_allow_reversing_ && go_forward_or_backward == -1)
    {
        velocity.x = 0;
        velocity.y = 0;
        lookahead_point.theta = goal_pose.theta;
        reversing_turning_first = true;
    }
    // ROS_INFO("go_forward_or_backward  =  %d", go_forward_or_backward);

    /*=====================================================================================
                calculte turning velocity with speed planning linear.x and angular.z
    =====================================================================================*/
    if (!xy_reached && velocity.x != 0)
    {
        // use w = v / r for angular z
        if (apply_curvature_heuristic)
            velocity.theta = fabs(velocity.theta) + (angular_acceleration_heuristic_ / control_frequency_);
        else
            velocity.theta = fabs(velocity.x / circularMotion_R);

        if (velocity.theta > angular_max_vel_)
            velocity.theta = angular_max_vel_;
    }
    else if (!if_allow_reversing_ && !xy_reached && reversing_turning_first)
    {
        velocity.theta = 0.45;
    }
    else
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

    /*=====================================================================================
                                     determine rotate direction
    =====================================================================================*/
    // use cross product to determine rotate direction
    // (cur.theta vector) cross (localgoal.theta vector)
    // (since localgoal.theta might changed, instead of using "outer_dir" directly, we calculate it again)
    int rotate_direction = 0;

    double rotation_cross;
    if (go_forward_or_backward == 1 || reversing_turning_first)
        rotation_cross = cos(cur.theta) * (lookahead_point.y - cur.y) - sin(cur.theta) * (lookahead_point.x - cur.x);
    else if (go_forward_or_backward == -1)
        rotation_cross = cos(cur.theta) * (cur.y - lookahead_point.y) - sin(cur.theta) * (cur.x - lookahead_point.x);

    if (xy_reached)
        rotation_cross = cos(cur.theta) * sin(goal_pose.theta) - sin(cur.theta) * cos(goal_pose.theta);

    rotate_direction = (rotation_cross >= 0) ? 1 : -1;
    // ROS_INFO("rotate_direction  =  %d", rotate_direction);

    velocity.x *= go_forward_or_backward;
    velocity.theta *= rotate_direction;

    return reversing_turning_first;
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
        // ROS_INFO("speedPlaning: %f ----(%f)----> %f", abs(last_vel), d_vel, output_vel_);
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

void pathTracking::publishVelocity(RobotPose vel_)
{
    geometry_msgs::Twist vel;
    vel.linear.x = vel_.x;
    vel.linear.y = 0;
    vel.angular.z = vel_.theta;
    vel_pub.publish(vel);
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

obstacleAvoidance::obstacleAvoidance(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;

    obstacle_detected = false;

    std_srvs::Empty e;
    params_srv_ = nh_local_.advertiseService("/navigation_obstacle/params", &obstacleAvoidance::initializeParams, this);
    initializeParams(e.request, e.response);
}

bool obstacleAvoidance::initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    bool get_param_ok = false;
    get_param_ok = nh_local_.param<double>("slow_down_distance", slow_down_distance_, 5.0);
    get_param_ok = nh_local_.param<double>("emergency_stop_distance", emergency_stop_distance_, 1.5);
    get_param_ok = nh_local_.param<double>("deadzone_timeout", deadzone_timeout_, 5.0);
    get_param_ok = nh_local_.param<double>("heuristic_a", heuristic_a_, 0.5);
    get_param_ok = nh_local_.param<double>("min_slowdown_vel", min_slowdown_vel_, 0.2);
    get_param_ok = nh_local_.param<double>("min_slowdown_duration", min_slowdown_duration_, 2.0);
    get_param_ok = nh_local_.param<double>("obstacle_resolution", obstacle_resolution_, 5.0);

    obs_sub = nh_.subscribe("/unilidar/scan", 1, &obstacleAvoidance::obstacleCallback, this);
    le_obs_pub = nh_.advertise<geometry_msgs::PointStamped>("/lethal_obstacle", 10);

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[" << node_name << "]: ObstacleAvoidance set param OK!");
    }
    else
    {
        ROS_WARN_STREAM("[" << node_name << "]: ObstacleAvoidance set param OK!");
    }
    return true;
}

void obstacleAvoidance::obstacleCallback(const sensor_msgs::LaserScan::ConstPtr &obs_msg)
{
    // storing in all_obstacls
    all_obstacles = *obs_msg;
}

// void obstacleAvoidance::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr &obs_msg)
// {
//     // storing in all_obstacls
//     all_obstacles = *obs_msg;
// }

// double obstacleAvoidance::obstaclesFilter(RobotPose cur, bool if_reversing, double turning_threshold) // original_version
// {
//     // using laserscan
//     sensor_msgs::LaserScan all_obs = all_obstacles;
//     int all_obs_num = all_obs.ranges.size();

//     double lethal_obs_rad = angleLimiting(turning_threshold * M_PI / 180);
//     int lethal_obs_num = std::ceil(2 * lethal_obs_rad / all_obs.angle_increment);

//     double lethal_obs_dist = -1, index = -1;
//     for (int i = -lethal_obs_num / 2; i <= lethal_obs_num / 2; i++)
//     {
//         int n = (if_reversing == false) ? i + all_obs_num / 2 : i + 0;
//         n = (n < 0) ? n + all_obs_num : n;
//         lethal_obs_dist = ((lethal_obs_dist < 0 || lethal_obs_dist > all_obs.ranges.at(n)) && !isinf(all_obs.ranges.at(n))) ? all_obs.ranges.at(n) : lethal_obs_dist;
//     }
//     return lethal_obs_dist;
// }

double obstacleAvoidance::obstaclesFilter(RobotPose cur, bool if_reversing, double turning_threshold)
{
    turning_threshold = 15; // in deg
    // using laserscan
    sensor_msgs::LaserScan all_obs = all_obstacles;
    int all_obs_num = all_obs.ranges.size();

    int lethal_obs_min_index = -1, lethal_obs_num = std::ceil(2 * turning_threshold / 180 * M_PI / all_obs.angle_increment); // round up
    double lethal_obs_min_dist = 100;
    int count_point_left = 0, count_point_right = 0;
    for (int i = -lethal_obs_num / 2; i <= lethal_obs_num / 2; i++)
    {
        int n = (if_reversing == false) ? i + all_obs_num / 2 : (i < 0 ? i + all_obs_num : i);
        double lethal_obs_dist = (!isinf(all_obs.ranges.at(n)) || all_obs.ranges.at(n) != all_obs.range_max) ? all_obs.ranges.at(n) : 100;
        if (lethal_obs_dist != 100)
        {
            if(lethal_obs_dist <= slow_down_distance_)
            {   
                count_point_right = (i<0)? count_point_right + 1: count_point_right;
                count_point_left = (i>0)? count_point_left + 1: count_point_left;
            }
            
            if (lethal_obs_dist < lethal_obs_min_dist && lethal_obs_dist <= slow_down_distance_)
            {
                lethal_obs_min_dist = lethal_obs_dist;
                lethal_obs_min_index = n;
            }
        }
    }
    
    if (lethal_obs_min_dist != 100 && lethal_obs_min_index != -1 && count_point_left + count_point_right >= 5) // if lethal_obs_max_dist = 1, it means that there are no lethal obstacles.
    {
        ROS_INFO("count point: left = %d / right = %d", count_point_left, count_point_right);
        // convert to RobotPose msg
        RobotPose lethal_obs; // relative to the robot
        double a = lethal_obs_min_index * all_obs.angle_increment + M_PI;
        lethal_obs.x = cos(a) * lethal_obs_min_dist;
        lethal_obs.y = sin(a) * lethal_obs_min_dist;
        lethal_obs.theta = lethal_obs_min_dist;
        // ROS_INFO("original lethal_obs:(%f, %f)", lethal_obs.x, lethal_obs.y);

        // // recalculate the obstacle pose (use obstacle reesolution)
        // int xx = (int)std::floor(lethal_obs.x / obstacle_resolution_); // round down
        // int yy = (int)std::floor(lethal_obs.y / obstacle_resolution_);
        // lethal_obs.x = (xx > 0) ? xx * obstacle_resolution_ : (xx + 1) * obstacle_resolution_;
        // lethal_obs.y = (yy > 0) ? yy * obstacle_resolution_ : (yy + 1) * obstacle_resolution_;
        // lethal_obs_min_dist = sqrt(pow(lethal_obs.x, 2) + pow(lethal_obs.y, 2)) - obstacle_resolution_;

        // ROS_INFO("lethal_obs:(%f, %f), dis from robot %f", lethal_obs.x, lethal_obs.y, lethal_obs_min_dist);
        // for rviz display
        geometry_msgs::PointStamped le_obs;
        // le_obs.header.frame_id = "base_footprint";
        le_obs.header.frame_id = "robot_0/base_footprint";
        le_obs.header.stamp = ros::Time::now();
        le_obs.point.x = lethal_obs.x;
        le_obs.point.y = lethal_obs.y;
        le_obs_pub.publish(le_obs);

        lethal_obs_min_dist = count_point_left < count_point_right ? lethal_obs_min_dist : -lethal_obs_min_dist;
    }
    else
        lethal_obs_min_dist = 100;
 
    return lethal_obs_min_dist; // if there are no obstacles return 100
    // if obstacle at the right hand side, positive
    // if obstacle at the left hand side, negative
}

// double obstacleAvoidance::obstaclesFilter(RobotPose cur, bool if_reversing, double turning_threshold)
// {
//     // using obstacle_detector
//     obstacle_detector::Obstacles all_obs = all_obstacles; // the pose of the obstacles are in base_footprint frame

//     double lethal_obs_rad = angleLimiting(turning_threshold * M_PI / 180);
//     double lethal_obs_dist = -1;

//     for (int i = 0; i < all_obs.circles.size(); i++)
//     {
//         double this_obs_x = all_obs.circles.at(i).center.x;
//         double this_obs_y = all_obs.circles.at(i).center.y;
//         if (angleLimiting(atan(abs(this_obs_y) / abs(this_obs_x))) <= lethal_obs_rad)
//         {
//             if ((if_reversing && this_obs_x < 0) || (!if_reversing && this_obs_x > 0))
//                 lethal_obs_dist = sqrt(this_obs_x * this_obs_x + this_obs_y * this_obs_y);
//         }
//     }

//     return lethal_obs_dist;
// }

RobotPose obstacleAvoidance::obstacleHeuristic(RobotPose cur_vel, double lethal_distance_)
{
    RobotPose heuristic_vel = cur_vel;

    /*==========================================================================
                    proximity heuristic - slow down technique
    ==========================================================================*/
    // heuristic_vel.x = cur_vel.x * heuristic_a_; // original calculation
    // test 1
    // heuristic_vel.x = cur_vel.x * sqrt(abs(heuristic_a_ * (lethal_distance_ / slow_down_distance_)));
    // test 2
    // heuristic_vel.x = cur_vel.x * heuristic_a_ * pow((lethal_distance_ / slow_down_distance_), 1 / 2);
    // test 3
    // heuristic_vel.x = cur_vel.x * sqrt(abs(heuristic_a_ * pow((lethal_distance_ / slow_down_distance_), 1 / 3)));
    heuristic_vel.x = cur_vel.x * 0.95;
    /*==========================================================================)
                            limitation (min_slowdown_vel_)
    ==========================================================================*/
    heuristic_vel.x = (heuristic_vel.x > 0 && heuristic_vel.x <= min_slowdown_vel_) ? min_slowdown_vel_ : heuristic_vel.x;
    heuristic_vel.x = (heuristic_vel.x < 0 && heuristic_vel.x > -min_slowdown_vel_) ? -min_slowdown_vel_ : heuristic_vel.x;

    return heuristic_vel;
}

// tools
double angleLimiting(double theta)
{
    // limit the angle from -pi to pi
    while (theta > M_PI)
        theta -= 2 * M_PI;

    while (theta < -M_PI)
        theta += 2 * M_PI;

    return theta;
}

double countdistance(RobotPose pose1, RobotPose pose2)
{
    return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
}

bool chechVelNaN(double vel, std::string s)
{
    if (isnan(vel))
    {
        ROS_ERROR("ERROR Happened at: %s", s.c_str());
        ROS_ERROR("velocity.x nan deteacted !!!");
        // ros::shutdown();
        return true;
    }
    return false;
}

// main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "regulated_pure_pursuit");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    pathTracking pathTracker(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}
