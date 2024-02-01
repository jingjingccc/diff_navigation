#include "diff_controller_plus.h"

pathTracking::pathTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty e;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("/params", &pathTracking::initializeParams, this);
    initializeParams(e.request, e.response);
    initialize();
}

void pathTracking::initialize()
{
    timer_ = nh_.createTimer(ros::Duration(1 / control_frequency_), &pathTracking::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();

    // flag
    mode = MODE::IDLE;
    past_mode = MODE::IDLE;

    while (!ros::service::waitForService("/move_base/make_plan", ros::Duration(3.0)))
    {
        ROS_INFO("Waiting for service /move_base/make_plan to become available");
    }

    ROS_INFO("Initialized!");
}

pathTracking::~pathTracking()
{
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("lookahead_distance");
    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("linear_max_vel");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("linear_kp");
    nh_local_.deleteParam("linear_min_brake_distance");
    nh_local_.deleteParam("linear_brake_distance_ratio");
    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("angular_max_vel");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("angular_kp");
    nh_local_.deleteParam("angular_brake_distance");
    nh_local_.deleteParam("angular_min_brake_distance");
    nh_local_.deleteParam("angular_brake_distance_ratio");
}

bool pathTracking::initializeParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    get_param_ok = nh_local_.param<double>("control_frequency", control_frequency_, 70);
    get_param_ok = nh_local_.param<double>("lookahead_distance", lookahead_distance_, 0.3);

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
    get_param_ok = nh_local_.param<double>("angular_kp", angular_kp_, 1.5);
    get_param_ok = nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.15);
    get_param_ok = nh_local_.param<double>("angular_min_brake_distance", angular_min_brake_distance_, 0.15);
    get_param_ok = nh_local_.param<double>("angular_brake_distance_ratio", angular_brake_distance_ratio_, 0.15);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            localgoal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
            center_pub = nh_.advertise<geometry_msgs::PointStamped>("/center_point", 10);
            // pose_sub = nh_.subscribe("/base_pose_ground_truth", 10, &pathTracking::poseCallback, this);
            pose_sub = nh_.subscribe("/ekf_pose", 10, &pathTracking::poseCallback, this);
            goal_sub = nh_.subscribe("/nav_goal", 10, &pathTracking::goalCallback, this);
        }
        else
        {
            vel_pub.shutdown();
            localgoal_pub.shutdown();
            center_pub.shutdown();
            pose_sub.shutdown();
            goal_sub.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO("set param ok");
    }
    else
    {
        ROS_WARN_STREAM("set param failed");
    }
    return true;
}

void pathTracking::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pose.x = msg->pose.position.x;
    goal_pose.y = msg->pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    goal_pose.theta = yaw;
    ROS_INFO("RECEIVED GOAL - %f %f %f", goal_pose.x, goal_pose.y, goal_pose.theta);

    linear_brake_distance_ = countdistance(cur_pose, goal_pose) * linear_brake_distance_ratio_;
    if (linear_brake_distance_ < linear_min_brake_distance_)
        linear_brake_distance_ = linear_min_brake_distance_;

    pathRequest(cur_pose, goal_pose);
    switchMode(MODE::PATH_RECEIVED);
    xy_reached = false;
    peak_v = 0;
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

void pathTracking::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) // base_pose_ground_truth
{
    cur_pose.x = msg->pose.pose.position.x;
    cur_pose.y = msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose.theta = yaw;
}

void pathTracking::pathRequest(RobotPose cur_, RobotPose goal_)
{
    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = "map";
    cur.pose.position.x = cur_.x;
    cur.pose.position.y = cur_.y;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_.theta);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_.x;
    goal.pose.position.y = goal_.y;
    goal.pose.position.z = 0;

    q.setRPY(0, 0, goal_pose.theta);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    if (!client)
    {
        ROS_FATAL("Couid not initialize get plan service from %s", client.getService().c_str());
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
            tf2::Quaternion q;
            tf2::fromMsg(path_msg.poses[i].pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta = yaw;
            global_path.push_back(pose);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service make_plan");
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
        ROS_INFO("IDLE");
        velocity.x = 0;
        velocity.y = 0;
        velocity.theta = 0;
        publishVelocity(velocity);
        break;
    case MODE::PATH_RECEIVED:
        ROS_INFO("PATH_RECEIVED");
        switchMode(MODE::TRACKING);
        break;
    case MODE::TRACKING:
        ROS_INFO("TRACKING");
        if (if_xy_reached(cur_pose, goal_pose) && if_theta_reached(cur_pose, goal_pose)) // reached
        {
            velocity.x = 0;
            velocity.y = 0;
            velocity.theta = 0;
            switchMode(MODE::IDLE);
            break;
        }
        RobotPose local_goal = rollingwindow(cur_pose); // find local goal
        diff_controller(local_goal, cur_pose);          // track local goal with speed planning
        break;
    }
}

RobotPose pathTracking::rollingwindow(RobotPose cur)
{
    // determine x, y position of localgoal
    RobotPose local_goal, past_local_goal;
    if (countdistance(cur_pose, goal_pose) < lookahead_distance_ + 0.2)
    {
        local_goal = goal_pose;
    }
    else
    {
        bool localgoal_found = false;
        int flag = 1, past_flag = 1;
        for (int i = 0; i < global_path.size(); i++)
        {
            past_flag = flag;
            if (countdistance(cur, global_path.at(i)) >= lookahead_distance_)
                flag = 1; // exterior
            else
                flag = 0; // interior

            if (flag - past_flag == 1)
            {
                local_goal = global_path.at(i);
                if (i == 0)
                    past_local_goal = global_path.at(i);
                else
                    past_local_goal = global_path.at(i - 1);
                localgoal_found = true;
                break;
            }
        }

        if (localgoal_found)
        {
            local_goal.x = (local_goal.x + past_local_goal.x) / 2;
            local_goal.y = (local_goal.y + past_local_goal.y) / 2;
        }
        else
        {
            double mindis = 10000, past_dis, dis = 0;
            for (int i = 0; i < global_path.size(); i++)
            {
                past_dis = dis;
                dis = countdistance(cur, global_path.at(i));
                if (mindis > dis)
                {
                    local_goal = global_path.at(i);
                    mindis = dis;
                }
            }
        }
    }

    return local_goal;
}

void pathTracking::diff_controller(RobotPose localgoal, RobotPose cur)
{
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
    center.x = (pow(cur.x, 2) + pow(cur.y, 2) - pow(localgoal.x, 2) - pow(localgoal.y, 2) + 2 * (localgoal.y - cur.y) * robotLine_perpendicular.theta / robotLine_perpendicular.y) / (2 * (cur.x - localgoal.x) - 2 * (cur.y - localgoal.y) * robotLine_perpendicular.x / robotLine_perpendicular.y);
    center.y = (robotLine_perpendicular.theta - robotLine_perpendicular.x * center.x) / robotLine_perpendicular.y;
    double circularMotion_R = countdistance(center, cur);

    // for rviz display "center point"
    geometry_msgs::PointStamped center_;
    center_.header.stamp = ros::Time::now();
    center_.header.frame_id = "map";
    center_.point.x = center.x;
    center_.point.y = center.y;
    center_.point.z = 0;
    center_pub.publish(center_);

    /*=====================================================================================
                        determine robot go_forward_or_backward (Vx + or -)
    =====================================================================================*/
    int go_forward_or_backward = 0;
    // extend the robot cur_pose length=1 with cur angle
    RobotPose fake_cur_point;
    fake_cur_point.x = cur.x + cos(cur.theta);
    fake_cur_point.y = cur.y + sin(cur.theta);

    // bring local_goal and cur_pose to the line (robotLine_perpendicular). If bigger than 0, on the right side of the line
    double local_ = robotLine_perpendicular.x * localgoal.x + robotLine_perpendicular.y * localgoal.y - robotLine_perpendicular.theta;
    double fake_cur_ = robotLine_perpendicular.x * fake_cur_point.x + robotLine_perpendicular.y * fake_cur_point.y - robotLine_perpendicular.theta;

    // if local_goal and cur_pose are on the same side, then go forward
    if ((local_ >= 0 && fake_cur_ >= 0) || (local_ <= 0 && fake_cur_ <= 0))
        go_forward_or_backward = 1;
    else
        go_forward_or_backward = -1;
    // ROS_INFO("go_forward_or_backward  =  %d", go_forward_or_backward);

    /*=====================================================================================
                                    determine localgoal_theta
    =====================================================================================*/
    // calculate the slope that perpendicular to the line, localgoal to center point
    localgoal.theta = atan2(-1, (localgoal.y - center.y) / (localgoal.x - center.x));
    // use cross product to check localgoal.theta correct or 180 deg difference
    RobotPose fake_local, fake_cur;
    fake_local.x = cos(localgoal.theta);
    fake_local.y = sin(localgoal.theta);
    fake_cur.x = cos(cur.theta);
    fake_cur.y = sin(cur.theta);
    // (center point to cur point) cross (center point to localgoal point)
    bool inner_dir = ((localgoal.y - center.y) * (cur.x - center.x) - (localgoal.x - center.x) * (cur.y - center.y)) >= 0 ? true : false;
    // cur.theta vector cross localgoal.theta vector
    bool outer_dir = (fake_local.y * fake_cur.x - fake_local.x * fake_cur.y) >= 0 ? true : false;
    if (!xy_reached)
    {
        if (inner_dir != outer_dir)
            localgoal.theta = angleLimiting(localgoal.theta + M_PI);
    }
    else
    {
        // if xy has reached, just turn around to meet the goal angle
        localgoal.theta = goal_pose.theta;
    }

    /*=====================================================================================
                                determine rotate direction
    =====================================================================================*/
    // use cross product to determine rotate direction
    // cur.theta vector cross localgoal.theta vector
    // (since localgoal.theta might changed, instead of using "outer_dir" directly, we calculate it again)
    int rotate_direction = 0;
    double cross = cos(cur.theta) * sin(localgoal.theta) - sin(cur.theta) * cos(localgoal.theta);
    if (cross >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;
    // ROS_INFO("rotate_direction  =  %d", rotate_direction);

    // for rviz display "local_goal"
    geometry_msgs::PoseStamped local;
    local.header.stamp = ros::Time::now();
    local.header.frame_id = "map";
    local.pose.position.x = localgoal.x;
    local.pose.position.y = localgoal.y;
    local.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, localgoal.theta);
    local.pose.orientation.x = q.x();
    local.pose.orientation.y = q.y();
    local.pose.orientation.z = q.z();
    local.pose.orientation.w = q.w();
    localgoal_pub.publish(local);

    /*=====================================================================================
                calculte velocity with speed planning linear.x and angular.z
    =====================================================================================*/
    double output_v = 0;

    // linear x
    double last_v = velocity.x; // for speed planning in acceleration stage

    if (if_xy_reached(cur_pose, goal_pose))
    {
        xy_reached = true;
        velocity.x = 0;
        velocity.y = 0;
    }
    else
    {
        output_v = speedPlanning(last_v, go_forward_or_backward, peak_v);
        velocity.x = output_v;
    }
    peak_v = (output_v > peak_v) ? output_v : peak_v;

    // angular z
    double last_w = velocity.theta;
    if (if_xy_reached(cur_pose, goal_pose))
    {
        if (if_theta_reached(cur_pose, goal_pose))
        {
            velocity.theta = 0;
            switchMode(MODE::IDLE);
        }
        else
        {
            // use p control for decceleration when xy has reached
            double theta_err = fabs(angleLimiting(cur_pose.theta - goal_pose.theta));
            if (theta_err > angular_brake_distance_)
                velocity.theta = fabs(last_w) + (angular_acceleration_ / control_frequency_);
            else
                velocity.theta = theta_err * angular_kp_;
        }
    }
    else
    {
        // use w = v / r for angular z
        velocity.theta = fabs(output_v / circularMotion_R);
        // if (velocity.theta > angular_max_vel_)
        // {
        //     velocity.theta = angular_max_vel_;
        // }
    }
    velocity.theta *= rotate_direction;
    publishVelocity(velocity);
}

double pathTracking::speedPlanning(double last_vel, int direction, double peal_vel)
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
        double dcc = pow(peal_vel, 2) / 2 / linear_brake_distance_;
        output_vel_ = sqrt(2 * dcc * xy_err);
        if (output_vel_ < linear_brake_vel_)
            output_vel_ = xy_err * linear_kp_;
    }
    if (output_vel_ > linear_max_vel_)
        output_vel_ = linear_max_vel_;
    output_vel_ *= direction;
    return output_vel_;
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
    // ROS_INFO("velocity = (%f, %f, %f)", vel.linear.x, vel.linear.y, vel_.theta);

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