#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <pih_docking/pih_docking.h>
#include <mbf_msgs/ExePathResult.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(pih_docking::PIHDocking, mbf_costmap_core::CostmapRecovery)

namespace pih_docking
{

void PIHDocking::initialize(std::string name, tf2_ros::Buffer* tf,
                                cmap::Costmap2DROS* /*global_costmap*/, cmap::Costmap2DROS* local_costmap)
{
    tf_ = tf;
    local_costmap_ = local_costmap;

    cmd_vel_pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
    io_dock_sub = nh_.subscribe("io_dock_topic", 10, &PIHDocking::io_dock_cb, this);

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("linear_vel_back", linear_vel_back_, -0.3);
    private_nh.param("step_back_length", step_back_length_, 1.0);
    private_nh.param("step_back_timeout", step_back_timeout_, 15.0);
    private_nh.searchParam("smach/occupied_ths", occupied_ths_param_);
    private_nh.param(occupied_ths_param_, occupied_ths_, 45.0);
    // occupied_ths_crv = occupied_ths_*2.55;
    occupied_ths_crv = occupied_ths_;


    initialized_ = true;
}

// void PIHDocking::io_dock_cb(const std_msgs::Bool msg)
// {
//     io_dock = msg.data;
// }

void PIHDocking::io_dock_cb(const ikh_ros_msgs::Labjack_dout msg)
{
    io_dock = msg.state;
}


// Get pose in local costmap frame
gm::Pose2D PIHDocking::getCurrentRobotPose() const
{
    geometry_msgs::PoseStamped p;
    local_costmap_->getRobotPose(p);
    gm::Pose2D pose;

    pose.x = p.pose.position.x;
    pose.y = p.pose.position.y;
    pose.theta = tf2::getYaw(p.pose.orientation);
    return pose;
}

uint32_t PIHDocking::moveBack() const
{
    unsigned int mx, my;

    gm::Twist twist;
    twist.linear.x = linear_vel_back_;

    ros::Rate r(controller_frequency_);
    const gm::Pose2D initialPose = getCurrentRobotPose();
    local_costmap_->getCostmap()->worldToMap(initialPose.x, initialPose.y, mx, my);
    double prev_cost = double(local_costmap_->getCostmap()->getCost(mx,my));

    ros::Time time_begin = ros::Time::now();
    while (double dist_diff = (step_back_length_ - getCurrentDiff(initialPose)) > 0.01 && !io_dock && ros::ok())
    {
        ROS_DEBUG("dist_diff = %.2f", dist_diff);
        double remaining_time = dist_diff / twist.linear.x;

        // time out
        if(step_back_timeout_ > 0.0 &&
                time_begin + ros::Duration(step_back_timeout_) < ros::Time::now())
        {
            publishStop();
            ROS_WARN("time out moving backwards");
            ROS_WARN("%.2f [sec] elapsed.", step_back_timeout_);
            break;
        }

        if (canceled_) {
            return mbf_msgs::ExePathResult::CANCELED;
        }

        cmd_vel_pub_.publish(twist);

        ////////////
        const gm::Pose2D& currentPose = getCurrentRobotPose();
        local_costmap_->getCostmap()->worldToMap(currentPose.x, currentPose.y, mx, my);
        double cost = double(local_costmap_->getCostmap()->getCost(mx,my));
        if (cost-prev_cost>0 && cost>occupied_ths_crv)
        {
            ROS_ERROR("REJECTING BACKWARDS MOTION");
            return mbf_msgs::ExePathResult::SUCCESS;
        }
        prev_cost = cost;
        ////////////

        ros::spinOnce();
        r.sleep();
    }
    return mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t PIHDocking::publishStop() const
{
    ros::Rate r(controller_frequency_);
    for (double t=0; t<1.0; t+=1/controller_frequency_)
    {
        cmd_vel_pub_.publish(zero_twist_);
        if (canceled_) {
            return mbf_msgs::ExePathResult::CANCELED;
        }
        r.sleep();
    }
    return mbf_msgs::ExePathResult::SUCCESS;
}

double PIHDocking::getCurrentDiff(const gm::Pose2D referencePose) const
{
    const gm::Pose2D& currentPose = getCurrentRobotPose();
    double current_diff = (currentPose.x - referencePose.x) * (currentPose.x - referencePose.x) +
            (currentPose.y - referencePose.y) * (currentPose.y - referencePose.y);
    current_diff = sqrt(current_diff);

    return current_diff;
}

bool PIHDocking::cancel() {
    canceled_ = true;
    return true;
}

uint32_t PIHDocking::runBehavior (std::string& message)
{
    ROS_ASSERT (initialized_);
    ROS_INFO("Start Moveback-Recovery.");
    canceled_ = false;

    // Figure out how long we can safely run the behavior
    const gm::Pose2D& initialPose = getCurrentRobotPose();

    // initial pose
    ROS_DEBUG("initial pose (%.2f, %.2f, %.2f)", initialPose.x,
                    initialPose.y, initialPose.theta);

    ROS_INFO("attempting step back");
    moveBack();
    ROS_INFO("complete step back");

    double final_diff = getCurrentDiff(initialPose);
    ROS_DEBUG("final_diff = %.2f",final_diff);

    publishStop();
    ROS_INFO("Finished MoveBack-Recovery");
}


} // namespace pih_docking
