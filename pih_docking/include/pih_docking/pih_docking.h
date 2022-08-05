#pragma once
#include <mbf_costmap_core/costmap_recovery.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/Bool.h>
#include <ikh_ros_msgs/Labjack_dout.h>

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;

namespace pih_docking
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class PIHDocking : public mbf_costmap_core::CostmapRecovery
{
public:
  
    /// Doesn't do anything: initialize is where the actual work happens
    PIHDocking()
      : local_costmap_(NULL),
        tf_(NULL),
        initialized_(false),
        canceled_(true)
    {
        zero_twist_.linear.x = 0.0;
        zero_twist_.linear.y = 0.0;
        zero_twist_.linear.z = 0.0;
        zero_twist_.angular.x = 0.0;
        zero_twist_.angular.y = 0.0;
        zero_twist_.angular.z = 0.0;

    }

    /// Initialize the parameters of the behavior
    virtual void initialize (std::string n, tf2_ros::Buffer* tf,
                     costmap_2d::Costmap2DROS* global_costmap,
                     costmap_2d::Costmap2DROS* local_costmap);

    /// Run the behavior
    virtual uint32_t runBehavior(std::string& message);

    virtual bool cancel();

    virtual ~PIHDocking() { };

    private:
    gm::Pose2D getCurrentRobotPose() const;
    uint32_t moveBack() const;
    uint32_t publishStop() const;
    double getCurrentDiff(const gm::Pose2D referencePose) const;
    // void io_dock_cb(const std_msgs::Bool msg);
    void io_dock_cb(const ikh_ros_msgs::Labjack_dout msg);

    ros::NodeHandle nh_;
    costmap_2d::Costmap2DROS* local_costmap_;
    tf2_ros::Buffer* tf_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber io_dock_sub;
    bool initialized_;
    bool canceled_;

    gm::Twist zero_twist_;

    double controller_frequency_;
    double linear_vel_back_;
    double step_back_length_;
    double step_back_timeout_;
    std::string occupied_ths_param_;
    double occupied_ths_;
    double occupied_ths_crv;

    bool io_dock = 0;

};

} 
