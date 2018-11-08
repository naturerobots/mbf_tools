#include"mbf_demo_controller/mbf_demo_controller.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mbf_demo_controller::DemoController, mbf_costmap_core::CostmapController);

namespace mbf_demo_controller{

  DemoController::DemoController(){

  }

  DemoController::~DemoController(){

  }

  uint32_t DemoController::computeVelocityCommands(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity,
      geometry_msgs::TwistStamped &cmd_vel,
      std::string &message)
  {
    return 0;
  }

  bool DemoController::isGoalReached(
      double xy_tolerance,
      double yaw_tolerance)
  {
    return false;
  }

  bool DemoController::setPlan(
      const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    return true;
  }

  bool DemoController::cancel(){
    return true;
  }

  void DemoController::initialize(
      std::string name,
      tf::TransformListener *tf,
      costmap_2d::Costmap2DROS *costmap_ros)
  {

  }


}; /* mbf_demo_controller */
