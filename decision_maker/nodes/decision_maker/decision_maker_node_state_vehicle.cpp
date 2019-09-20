#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryInitState(cstring_t& state_name, int status)
{
  ROS_INFO("Hello Autoware World");

  ROS_INFO("ROS is ready");

  tryNextState("init_start");
}

void DecisionMakerNode::updateInitState(cstring_t& state_name, int status)
{
  static bool is_first_callback = true;

  if (!is_first_callback)
  {
    return;
  }
  ROS_INFO("Autoware is initializing now");
  is_first_callback = false;
}

void DecisionMakerNode::entrySensorInitState(cstring_t& state_name, int status)
{
  Subs["filtered_points"] = nh_.subscribe("filtered_points", 1, &DecisionMakerNode::callbackFromFilteredPoints, this);
  publishOperatorHelpMessage("Please publish \"filtered_points\"");
}

void DecisionMakerNode::updateSensorInitState(cstring_t& state_name, int status)
{
  const double timeout = 1;

  if (sim_mode_)
  {
    ROS_INFO("DecisionMaker is in simulation mode");
    tryNextState("sensor_is_ready");
    return;
  }
  else if (isEventFlagTrue("received_pointcloud_for_NDT"))
  {
    tryNextState("sensor_is_ready");
  }
  ROS_INFO("DecisionMaker is waiting filtered_point for NDT");
}

void DecisionMakerNode::entryLocalizationInitState(cstring_t& state_name, int status)
{
  Subs["current_pose"] = nh_.subscribe("current_pose", 5, &DecisionMakerNode::callbackFromCurrentPose, this);
  publishOperatorHelpMessage("Please start localization in stopped.");
}

void DecisionMakerNode::updateLocalizationInitState(cstring_t& state_name, int status)
{
  if (isLocalizationConvergence(current_status_.pose.position))
  {
    tryNextState("localization_is_ready");
  }
}

void DecisionMakerNode::entryPlanningInitState(cstring_t& state_name, int status)
{
  Subs["closest_waypoint"] =
      nh_.subscribe("closest_waypoint", 1, &DecisionMakerNode::callbackFromClosestWaypoint, this);
}

void DecisionMakerNode::updatePlanningInitState(cstring_t& state_name, int status)
{
  tryNextState("planning_is_ready");
}

void DecisionMakerNode::entryVehicleInitState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Please prepare vehicle for depature.");
}

void DecisionMakerNode::updateVehicleInitState(cstring_t& state_name, int status)
{
  if (true /*isEventFlagTrue("received_vehicle_status")*/)
  {
    tryNextState("vehicle_is_ready");
  }
}

void DecisionMakerNode::entryVehicleReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateVehicleReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateBatteryChargingState(cstring_t& state_name, int status)
{
 }

void DecisionMakerNode::entryVehicleEmergencyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateVehicleEmergencyState(cstring_t& state_name, int status)
{
}
}
