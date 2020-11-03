# ll2_global_planner

This ROS package provides a global planner that finds routes in Lanelet2 maps.

## Dependencies

- Lanelet2 (Installed along with Autoware)

## ROS API

#### Subs

- `lanelet_map_bin` (autoware_lanelet2_msgs/MapBin)  
This topic is used to load the lanelet2 map.
- `move_base_simple/goal` (geometry_msgs/PoseStamped)  
This topic is used to receive a waypoint goal from RViz.

#### Pubs

- `based/lane_waypoints_raw` (autoware_msgs/LaneArray)
The waypoints that make up the planned route.
It is expected that this topic will be fed into the waypoint_replanner to ensure speeds are comfortable around turns.

#### Transform Listeners

- `map` -> `base_link` dynamic TF  
Used to determine the current position of the vehicle within the map.

#### Transform Broadcasters


#### Configuration Parameters

See the `config/params.yaml` file for a list of parameters and their descriptions.

## Notes
