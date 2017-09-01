### Original global planner



### Published Topics
~/YmgGPHybROS/ymggp_plan (nav_msgs/Path)  
~/YmgGPHybROS/navfn_plan (nav_msgs/Path)  



### Subscribed Topics
~/YmgGPHybROS/reset_flag (std_msgs/Empty)  



### Parameters
(move base param) ~/base_global_planner: "ymggp/YmgGPHybROS"  

~/YmgGPROS/path_resolution (double[points/m], default: 10.0)

~/YmgGPROS/stuck_timeout (double[sec], default: 10.0)
When the robot stops while this time, this planner changes algorithm to dijkstra.

~/YmgGPROS/navfn_goal_dist (double[m], default: 5.0)

~/YmgGPROS/recovery_dist (double[m], default: 2.0)
