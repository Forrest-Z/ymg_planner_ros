global and local planner plugins
======================================



## Global Planner

### Published Topics
~/YmgGPROS/plan (nav_msgs/Path)  

### Parameters
(move base param) ~/base_global_planner: "ymggp/YmgGPROS"  

~/YmgGPROS/path_resolution (double[points/m], default: 10.0)  



## Local Planner

### Published Topics
~/YmgLPROS/global_plan (nav_msgs/Path)  
~/YmgLPROS/local_plan (nav_msgs/Path)  

### Parameters
(move base param) ~/base_local_planner: "ymglp/YmglPROS"  

~/YmgLPROS/max_trans_vel (double[m/s], default: 0.55)  
The absolute value of the maximum translational velocity for the robot in m/s  

~/YmgLPROS/min_trans_vel (double[m/s], default: 0.1)  
The absolute value of the minimum translational velocity for the robot in m/s  

~/YmgLPROS/max_vel_x (double[m/s], default: 0.55)  
The maximum x velocity for the robot in m/s  

~/YmgLPROS/min_vel_x (double[m/s], default: 0.0)  
The minimum x velocity for the robot in m/s  

~/YmgLPROS/max_vel_y (double[m/s], default: 0.1)  
The maximum y velocity for the robot in m/s  

~/YmgLPROS/min_vel_y (double[m/s], default: -0.1)  
The minimum y velocity for the robot in m/s  

~/YmgLPROS/max_rot_vel (double[rad/s], default: 1.0)  
The absolute value of the maximum rotational velocity for the robot in rad/s  

~/YmgLPROS/min_rot_vel (double[rad/s], default: 0.4)  
The absolute value of the minimum rotational velocity for the robot in rad/s  

~/YmgLPROS/acc_lim_x (double[m/s^2], default: 2.5)  
The acceleration limit of the robot in the x direction

~/YmgLPROS/acc_lim_y (double[m/s^2], default: 2.5)  
The acceleration limit of the robot in the y direction  

~/YmgLPROS/acc_lim_theta (double[rad/s^2], default: 3.2)  
The acceleration limit of the robot in the theta direction  

~/YmgLPROS/acc_limit_trans (double[m/s^2], default: 0.1)  
The absolute value of the maximum translational acceleration for the robot in m/s^2  

