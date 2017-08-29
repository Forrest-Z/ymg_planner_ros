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

~/YmgLPROS/sim_time_obstacle (double[sec], defalut: 1.7)  
The amount of time to roll trajectories out for in seconds  

~/YmgLPROS/sim_time_trajectory (double[sec], default: 1.7)  
The amount of time to roll trajectories out for in seconds  

~/YmgLPROS/sim_granularity (double[m], default: 0.025)  
The granularity with which to check for collisions along each trajectory in meters  

~/YmgLPROS/angular_sim_granularity (double[rad], default: 0.1)  
The granularity with which to check for collisions for rotations in radians  

~/YmgLPROS/path_distance_bias (double, default: 32.0)  
The weight for the path distance part of the cost function  

~/YmgLPROS/goal_distance_bias (double, default: 24.0)  
The weight for the goal distance part of the cost function  

~/YmgLPROS/occdist_scale (double, default: 0.01)  
The weight for the obstacle distance part of the cost function  

~/YmgLPROS/stop_time_buffer (double[sec], default: 0.2)  
The amount of time that the robot must stop before a collision in order for a trajectory to be considered valid in seconds  

~/YmgLPROS/oscillation_reset_dist (double[m], default: 0.05)  
The distance the robot must travel before oscillation flags are reset, in meters  

~/YmgLPROS/oscillation_reset_angle (double[rad], default: 0.2)  
The angle the robot must turn before oscillation flags are reset, in radians  

~/YmgLPROS/forward_point_distance (double[m], default: 0.325)  
The distance from the center point of the robot to place an additional scoring point, in meters  

~/YmgLPROS/scaling_speed (double[m/s], default: 0.25)  
The absolute value of the velocity at which to start scaling the robot's footprint, in m/s  

~/YmgLPROS/max_scaling_factor (double, default: 0.2)  
The maximum factor to scale the robot's footprint by  

~/YmgLPROS/vx_samples (int, default: 3)  
The number of samples to use when exploring the x velocity space  

~/YmgLPROS/vy_samples (int, default: 10)  
The number of samples to use when exploring the y velocity space  

~/YmgLPROS/vth_samples (int, default: 20)  
The number of samples to use when exploring the theta velocity space  

~/YmgLPROS/restore_defaults (bool, default: False)  
Restore to the original configuration  
