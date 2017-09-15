### Original local planner based on dwa_local_planner



### Published Topics
~/YmgLPROS/global_plan (nav_msgs/Path)  
$B$3$N%W%i%s%J!<$,DI=>$9$k(Bglobal plan$B!%(B

~/YmgLPROS/local_plan (nav_msgs/Path)  
dwa$B$K$h$j7hDj$5$l$?(Blocal plan$B!%(B

~/YmgLPROS/local_goal (geometry_msgs/PointStamped)  
local goal$B$N0LCV$r2D;k2=!%%G%P%C%0MQ!%(B



### Parameters
(move base param) ~/base_local_planner: "ymglp/YmgLPROS"  


~/YmgLPROS/max_trans_vel (double[m/s], default: 0.55)  
The absolute value of the maximum translational velocity for the robot in m/s  
$B%m%\%C%H$NB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/min_trans_vel (double[m/s], default: 0.1)  
The absolute value of the minimum translational velocity for the robot in m/s  
$B%m%\%C%H$NB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/max_vel_x (double[m/s], default: 0.55)  
The maximum x velocity for the robot in m/s  
$B%m%\%C%H$N(Bx$BJ}8~$NB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/min_vel_x (double[m/s], default: 0.0)  
The minimum x velocity for the robot in m/s  
$B%m%\%C%H$N(Bx$BJ}8~$NB.EY$N@dBPCM$N2<8B!%(B

~/YmgLPROS/max_vel_y (double[m/s], default: 0.1)  
The maximum y velocity for the robot in m/s  
$B%m%\%C%H$N(By$BJ}8~$NB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/min_vel_y (double[m/s], default: -0.1)  
The minimum y velocity for the robot in m/s  
$B%m%\%C%H$N(By$BJ}8~$NB.EY$N@dBPCM$N2<8B!%(B

~/YmgLPROS/max_rot_vel (double[rad/s], default: 1.0)  
The absolute value of the maximum rotational velocity for the robot in rad/s  
$B%m%\%C%H$N3QB.EY$N@dBPCM$N>e8B(B.

~/YmgLPROS/min_rot_vel (double[rad/s], default: 0.4)  
The absolute value of the minimum rotational velocity for the robot in rad/s  
$B%m%\%C%H$N3QB.EY$N@dBPCM$N2<8B!%(B

~/YmgLPROS/acc_lim_x (double[m/s^2], default: 2.5)  
The acceleration limit of the robot in the x direction
$B%m%\%C%H$N(Bx$BJ}8~$N2CB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/acc_lim_y (double[m/s^2], default: 2.5)  
The acceleration limit of the robot in the y direction  
$B%m%\%C%H$N(By$BJ}8~$N2CB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/acc_lim_theta (double[rad/s^2], default: 3.2)  
The acceleration limit of the robot in the theta direction  
$B%m%\%C%H$N3Q2CB.EY$N@dBPCM$N>e8B!%(B

~/YmgLPROS/acc_limit_trans (double[m/s^2], default: 0.1)  
The absolute value of the maximum translational acceleration for the robot in m/s^2  
$B%m%\%C%H$N(B(x,y$BJ}8~$r9g$o$;$?(B)$B2CB.EY$N@dBPCM$N>e8B!%(B



~/YmgLPROS/xy_goal_tolerance (double[m], default: 0.1)  
Within what maximum distance we consider the robot to be in goal  
$B%4!<%kE~C#H=Dj;~$N(Bxy$BJ}8~$N%:%l$N5vMFHO0O!%(B

~/YmgLPROS/yaw_goal_tolerance (double[rad], default: 0.1)  
Within what maximum angle difference we consider the robot to face goal direction  
$B%4!<%kE~C#H=Dj;~$N(Byaw$BJ}8~$N%:%l$N5vMFHO0O!%(B


~/YmgLPROS/trans_stopped_vel (double[m/s], default: 0.1)  
Below what maximum velocity we consider the robot to be stopped in translation  
$B%m%\%C%H$NB.EY$N@dBPCM$,$3$NCM0J2<$N>l9g$K%m%\%C%H$,;_$^$C$F$$$k$H$_$J$9!%(B

~/YmgLPROS/rot_stopped_vel (double[m/s], default: 0.1)  
Below what maximum rotation velocity we consider the robot to be stopped in rotation  
$B%m%\%C%H$N3QB.EY$N@dBPCM$,$3$NCM0J2<$N>l9g$K%m%\%C%H$,;_$^$C$F$$$k$H$_$J$9!%(B



~/YmgLPROS/sim_time (double[sec], defalut: 1.7)  
The amount of time to roll trajectories out for in seconds  
dwa$B$G7PO)$r;;=P$9$k:]$N%7%_%e%l!<%H;~4V!%(B

~/YmgLPROS/additional_sim_time (double[sec], default: 1.7)  
The amount of time for calc obstacle costs in seconds  

~/YmgLPROS/sim_granularity (double[m], default: 0.025)  
The granularity with which to check for collisions along each trajectory in meters  
$B%7%_%e%l!<%H$N:]$NE@$N4V3V!%(B

~/YmgLPROS/angular_sim_granularity (double[rad], default: 0.1)  
The granularity with which to check for collisions for rotations in radians  
$B%7%_%e%l!<%H$N:]$N3QEY$N4V3V!%(B


~/YmgLPROS/path_distance_bias (double, default: 32.0)  
The weight for the path distance part of the cost function  
dwa$B$G$N%3%9%H7W;;$N:]$N%Q%9$H$N5wN%$N=E$_!%(B

~/YmgLPROS/goal_distance_bias (double, default: 24.0)  
The weight for the goal distance part of the cost function  
dwa$B$G$N%3%9%H7W;;$N:]$N%m!<%+%k%4!<%k$H$N5wN%$N=E$_!%(B

~/YmgLPROS/occdist_scale (double, default: 0.01)  
The weight for the obstacle distance part of the cost function  
dwa$B$G$N%3%9%H7W;;$N:]$N>c32J*$rHr$1$k=E$_!%(B

~/YmgLPROS/local_goal_distance (double, default: 2.0)
The distance to the local goal
$B%m!<%+%k%4!<%k$r8=:_0LCV$+$i(B($B%0%m!<%P%k%Q%9>e$N(B)$B$I$N$/$i$$@h$KCV$/$+!%(B


~/YmgLPROS/forward_point_distance (double[m], default: 0.325)  
The distance from the center point of the robot to place an additional scoring point, in meters  
$B%0%m!<%P%k%Q%9$H$N5wN%!$%m!<%+%k%4!<%k$^$G$N5wN%$+$i%3%9%H$r;;=P$9$k:]$K%m%\%C%H$NA0J}$KDI2C$NE@$rCV$/$3$H$,$G$-$k!%(B
$B$=$N:]!$;;=P$5$l$k%3%9%H$O%m%\%C%H$NCf?4$HDI2C$NE@$G;;=P$5$l$?%3%9%H$NJ?6QCM$H$J$k!%(B

~/YmgLPROS/scaling_speed (double[m/s], default: 0.25)  
The absolute value of the velocity at which to start scaling the robot's footprint, in m/s  
$BB.EY$NogCM!%%m%\%C%H$NB.EY$,$3$NCM$rD6$($k$H!$(Bfootprint$B$,%9%1!<%j%s%0$5$l$k!%(B

~/YmgLPROS/max_scaling_factor (double, default: 1.5)  
The maximum factor to scale the robot's footprint by  
footprint$B$N%9%1!<%j%s%0$N3d9g$N>e8B!%(B
$B%G%U%)%k%HCM$rMQ$$!$%m%\%C%H$,(Bmax_trans_vel$B$GAv9T$9$k$H(Bfootprint$B$,(B1.5$BG\$K%9%1!<%j%s%0$5$l$k!%(B

~/YmgLPROS/vx_samples (int, default: 3)  
The number of samples to use when exploring the x velocity space  
dwa$B$G$N%m%\%C%H$N(Bx$BJ}8~B.EY$N%7%_%e%l!<%H$N?t!%(B

~/YmgLPROS/vy_samples (int, default: 0)  
The number of samples to use when exploring the y velocity space  
dwa$B$G$N%m%\%C%H$N(Bx$BJ}8~B.EY$N%7%_%e%l!<%H$N?t!%(B

~/YmgLPROS/vth_samples (int, default: 21)  
The number of samples to use when exploring the theta velocity space  
$B%m%\%C%H$N3QB.EY$N%7%_%e%l!<%H$N?t!%(B

~/YmgLPROS/restore_defaults (bool, default: False)  
Restore to the original configuration  
