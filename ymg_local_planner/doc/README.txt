### Original local planner based on dwa_local_planner



### Published Topics
~/YmgLPROS/global_plan (nav_msgs/Path)  
このプランナーが追従するglobal plan．

~/YmgLPROS/local_plan (nav_msgs/Path)  
dwaにより決定されたlocal plan．

~/YmgLPROS/local_goal (geometry_msgs/PointStamped)  
local goalの位置を可視化．デバッグ用．



### Parameters
(move base param) ~/base_local_planner: "ymglp/YmgLPROS"  


~/YmgLPROS/max_trans_vel (double[m/s], default: 0.55)  
The absolute value of the maximum translational velocity for the robot in m/s  
ロボットの速度の絶対値の上限．

~/YmgLPROS/min_trans_vel (double[m/s], default: 0.1)  
The absolute value of the minimum translational velocity for the robot in m/s  
ロボットの速度の絶対値の上限．

~/YmgLPROS/max_vel_x (double[m/s], default: 0.55)  
The maximum x velocity for the robot in m/s  
ロボットのx方向の速度の絶対値の上限．

~/YmgLPROS/min_vel_x (double[m/s], default: 0.0)  
The minimum x velocity for the robot in m/s  
ロボットのx方向の速度の絶対値の下限．

~/YmgLPROS/max_vel_y (double[m/s], default: 0.1)  
The maximum y velocity for the robot in m/s  
ロボットのy方向の速度の絶対値の上限．

~/YmgLPROS/min_vel_y (double[m/s], default: -0.1)  
The minimum y velocity for the robot in m/s  
ロボットのy方向の速度の絶対値の下限．

~/YmgLPROS/max_rot_vel (double[rad/s], default: 1.0)  
The absolute value of the maximum rotational velocity for the robot in rad/s  
ロボットの角速度の絶対値の上限.

~/YmgLPROS/min_rot_vel (double[rad/s], default: 0.4)  
The absolute value of the minimum rotational velocity for the robot in rad/s  
ロボットの角速度の絶対値の下限．

~/YmgLPROS/acc_lim_x (double[m/s^2], default: 2.5)  
The acceleration limit of the robot in the x direction
ロボットのx方向の加速度の絶対値の上限．

~/YmgLPROS/acc_lim_y (double[m/s^2], default: 2.5)  
The acceleration limit of the robot in the y direction  
ロボットのy方向の加速度の絶対値の上限．

~/YmgLPROS/acc_lim_theta (double[rad/s^2], default: 3.2)  
The acceleration limit of the robot in the theta direction  
ロボットの角加速度の絶対値の上限．

~/YmgLPROS/acc_limit_trans (double[m/s^2], default: 0.1)  
The absolute value of the maximum translational acceleration for the robot in m/s^2  
ロボットの(x,y方向を合わせた)加速度の絶対値の上限．



~/YmgLPROS/xy_goal_tolerance (double[m], default: 0.1)  
Within what maximum distance we consider the robot to be in goal  
ゴール到達判定時のxy方向のズレの許容範囲．

~/YmgLPROS/yaw_goal_tolerance (double[rad], default: 0.1)  
Within what maximum angle difference we consider the robot to face goal direction  
ゴール到達判定時のyaw方向のズレの許容範囲．


~/YmgLPROS/trans_stopped_vel (double[m/s], default: 0.1)  
Below what maximum velocity we consider the robot to be stopped in translation  
ロボットの速度の絶対値がこの値以下の場合にロボットが止まっているとみなす．

~/YmgLPROS/rot_stopped_vel (double[m/s], default: 0.1)  
Below what maximum rotation velocity we consider the robot to be stopped in rotation  
ロボットの角速度の絶対値がこの値以下の場合にロボットが止まっているとみなす．



~/YmgLPROS/sim_time (double[sec], defalut: 1.7)  
The amount of time to roll trajectories out for in seconds  
経路を算出する際のシミュレート時間．

~/YmgLPROS/additional_sim_time (double[sec], default: 1.7)  
The amount of time for calc obstacle costs in seconds  
障害物のコスト算出の際に，追加で前方を見ることができる．

~/YmgLPROS/sim_granularity (double[m], default: 0.025)  
The granularity with which to check for collisions along each trajectory in meters  
シミュレートの際の点の間隔．

~/YmgLPROS/angular_sim_granularity (double[rad], default: 0.1)  
The granularity with which to check for collisions for rotations in radians  
シミュレートの際の角度の間隔．

~/YmgLPROS/use_dwa (bool, default: False)
Use dynamic window approach to constrain sampling velocities to small window.
プランニングにdwaを使用するか．
以下のpath_tolerance, obstacle_toleranceはymglpのパラメータ.
path_distance_bias, goal_distance_bias, occdist_scaleはdwaのパラメータ．

~/YmgLPROS/path_tolerance (double[m], default: 0.1)
The tolerance between global path and endpoint of the simulated local path.
ローカルプランの終端とグローバルパスの距離の許容値．
距離はグリッドマップ上で計算されるため，距離の精度はローカルコストマップの解像度に依存するため注意．
（解像度が粗く，path_toleranceが小さい場合には有効なパスが引かれない可能性がある．）

~/YmgLPROS/obstacle_tolerance (int, default: 253)
The maximum cost of the cell which the path can be drawn.
シミュレートされた経路上にこの値より大きいコストがあった場合，その経路は棄却される．

~/YmgLPROS/path_distance_bias (double, default: 32.0)  
The weight for the path distance part of the cost function  
dwaでのコスト計算の際のパスとの距離の重み．

~/YmgLPROS/goal_distance_bias (double, default: 24.0)  
The weight for the goal distance part of the cost function  
dwaでのコスト計算の際のローカルゴールとの距離の重み．

~/YmgLPROS/occdist_scale (double, default: 0.01)  
The weight for the obstacle distance part of the cost function  
dwaでのコスト計算の際の障害物を避ける重み．

~/YmgLPROS/local_goal_distance (double, default: 2.0)
The distance to the local goal
ローカルゴールを現在位置から(グローバルパス上の)どのくらい先に置くか．


~/YmgLPROS/forward_point_distance (double[m], default: 0.325)  
The distance from the center point of the robot to place an additional scoring point, in meters  
グローバルパスとの距離，ローカルゴールまでの距離からコストを算出する際にロボットの前方に追加の点を置くことができる．
その際，算出されるコストはロボットの中心と追加の点で算出されたコストの平均値となる．
設定値を大きくし過ぎるとゴールにたどり着くのが困難になる可能性があるため，xy_goal_toleranceを大きめに取ること．

~/YmgLPROS/scaling_speed (double[m/s], default: 0.25)  
The absolute value of the velocity at which to start scaling the robot's footprint, in m/s  
速度の閾値．ロボットの速度がこの値を超えると，footprintがスケーリングされる．

~/YmgLPROS/max_scaling_factor (double, default: 1.5)  
The maximum factor to scale the robot's footprint by  
footprintのスケーリングの割合の上限．
デフォルト値を用い，ロボットがmax_trans_velで走行するとfootprintが1.5倍にスケーリングされる．

~/YmgLPROS/vx_samples (int, default: 3)  
The number of samples to use when exploring the x velocity space  
dwaでのロボットのx方向速度のシミュレートの数．

~/YmgLPROS/vy_samples (int, default: 0)  
The number of samples to use when exploring the y velocity space  
dwaでのロボットのx方向速度のシミュレートの数．

~/YmgLPROS/vth_samples (int, default: 21)  
The number of samples to use when exploring the theta velocity space  
ロボットの角速度のシミュレートの数．

~/YmgLPROS/restore_defaults (bool, default: False)  
Restore to the original configuration  
