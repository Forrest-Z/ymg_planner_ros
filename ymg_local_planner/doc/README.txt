### Original local planner based on dwa_local_planner



### Published Topics
~/YmgLPROS/global_plan (nav_msgs/Path)  
このプランナーが追従するglobal plan．local_costmapのサイズにリサイズされている．

~/YmgLPROS/local_plan (nav_msgs/Path)  
ymglpにて算出されたlocal plan．

~/YmgLPROS/local_plan (geometry_msgs/PoseArray)  
ymglpにて算出されたlocal plan．
nav_msgs/pathでは，並進速度が0で回転角速度のみが出ている場合に速度を可視化することができないため．

~/YmgLPROS/local_goal (geometry_msgs/PointStamped)  
local goalの位置を可視化．デバッグ用．local_goalはuse_dwa = trueにした時のみpublishされる．



### Parameters
(move base param) ~/base_local_planner: "ymglp/YmgLPROS"  


~/YmgLPROS/max_vel_x (double[m/s], default: 0.6)  
The maximum x velocity for the robot in m/s  
ロボットの並進速度の上限．マイナスにセットすると，バックすることができる．
min_vel_xは自動的に0.0に設定される．(ymg_local_plannerは障害物手前で停止するように設計されているため．)

~/YmgLPROS/max_rot_vel (double[rad/s], default: 0.6)  
The absolute value of the maximum rotational velocity for the robot in rad/s  
ロボットの回転角速度の絶対値の上限.
min_rot_velは自動的に0.0に設定される．(ymg_local_plannerは障害物手前で停止するように設計されているため．)

~/YmgLPROS/acc_lim_x (double[m/s^2], default: 0.3)  
The acceleration limit of the robot in the x direction
ロボットの並進加速度の絶対値の上限．

~/YmgLPROS/acc_lim_theta (double[rad/s^2], default: 0.3)  
The acceleration limit of the robot in the theta direction  
ロボットの回転角加速度の絶対値の上限．



~/YmgLPROS/xy_goal_tolerance (double[m], default: 0.1)  
Within what maximum distance we consider the robot to be in goal  
ゴール到達判定時のxy方向のズレの許容範囲．

~/YmgLPROS/yaw_goal_tolerance (double[rad], default: 0.1)  
Within what maximum angle difference we consider the robot to face goal direction  
ゴール到達判定時のyaw方向のズレの許容範囲．



~/YmgLPROS/trans_stopped_vel (double[m/s], default: 0.1)  
Below what maximum velocity we consider the robot to be stopped in translation  
ロボットの並進速度の絶対値がこの値以下の場合にロボットが止まっているとみなす．

~/YmgLPROS/rot_stopped_vel (double[m/s], default: 0.1)  
Below what maximum rotation velocity we consider the robot to be stopped in rotation  
ロボットの回転角速度の絶対値がこの値以下の場合にロボットが止まっているとみなす．



~/YmgLPROS/sim_time (double[sec], defalut: 1.5)  
The amount of time to roll trajectories out for in seconds  
経路を算出する際のシミュレート時間．

~/YmgLPROS/sim_granularity (double[m], default: 0.025)  
The granularity with which to check for collisions along each trajectory in meters  
シミュレートの際の点の間隔．

~/YmgLPROS/angular_sim_granularity (double[rad], default: 0.1  = about 5.7[degree]) 
The granularity with which to check for collisions for rotations in radians  
シミュレートの際の角度の間隔．



~/YmgLPROS/use_dwa (bool, default: False)
Use dynamic window approach to constrain sampling velocities to small window.
プランニングにdwaを使用するか．
以下のpath_tolerance, obstacle_toleranceはymg_sampling_plannerのパラメータ.
path_distance_bias, goal_distance_bias, occdist_scale, local_goal_distanceはdwaのパラメータ．

~/YmgLPROS/path_tolerance (double[m], default: 0.1)
The tolerance between global path and endpoint of the simulated local path.
[ymg_sampling_planner]ローカルプランの終端とグローバルパスの距離の許容値．
距離はglobal pathの点との距離が算出されるため，距離の精度はglobal pathの点密度に依存することに注意．
（点密度が粗く，path_toleranceが小さい場合には有効なパスが引かれない可能性がある．）

~/YmgLPROS/obstacle_tolerance (int, default: 253)
The maximum cost of the cell which the path can be drawn.
[ymg_sampling_planner]シミュレートされた経路上にこの値より大きいコストがあった場合，その経路は棄却される．
costmap_2dのコストは，254がcritical(definitely in collision)に設定されている．詳しくはROS wiki costmap_2dを参照．



~/YmgLPROS/path_distance_bias (double, default: 32.0)  
The weight for the path distance part of the cost function  
[dwa]コスト計算の際のパスとの距離の重み．

~/YmgLPROS/goal_distance_bias (double, default: 24.0)  
The weight for the goal distance part of the cost function  
[dwa]コスト計算の際のローカルゴールとの距離の重み．

~/YmgLPROS/occdist_scale (double, default: 0.01)  
The weight for the obstacle distance part of the cost function  
[dwa]コスト計算の際の障害物を避ける重み．

~/YmgLPROS/local_goal_distance (double, default: 2.0)
The distance to the local goal
[dwa]ローカルゴールを現在位置から(グローバルパス上の)どのくらい先に置くか．



~/YmgLPROS/scoring_point_offset_x (double[m], default: 0.3)  
The distance from the center point of the robot to place an additional scoring point, in meters  
グローバルパスとの距離，ローカルゴールまでの距離からコストを算出する際にロボットの前方に追加の点を置くことができる．
その際，算出されるコストはロボットの中心と追加の点で算出されたコストの平均値となる．
設定値を大きくし過ぎるとゴールにたどり着くのが困難になる可能性があるため，xy_goal_toleranceを大きめに取ること．

~/YmgLPROS/obstacle_stop_margine (double[m], default: 0.5)  
障害物の手前で停止する際にどれだけ余裕を持って停止するか．

~/YmgLPROS/scaling_speed (double[m/s], default: 0.25)  
The absolute value of the velocity at which to start scaling the robot's footprint, in m/s  
速度の閾値．ロボットの並進速度の絶対値がこの値を超えると，footprintがスケーリングされる．

~/YmgLPROS/max_scaling_factor (double, default: 1.5)  
The maximum factor to scale the robot's footprint by  
footprintのスケーリングの割合の上限．
デフォルト値を用い，ロボットがmax_trans_velで走行するとfootprintが1.5倍にスケーリングされる．

velocity_x || 0.0 | ~ | scaling_speed | ~~~~~~~~~~~~~~~~~~ | max_vel_x
     scale || 1.0 | ~ |      1.0      | gradually inclease | max_scaling_factor

~/YmgLPROS/vx_samples (int, default: 4)  
The number of samples to use when exploring the x velocity space  
ロボットの並進速度のシミュレートの数．

~/YmgLPROS/vth_samples (int, default: 20)  
The number of samples to use when exploring the theta velocity space  
ロボットの回転角速度のシミュレートの数．

~/YmgLPROS/restore_defaults (bool, default: False)  
Restore to the original configuration  
全パラメータをデフォルト値にセットする.
