### Original global planner
ymg_global_plannerとBaseGlobalPlannerのHybrid Global Planner．
普段はymg_global_plannerで走行し，ロボットがスタックした時のみBaseGlobalPlannerにてプランニングを行う．


### Published Topics
~/YmgGPBGP/ymggp_plan (nav_msgs/Path)  
ymggpで算出されるグローバルパス．
~/YmgGPBGP/bgp_plan (nav_msgs/Path)  
BaseGlobalPlanner(以下BGP)で算出されるグローバルパス．ymggpを用いているときは空の配列となる．



### Subscribed Topics
~/YmgGPBGP/reset_flag (std_msgs/Empty)  
このフラグを受け取ると，グローバルパスが空になる．
BGPモードに切り替わっているときは，強制的にymggpモードに切り替わる．

~/YmgGPBGP/use_ymggp_force (std_msgs/Int32)  
値の内容は0,1,2のみ．
この値を設定することで，スタック時にBGPモードに切り替わるのを防ぐことができる．
BGPモードの状態で1を受け取るとBGPのゴールに到達した後ymggpモードに切り替わる．
2を受け取ると，現在BGPモードでも強制的にymggpモードに切り替わる．



### Parameters
(move base param) ~/base_global_planner: "ymggp_bgp/YmgGPBGP"  

~/YmgGPBGP/path_granularity (double[m], default: 0.05)
点の集合で表されるグローバルパスのきめ細かさ．

~/YmgGPBGP/stuck_timeout (double[sec], default: 10.0)
When the robot stops while this time, this planner changes algorithm to dijkstra.
この秒数間ロボットがスタックしたら，グローバルパスプランをBGPに切り替える．

~/YmgGPBGP/stuck_vel (double[m/s], default: 0.05)
ロボットの速度の絶対値がこの速度以下となった場合にロボットがスタックしたとみなす．
マイナスにセットした場合は，スタック判定を行わない．(BGPに切り替わらない)

~/YmgGPBGP/stuck_rot_vel (double[rad/s], default: -1.0)
ロボットの角速度の絶対値がこの速度以下となった場合にロボットがスタックしたとみなす．
マイナスにセットした場合は角速度はスタック判定に用いられない．

~/YmgGPBGP/bgp_goal_dist (double[m], default: 5.0)
BGPにアルゴリズムが切り替わった時，現在地から(ymggpで算出されたパス上の)何メートル先にBGPのゴールを置くか．

~/YmgGPBGP/recovery_dist (double[m], default: 2.0)
BGPモード時にBGPのゴールとの距離がこの値以下となった時，ymggpモードに切り替わる．

~/YmgGPBGP/goal_tolerance (double[m], default: 0.3)
ロボットとゴールの位置がこの距離以下となって止まっている時，ロボットはスタック判定を行わない．(BGPモードに切り替わらない)

~/YmgGPBGP/clear_plan_when_goal_reached (bool, default: true)
ロボットがゴールに到達した時，ymggpのglobal_planをclearするかどうか．
