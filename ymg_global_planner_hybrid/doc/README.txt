### Original global planner



### Published Topics
~/YmgGPHybROS/ymggp_plan (nav_msgs/Path)  
ymggpで算出されるグローバルパス．
~/YmgGPHybROS/navfn_plan (nav_msgs/Path)  
navfnで算出されるグローバルパス．ymggpを用いているときは空の配列となる．



### Subscribed Topics
~/YmgGPHybROS/reset_flag (std_msgs/Empty)  
このフラグを受け取ると，グローバルパスが空になる．
navfnモードに切り替わっているときは，強制的にymggpモードに切り替わる．

~/YmgGPHybROS/use_ymggp_force (std_msgs/Int32)  
値の内容は0,1,2のみ．
この値を設定することで，スタック時にnavfnモードに切り替わるのを防ぐことができる．
navfnモードの状態で1を受け取るとnavfnのゴールに到達した後ymggpモードに切り替わる．
2を受け取ると，現在navfnモードでも強制的にymggpモードに切り替わる．



### Parameters
(move base param) ~/base_global_planner: "ymggp/YmgGPHybROS"  

~/YmgGPROS/path_resolution (double[points/m], default: 10.0)
点の集合で表されるグローバルパスの密度．

~/YmgGPROS/stuck_timeout (double[sec], default: 10.0)
When the robot stops while this time, this planner changes algorithm to dijkstra.
この秒数間ロボットがスタックしたら，グローバルパスの算出アルゴリズムをnavfn(dijkstra法)に切り替える．

~/YmgGPROS/stuck_vel (double[m/s], default: 0.05)
ロボットの速度の絶対値がこの速度以下となった場合にロボットがスタックしたとみなす．
マイナスにセットした場合は，スタック判定を行わない．(navfnに切り替わらない)

~/YmgGPROS/stuck_rot_vel (double[rad/s], default: -1.0)
ロボットの角速度の絶対値がこの速度以下となった場合にロボットがスタックしたとみなす．
マイナスにセットした場合は角速度はスタック判定に用いられない．

~/YmgGPROS/navfn_goal_dist (double[m], default: 5.0)
navfnにアルゴリズムが切り替わった時，現在地から(ymggpで算出されたパス上の)何メートル先にnavfnのゴールを置くか．

~/YmgGPROS/recovery_dist (double[m], default: 2.0)
navfnモード時にnavfnのゴールとの距離がこの値以下となった時，ymggpモードに切り替わる．

~/YmgGPROS/goal_tolerance (double[m], default: 0.3)
ロボットとゴールの位置がこの距離以下となって止まっている時，ロボットはスタック判定を行わない．(navfnモードに切り替わらない)
