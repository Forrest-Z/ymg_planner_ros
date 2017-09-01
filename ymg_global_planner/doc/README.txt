Waypoint を直線で結ぶだけのシンプルな Global Planner．


<Published Topics>
~/YmgGPROS/plan (nav_msgs/Path)  


<Parameters>
(move base param) ~/base_global_planner: "ymggp/YmgGPROS"  
~/YmgGPROS/path_resolution (double[points/m], default: 10.0)
   算出されるグローバルパスの細かさ．
