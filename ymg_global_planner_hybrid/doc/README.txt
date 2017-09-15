### Original global planner



### Published Topics
~/YmgGPHybROS/ymggp_plan (nav_msgs/Path)  
ymggp$B$G;;=P$5$l$k%0%m!<%P%k%Q%9!%(B
~/YmgGPHybROS/navfn_plan (nav_msgs/Path)  
navfn$B$G;;=P$5$l$k%0%m!<%P%k%Q%9!%(Bymggp$B$rMQ$$$F$$$k$H$-$O6u$NG[Ns$H$J$k!%(B



### Subscribed Topics
~/YmgGPHybROS/reset_flag (std_msgs/Empty)  
$B$3$N%U%i%0$r<u$1<h$k$H!$%0%m!<%P%k%Q%9$,6u$K$J$k!%(B
navfn$B%b!<%I$K@Z$jBX$o$C$F$$$k$H$-$O!$6/@)E*$K(Bymggp$B%b!<%I$K@Z$jBX$o$k!%(B

~/YmgGPHybROS/use_ymggp_force (std_msgs/Int32)  
$BCM$NFbMF$O(B0,1,2$B$N$_!%(B
$B$3$NCM$r@_Dj$9$k$3$H$G!$%9%?%C%/;~$K(Bnavfn$B%b!<%I$K@Z$jBX$o$k$N$rKI$0$3$H$,$G$-$k!%(B
navfn$B%b!<%I$N>uBV$G(B1$B$r<u$1<h$k$H(Bnavfn$B$N%4!<%k$KE~C#$7$?8e(Bymggp$B%b!<%I$K@Z$jBX$o$k!%(B
2$B$r<u$1<h$k$H!$8=:_(Bnavfn$B%b!<%I$G$b6/@)E*$K(Bymggp$B%b!<%I$K@Z$jBX$o$k!%(B



### Parameters
(move base param) ~/base_global_planner: "ymggp/YmgGPHybROS"  

~/YmgGPROS/path_resolution (double[points/m], default: 10.0)
$BE@$N=89g$GI=$5$l$k%0%m!<%P%k%Q%9$NL)EY!%(B

~/YmgGPROS/stuck_timeout (double[sec], default: 10.0)
When the robot stops while this time, this planner changes algorithm to dijkstra.
$B$3$NIC?t4V%m%\%C%H$,%9%?%C%/$7$?$i!$%0%m!<%P%k%Q%9$N;;=P%"%k%4%j%:%`$r(Bnavfn(dijkstra$BK!(B)$B$K@Z$jBX$($k!%(B

~/YmgGPROS/stuck_vel (double[m/s], default: 0.05)
$B%m%\%C%H$NB.EY$N@dBPCM$,$3$NB.EY0J2<$H$J$C$?>l9g$K%m%\%C%H$,%9%?%C%/$7$?$H$_$J$9!%(B
$B%^%$%J%9$K%;%C%H$7$?>l9g$O!$%9%?%C%/H=Dj$r9T$o$J$$!%(B(navfn$B$K@Z$jBX$o$i$J$$(B)

~/YmgGPROS/stuck_rot_vel (double[rad/s], default: -1.0)
$B%m%\%C%H$N3QB.EY$N@dBPCM$,$3$NB.EY0J2<$H$J$C$?>l9g$K%m%\%C%H$,%9%?%C%/$7$?$H$_$J$9!%(B
$B%^%$%J%9$K%;%C%H$7$?>l9g$O3QB.EY$O%9%?%C%/H=Dj$KMQ$$$i$l$J$$!%(B

~/YmgGPROS/navfn_goal_dist (double[m], default: 5.0)
navfn$B$K%"%k%4%j%:%`$,@Z$jBX$o$C$?;~!$8=:_CO$+$i(B(ymggp$B$G;;=P$5$l$?%Q%9>e$N(B)$B2?%a!<%H%k@h$K(Bnavfn$B$N%4!<%k$rCV$/$+!%(B

~/YmgGPROS/recovery_dist (double[m], default: 2.0)
navfn$B%b!<%I;~$K(Bnavfn$B$N%4!<%k$H$N5wN%$,$3$NCM0J2<$H$J$C$?;~!$(Bymggp$B%b!<%I$K@Z$jBX$o$k!%(B
