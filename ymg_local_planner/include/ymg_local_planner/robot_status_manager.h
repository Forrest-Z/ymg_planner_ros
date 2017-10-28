#ifndef ROBOT_STATUS_MANAGER_H_
#define ROBOT_STATUS_MANAGER_H_

#include <ros/ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace ymglp {

class RobotStatusManager {

	public:
		RobotStatusManager();
		void setStoppedVel(double trans_stopped_vel, double rot_stopped_vel);
		void clearStoppedTime();
		ros::Duration getTimeWhileStopped();
		std::string getRobotStatusString();

	private:
		double trans_stopped_vel_, rot_stopped_vel_;
		ros::Time stopped_time_;
		bool is_goal_reached_, is_robot_moving_;

		void updateRobotStatus(double robot_v, double robot_w);

		ros::Subscriber movebase_status_sub_, cmd_vel_sub_;
		void movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
		void cmdVelCallback (const geometry_msgs::Twist& msg);

};   // class RobotStatusManager

} // namespace ymglp

#endif
