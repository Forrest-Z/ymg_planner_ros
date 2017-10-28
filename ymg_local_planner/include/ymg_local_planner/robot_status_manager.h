#ifndef ROBOT_STATUS_MANAGER_H_
#define ROBOT_STATUS_MANAGER_H_

#include <ros/ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace ymglp {

class RobotStatusManager {

	public:
		RobotStatusManager();
		enum RobotStatus {MOVING, STOPPED, GOAL_REACHED};
		void updateRobotStatus();
		void setRobotStatus(RobotStatus status);
		ros::Duration getTimeWhileStopped();

	private:
		double stuck_vel_, stuck_rot_vel_;
		ros::Time stop_time_;
		RobotStatus robot_status_;
		base_local_planner::OdometryHelperRos odom_helper_;
		std::string robotStatusToString(RobotStatus status);

		ros::Subscriber movebase_status_sub_;
		void movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

};   // class RobotStatusManager

} // namespace ymglp

#endif
