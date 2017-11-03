#ifndef ROBOT_STATUS_MANAGER_H_
#define ROBOT_STATUS_MANAGER_H_

#include <ros/ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace ymglp {

class RobotStatusManagerMk2 {

	public:
		RobotStatusManagerMk2();
		void setParameters(double trans_stopped_vel, double rot_stopped_vel, double period);
		void clearStatus();
		bool isStack();

	private:
		double trans_stopped_vel_, rot_stopped_vel_, period_;

		std::vector<double> v_list_, w_list_;
		std::vector<ros::Time> timestamp_;

		bool getCmdVelBar(double& v_bar, double& w_bar);

		ros::Subscriber movebase_status_sub_, cmd_vel_sub_;
		void movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
		void cmdVelCallback (const geometry_msgs::Twist& msg);

};   // class RobotStatusManager

} // namespace ymglp

#endif
