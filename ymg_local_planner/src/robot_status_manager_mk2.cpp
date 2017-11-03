#include <ymg_local_planner/robot_status_manager_mk2.h>

namespace ymglp {

RobotStatusManagerMk2::RobotStatusManagerMk2()
: /*{{{*/
	trans_stopped_vel_(-1.0), rot_stopped_vel_(-1.0), period_(10.0)
{
	ros::NodeHandle nh;
	movebase_status_sub_ = nh.subscribe("/move_base/status", 100, &RobotStatusManagerMk2::movebaseStatusCallback, this);
	cmd_vel_sub_ = nh.subscribe("/cmd_vel", 100, &RobotStatusManagerMk2::cmdVelCallback, this);
}/*}}}*/

void RobotStatusManagerMk2::setParameters(double trans_stopped_vel, double rot_stopped_vel, double period)
{/*{{{*/
	trans_stopped_vel_ = trans_stopped_vel;
	rot_stopped_vel_ = rot_stopped_vel;
	period_ = period;
}/*}}}*/

void RobotStatusManagerMk2::clearStatus()
{/*{{{*/
	v_list_.clear();
	w_list_.clear();
	timestamp_.clear();
}/*}}}*/

bool RobotStatusManagerMk2::isStack()
{/*{{{*/
	double v_bar, w_bar;
	if (!getCmdVelBar(v_bar, w_bar)) {
		return false;
	}
	ROS_INFO("v_bar, w_bar = %f, %f", v_bar, w_bar);

	if (trans_stopped_vel_ < 0.0) {
		return false;
	}
	else {
		if (rot_stopped_vel_ < 0.0) {
			return v_bar < trans_stopped_vel_;
		}
		else {
			return (v_bar < trans_stopped_vel_ && w_bar < rot_stopped_vel_);
		}
	}

	return false;
}/*}}}*/

bool RobotStatusManagerMk2::getCmdVelBar(double& v_bar, double& w_bar)
{/*{{{*/
	if (timestamp_.empty()) {
		return false;
	}

	ros::Time now_time = ros::Time::now();
	if (now_time - timestamp_[0] < ros::Duration(period_)) {
		return false;
	}

	int erase_index=timestamp_.size()-1;
	for (; 0<=erase_index; --erase_index) {
		if (ros::Duration(period_) < now_time - timestamp_[erase_index]) {
			break;
		}
	}

	v_list_.erase(v_list_.begin(), v_list_.begin()+erase_index);
	w_list_.erase(w_list_.begin(), w_list_.begin()+erase_index);
	timestamp_.erase(timestamp_.begin(), timestamp_.begin()+erase_index);

	double v_sum = 0.0, w_sum = 0.0;
	for (int i=0; i<timestamp_.size(); ++i) {
		v_sum += fabs(v_list_[i]);
		w_sum += fabs(w_list_[i]);
	}
	v_bar = v_sum / timestamp_.size();
	w_bar = v_sum / timestamp_.size();

	return true;
}/*}}}*/

void RobotStatusManagerMk2::movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{/*{{{*/
	if (msg->status_list.empty())
		return;

	actionlib_msgs::GoalStatus status = msg->status_list[0];

	if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
		clearStatus();
	}
}/*}}}*/

void RobotStatusManagerMk2::cmdVelCallback (const geometry_msgs::Twist& msg)
{/*{{{*/
	v_list_.push_back(msg.linear.x);
	w_list_.push_back(msg.angular.z);
	timestamp_.push_back(ros::Time::now());
}/*}}}*/

}// namespace ymglp
