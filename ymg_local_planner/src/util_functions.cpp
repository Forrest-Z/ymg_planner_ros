#include <ros/ros.h>
#include <float.h>
#include <geometry_msgs/PoseStamped.h>
#include <ymg_local_planner/util_functions.h>
#include <tf/tf.h>

namespace ymglp {

UtilFcn::UtilFcn()
{/*{{{*/
	resetFlag();
}/*}}}*/

double UtilFcn::calcSqDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return dx*dx + dy*dy;
}/*}}}*/

double UtilFcn::calcDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return sqrt(dx*dx + dy*dy);
}/*}}}*/

int UtilFcn::getClosestIndexOfPath(const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	int closest_index = -1;
	double sq_dist, min_sq_dist = DBL_MAX;

	for (int i=0; i<path.size(); ++i) {
		sq_dist = calcSqDist(pose, path[i]);
		if (sq_dist < min_sq_dist) {
			min_sq_dist = sq_dist;
			closest_index = i;
		}
	}

	return closest_index;
}/*}}}*/

int UtilFcn::getClosestIndexOfPath(const tf::Stamped<tf::Pose>& pose, const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	geometry_msgs::PoseStamped p;
	tf::poseStampedTFToMsg(pose, p);
	// p.pose.position.x = pose.getOrigin().getX();
	// p.pose.position.y = pose.getOrigin().getY();

	return getClosestIndexOfPath(p, path);
}/*}}}*/

void UtilFcn::setInfo(const geometry_msgs::PoseStamped& pose,
		const std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	pose_ = pose;
	plan_ = plan;
	resetFlag();
}/*}}}*/

void UtilFcn::setInfo(const tf::Stamped<tf::Pose>& pose,
		const std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	geometry_msgs::PoseStamped p;
	tf::poseStampedTFToMsg(pose, p);
	setInfo(p, plan);
}/*}}}*/

int UtilFcn::getNearestIndex()
{/*{{{*/
	if (has_nearest_index_) {
		return nearest_index_;
	}

	nearest_index_ = -10;
	double sq_dist, min_sq_dist = DBL_MAX;

	for (int i=0; i<plan_.size(); ++i) {
		sq_dist = calcSqDist(pose_, plan_[i]);
		if (sq_dist < min_sq_dist) {
			min_sq_dist = sq_dist;
			nearest_index_ = i;
		}
	}

	has_nearest_index_ = true;
	return nearest_index_;
}/*}}}*/

void UtilFcn::getShortenedPlan(double distance, std::vector<geometry_msgs::PoseStamped>& shortened_plan)
{/*{{{*/
	int start_index = getNearestIndex();
	if (start_index < 0) {
		ROS_INFO("cannot get shorten plan.");
		return;
	}

	shortened_plan.clear();
	shortened_plan.push_back(plan_[start_index]);
	double now_distance = 0.0;
	for (int i=start_index+1; i<plan_.size(); ++i) {
		now_distance += UtilFcn::calcDist(plan_[i-1], plan_[i]);
		shortened_plan.push_back(plan_[i]);
		if (distance < now_distance) {
			return;
		}
	}
}/*}}}*/

double UtilFcn::getRobotDirection()
{/*{{{*/
	return tf::getYaw(pose_.pose.orientation);
}/*}}}*/

double UtilFcn::getNearestDirection()
{/*{{{*/
	if (has_nearest_direction_) {
		return nearest_direction_;
	}

	if (getNearestIndex() == plan_.size()-1) {
		nearest_direction_ = tf::getYaw(plan_.back().pose.orientation);
	}
	else {
		nearest_direction_ = atan2(plan_[nearest_index_+1].pose.position.y - plan_[nearest_index_].pose.position.y,
				plan_[nearest_index_+1].pose.position.x - plan_[nearest_index_].pose.position.x);
	}

	return nearest_direction_;
}/*}}}*/

double UtilFcn::getDistance()
{/*{{{*/
	return calcDist(pose_, plan_[getNearestIndex()]);
}/*}}}*/

double UtilFcn::getDirectionError()
{/*{{{*/
	// pi to -pi minus pi to -pi  ===>  2*pi to -2*pi
	double error = getRobotDirection() - getNearestDirection();
	return atan2(sin(error), cos(error));
}/*}}}*/

double UtilFcn::getDirectionError(double base, double comp)
{/*{{{*/
	double error = comp - base;
	return atan2(sin(error), cos(error));
}/*}}}*/

void UtilFcn::resetFlag()
{/*{{{*/
	has_nearest_index_ = false;
	has_nearest_direction_ = false;
}/*}}}*/

}   // namespace ymglp
