#ifndef UTIL_FUNCTION_H_
#define UTIL_FUNCTION_H_

#include <ros/ros.h>
#include <float.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ymglp {

inline double calcSqDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return dx*dx + dy*dy;
}/*}}}*/

inline double calcDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return sqrt(dx*dx + dy*dy);
}/*}}}*/

int getClosestIndexOfPath(const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::PoseStamped>& path)
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

int getClosestIndexOfPath(const tf::Stamped<tf::Pose>& pose, const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	geometry_msgs::PoseStamped p;
	p.pose.position.x = pose.getOrigin().getX();
	p.pose.position.y = pose.getOrigin().getY();

	return getClosestIndexOfPath(p, path);
}/*}}}*/

// XXX new function and has not tested yet.
bool isZero(double x)
{/*{{{*/
	return (0<=x && x<DBL_MIN*100);
}/*}}}*/

};   // namespace ymglp

#endif
