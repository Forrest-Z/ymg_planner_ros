#ifndef UTIL_FUNCTION_H_
#define UTIL_FUNCTION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

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
	if (path.empty())
		return -1;

	int closest_index = 0;
	double sq_dist, min_sq_dist = calcSqDist(pose, path[0]);

	for (int i=1; i<path.size(); ++i) {
		sq_dist = calcSqDist(pose, path[i]);
		if (sq_dist < min_sq_dist) {
			min_sq_dist = sq_dist;
			closest_index = i;
		}
	}

	return closest_index;
}/*}}}*/

};   // namespace ymglp

#endif
