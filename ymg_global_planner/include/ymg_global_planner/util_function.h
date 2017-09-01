#ifndef UTIL_FUNCTION_H_
#define UTIL_FUNCTION_H_

#include <geometry_msgs/PoseStamped.h>

namespace ymggp {

inline double sqDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return sqrt(dx*dx + dy*dy);
}/*}}}*/

inline int getClosestIndexOfPath(const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	int closest_index = -1;
	double dist, min_dist = -1.0;
	for (int i=0; i<path.size(); ++i) {
		dist = sqDistance(pose, path[i]);
		if (dist < min_dist || min_dist < 0) {
			min_dist = dist;
			closest_index = i;
		}
	}
	return closest_index;
}/*}}}*/

};   // namespace ymglp

#endif
