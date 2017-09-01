#ifndef UTIL_FUNCTION_H_
#define UTIL_FUNCTION_H_

namespace ymglp {

inline double sqDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return sqrt(dx*dx + dy*dy);
}/*}}}*/

};   // namespace ymglp

#endif
