#ifndef UTIL_FUNCTIONS_H_
#define UTIL_FUNCTIONS_H_

#include <ros/ros.h>
#include <float.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ymglp {

namespace utilfcn {

double calcSqDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

double calcDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

int getClosestIndexOfPath(const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::PoseStamped>& path);

int getClosestIndexOfPath(const tf::Stamped<tf::Pose>& pose, const std::vector<geometry_msgs::PoseStamped>& path);

bool isZero(double x);

}   // namespace utilfcn

}   // namespace ymglp

#endif
