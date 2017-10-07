#ifndef UTIL_FUNCTIONS_H_
#define UTIL_FUNCTIONS_H_

#include <ros/ros.h>
#include <float.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ymglp {

class UtilFcn {

	public:

		static double calcSqDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
		static double calcDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
		static int getClosestIndexOfPath(const geometry_msgs::PoseStamped& pose,
				const std::vector<geometry_msgs::PoseStamped>& path);
		static int getClosestIndexOfPath(const tf::Stamped<tf::Pose>& pose,
				const std::vector<geometry_msgs::PoseStamped>& path);
		static double getDirectionError(double base, double comp);
		
		UtilFcn();
		void setInfo(const geometry_msgs::PoseStamped& pose,
				const std::vector<geometry_msgs::PoseStamped>& plan);
		void setInfo(const tf::Stamped<tf::Pose>& pose,
				const std::vector<geometry_msgs::PoseStamped>& plan);
		int getNearestIndex();
		void getShortenedPlan(double distance, std::vector<geometry_msgs::PoseStamped>& shortened_plan);
		double getRobotDirection();
		double getNearestDirection();
		double getDistance();
		double getDirectionError();

	private:
		void resetFlag();
		geometry_msgs::PoseStamped pose_;
		std::vector<geometry_msgs::PoseStamped> plan_;

		bool has_nearest_index_, has_nearest_direction_;
		int nearest_index_;
		double nearest_direction_;

};   // class utilfcn

}   // namespace ymglp

#endif
