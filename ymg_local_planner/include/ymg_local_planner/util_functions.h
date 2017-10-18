#ifndef UTIL_FUNCTIONS_H_
#define UTIL_FUNCTIONS_H_

#include <ros/ros.h>
#include <float.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <base_local_planner/trajectory.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ymglp {

class UtilFcn {

	public:

		static inline bool isZero(double num)
		{/*{{{*/
			return fabs(num) < DBL_MIN*128;
		}/*}}}*/

		static inline double calcSqDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
		{/*{{{*/
			double dx = p1.pose.position.x - p2.pose.position.x;
			double dy = p1.pose.position.y - p2.pose.position.y;
			return dx*dx + dy*dy;
		}/*}}}*/

		static inline double calcDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
		{/*{{{*/
			double dx = p1.pose.position.x - p2.pose.position.x;
			double dy = p1.pose.position.y - p2.pose.position.y;
			return sqrt(dx*dx + dy*dy);
		}/*}}}*/

		static inline int getClosestIndexOfPath(
				const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::PoseStamped>& path)
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

		static inline int getClosestIndexOfPath(
				const tf::Stamped<tf::Pose>& pose, const std::vector<geometry_msgs::PoseStamped>& path)
		{/*{{{*/
			geometry_msgs::PoseStamped p;
			tf::poseStampedTFToMsg(pose, p);
			// p.pose.position.x = pose.getOrigin().getX();
			// p.pose.position.y = pose.getOrigin().getY();

			return getClosestIndexOfPath(p, path);
		}/*}}}*/

		static inline double getDirectionError(double base, double comp)
		{/*{{{*/
			double error = comp - base;
			return atan2(sin(error), cos(error));
		}/*}}}*/

		static inline double getDirectionErrorCCW(double base, double comp)
		{/*{{{*/
			double error = comp - base;
			if (error < 0.0) {
				error += 2*M_PI;
			}

			return error;
		}/*}}}*/

		static inline double getDirectionErrorCW(double base, double comp)
		{/*{{{*/
			double error = comp - base;
			if (0,0 < error) {
				error -= 2*M_PI;
			}
			return fabs(error);
		}/*}}}*/


		UtilFcn();
		void setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
		void setPose(const geometry_msgs::PoseStamped& pose);
		void setPose(const tf::Stamped<tf::Pose>& pose);
		void setScoringPointOffsetX(double scoring_point_offset_x);
		void setSearchDist(double max_dist);

		geometry_msgs::PoseStamped getPoseStamped();
		int getNearestIndex();
		void getShortenedPlan(double distance, std::vector<geometry_msgs::PoseStamped>& shortened_plan);
		double getRobotDirection();
		double getNearestDirection();
		double getDirectionError();
		double getPathDist();
		double getPathDist(double x, double y);
		double getPathDistHQ(double x, double y);
		double getForwardPointPathDist(bool back_mode = false);
		double scoreTrajDist(base_local_planner::Trajectory& traj, bool back_mode = false);
		void getLocalGoal(double dist, Eigen::Vector2d& goal);
		void tfGlobal2Robot(Eigen::Vector2d global, Eigen::Vector2d robot);

	private:
		void resetFlag();
		geometry_msgs::PoseStamped pose_;
		std::vector<geometry_msgs::PoseStamped> plan_;
		double scoring_point_offset_x_;

		bool has_nearest_index_, has_nearest_direction_;
		int nearest_index_, max_search_index_;
		double nearest_direction_;

};   // class utilfcn

}   // namespace ymglp

#endif
