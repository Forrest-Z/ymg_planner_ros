#include <ymg_local_planner/obstacle_cost_function_kai.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

ObstacleCostFunctionKai::ObstacleCostFunctionKai(
		costmap_2d::Costmap2D* costmap, double forward_point_dist, double sim_granularity)
	/*{{{*/
	: costmap_(costmap), forward_point_dist_(forward_point_dist),
	sim_granularity_(sim_granularity), scaling_flag_(true)
	{
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
}/*}}}*/

ObstacleCostFunctionKai::~ObstacleCostFunctionKai()
{/*{{{*/
  if (world_model_ != NULL) {
    delete world_model_;
  }
}/*}}}*/

void ObstacleCostFunctionKai::setParams(double max_vel_abs, double max_scaling_factor, double scaling_speed)
{/*{{{*/
  // TODO: move this to prepare if possible
  max_vel_abs_ = max_vel_abs;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}/*}}}*/

void ObstacleCostFunctionKai::setFootprint(std::vector<geometry_msgs::Point> footprint_spec)
{/*{{{*/
  footprint_spec_ = footprint_spec;
}/*}}}*/

bool ObstacleCostFunctionKai::prepare()
{/*{{{*/
  return true;
}/*}}}*/

// this function gets maximum cost in the trajectory and returns scaled value.
double ObstacleCostFunctionKai::scoreTrajectory(Trajectory &traj)
{/*{{{*/

	if (getScale() == 0.0) {
		return 0.0;
	}
	
	double scale = 1.0;
	if (scaling_flag_)
		scale = getScalingFactor(traj, scaling_speed_, max_vel_abs_, max_scaling_factor_);

  if (footprint_spec_.size() == 0) {
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  double px, py, pth;
  double cost = 0.0;
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
				scale, footprint_spec_, costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;
    }

		cost = std::max(cost, f_cost);   // changed   cost = f_cost ->
  }

	// calc forward point score
	if (!ymglp::UtilFcn::isZero(traj.xv_)) {
		traj.getEndpoint(px, py, pth);
		int additional_points = forward_point_dist_ / sim_granularity_;

		int sign = 1;
		if (traj.xv_<0.0) sign = -1;

		double len;
		for (int i=1; i<=additional_points; ++i) {
			len = sign * i * sim_granularity_;
			double f_cost = footprintCost(px+len*cos(pth), py+len*sin(pth), pth,
					scale, footprint_spec_, costmap_, world_model_);

			if(f_cost < 0){
					return f_cost;
			}

			cost = std::max(cost, f_cost);   // changed   cost = f_cost ->
		}
	}
	else {
		ROS_INFO("velocity is zero");
	}

  return cost * getScale();
}/*}}}*/

double ObstacleCostFunctionKai::scoreTrajectory(Trajectory &traj, bool scaling_flag)
{/*{{{*/
	bool default_scaling_flag = scaling_flag_;
	if (scaling_flag) scaling_flag_ = true;
	else              scaling_flag_ = false;

	double score = scoreTrajectory(traj);

	scaling_flag_ = default_scaling_flag;

	return score;
}/*}}}*/

double ObstacleCostFunctionKai::getScalingFactor(Trajectory &traj, double scaling_speed, double max_vel_abs, double max_scaling_factor)
{/*{{{*/
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_vel_abs - scaling_speed);
    scale = (max_scaling_factor - 1.0) * ratio + 1.0;
  }
  return scale;
}/*}}}*/

double ObstacleCostFunctionKai::footprintCost (
    const double& x, const double& y, const double& th, double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model)
{/*{{{*/

  //check if the footprint is legal
  // TODO: Cache inscribed radius
	double footprint_cost;
	
	if (1.0 < scale) {
		std::vector<geometry_msgs::Point> scaled_footprint_spec = footprint_spec;
		for (int i=0; i<scaled_footprint_spec.size(); ++i) {
			scaled_footprint_spec[i].x *= scale;
			scaled_footprint_spec[i].y *= scale;
		}
		footprint_cost = world_model->footprintCost(x, y, th, scaled_footprint_spec);
	}
	else {
		footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);
	}

  if (footprint_cost < 0) {
    return -6.0;
  }
  unsigned int cell_x, cell_y;

  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
    return -7.0;
  }

  double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

  return occ_cost;
}/*}}}*/

} /* namespace base_local_planner */
