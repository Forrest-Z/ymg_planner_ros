#include <ymg_local_planner/obstacle_cost_function_kai.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

ObstacleCostFunctionKai::ObstacleCostFunctionKai(costmap_2d::Costmap2D* costmap,
		double additional_sim_time, double forward_point_dist, double sim_granularity)
	/*{{{*/
	: costmap_(costmap), sum_scores_(false), additional_sim_time_(additional_sim_time),
		forward_point_dist_(forward_point_dist), sim_granularity_(sim_granularity)
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


void ObstacleCostFunctionKai::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed)
{/*{{{*/
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
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

// XXX new function and has not tested yet.
bool ObstacleCostFunctionKai::isZero(double x)
{/*{{{*/
	return (0<=x && x<DBL_MIN*128);
}/*}}}*/

double ObstacleCostFunctionKai::scoreTrajectory(Trajectory &traj)
{/*{{{*/
	double additional_length = 0.0;

	if (0.0 < additional_sim_time_) {
		additional_length += traj.xv_ * additional_sim_time_;
	}

	// XXX added but has not tested yet.
	if (!isZero(traj.xv_)) {
		additional_length += forward_point_dist_;
	}

	double ep_x, ep_y, ep_th;
	traj.getEndpoint(ep_x, ep_y, ep_th);
	int additional_points = additional_length / sim_granularity_;
	for (int i=0; i<additional_points; ++i) {
		double len = (i+1) * sim_granularity_;
		traj.addPoint(ep_x+len*cos(ep_th), ep_y+len*sin(ep_th), ep_th);
	}

  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        scale, footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;
    }

    if(sum_scores_)
        cost +=  f_cost;
    else
        cost = std::max(cost, f_cost);   // changed   cost = f_cost ->
  }
  return cost;
}/*}}}*/

double ObstacleCostFunctionKai::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor)
{/*{{{*/
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
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
	ROS_INFO("footprint_cost = %f", footprint_cost);


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
