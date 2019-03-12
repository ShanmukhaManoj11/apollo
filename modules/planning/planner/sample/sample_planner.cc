/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/planner/sample/sample_planner.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/collision_checker.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/lattice/behavior/path_time_graph.h"
#include "modules/planning/lattice/behavior/prediction_querier.h"
#include "modules/planning/lattice/trajectory_generation/backup_trajectory_generator.h"
#include "modules/planning/lattice/trajectory_generation/lattice_trajectory1d.h"
#include "modules/planning/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/lattice/trajectory_generation/trajectory_combiner.h"
#include "modules/planning/lattice/trajectory_generation/trajectory_evaluator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;
using apollo::common::time::Clock;

/*
method to convert reference points on reference line to a vector of path points
*/
std::vector<PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint>& ref_points){
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
      cartesian_state.path_point().x(), cartesian_state.path_point().y(),
      cartesian_state.v(), cartesian_state.a(),
      cartesian_state.path_point().theta(),
      cartesian_state.path_point().kappa(), ptr_s, ptr_d);
}

Status SamplePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame,
                            ADCTrajectory* ptr_computed_trajectory) {
  return Status::OK();
}

Status SamplePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
	reference_line_info->set_is_on_reference_line();
	//1. reference points to vector of PathPoints
	auto path_points = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
		reference_line_info->reference_line().reference_points()));
	//2. match planning_init_point to the vector of PathPoints
	PathPoint matched_point=PathMatcher::MatchToPath(*path_points,planning_init_point.path_point().x(),
		planning_init_point.path_point().y());
	//3. convert planning_init_point to frenet frame according to matched point
	std::array<double,3> init_s;
	std::array<double,3> init_d;
	ComputeInitFrenetState(matched_point,planning_init_point,&init_s,&init_d);
	//4. get obstacles on reference line and compute target frenet coordinates
	std::array<double,3> target_s;
	std::array<double,3> target_d={0.0,0.0,0.0};
	
	//5. compute JMTs (5th order polynomials) for s and d independently from init and target frenet coordinates

	//6. convert frenet s,d points from JMTs back to xy coordinates
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
