// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FILTER_PREDICTED_OBJECTS_HPP_
#define FILTER_PREDICTED_OBJECTS_HPP_

#include "types.hpp"

#include <string>

namespace behavior_velocity_planner::out_of_lane
{
/// @brief filter predicted objects and their predicted paths
/// @param [in] objects predicted objects to filter
/// @param [in] params parameters
/// @return filtered predicted objects
 autoware_auto_perception_msgs::msg::PredictedObjects filter_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const PlannerParam & params) {
  autoware_auto_perception_msgs::msg::PredictedObjects filtered_objects;  
  filtered_objects.header = objects.header;
  for(const auto & object : objects.objects) {
    const auto is_pedestrian = std::find_if(
      object.classification.begin(), object.classification.end(),
      [](const auto & c){ return c.label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN; })
      != object.classification.end();
    const auto has_low_exist_probability =
      object.existence_probability < params.objects_min_existence_probability;
    if(is_pedestrian || has_low_exist_probability) continue;

    auto filtered_object = object;
    if(params.objects_use_predicted_paths) {
      auto & predicted_paths = filtered_object.kinematics.predicted_paths;
      const auto new_end = std::remove_if(
        predicted_paths.begin(), predicted_paths.end(),
        [&](const auto & predicted_path) {
          return predicted_path.confidence < params.objects_min_confidence;
        }
      );
      predicted_paths.erase(new_end, predicted_paths.end());
    }
    if(!params.objects_use_predicted_paths || !filtered_object.kinematics.predicted_paths.empty())
      filtered_objects.objects.push_back(filtered_object);
  }
  return filtered_objects;
}

}  // namespace behavior_velocity_planner::out_of_lane

#endif  // FILTER_PREDICTED_OBJECTS_HPP_
