# Crosswalk

## Role

This module judges whether the ego should stop in front of the crosswalk in order to provide safe passage of pedestrians and bicycles based on object's behavior and surround traffic.

<figure markdown>
  ![example](docs/example.png){width=1000}
  <figcaption>crosswalk module</figcaption>
</figure>

## Features

### Yield

#### Target Object

The target object's type is filtered by the following parameters in the `object_filtering.target_object` namespace.

| Parameter    | Unit | Type | Description                                                                                                           |
| ------------ | ---- | ---- | --------------------------------------------------------------------------------------------------------------------- |
| `unknown`    | [-]  | bool | target vehicle velocity when module receive slow down command from FOA                                                |
| `bicycle`    | [-]  | bool | minimum jerk deceleration for safe brake                                                                              |
| `motorcycle` | [-]  | bool | minimum accel deceleration for safe brake                                                                             |
| `pedestrian` | [-]  | bool | if the current velocity is less than X m/s, ego always stops at the stop position(not relax deceleration constraints) |

For pedestrians crossing outside the crosswalk, the crosswalk module creates an attention area around the crosswalk. If the object's predicted path collides with the attention area, the object will be targeted for yield.

<figure markdown>
  ![crosswalk_attention_range](docs/crosswalk_attention_range.svg){width=1000}
  <figcaption>crosswalk attention range</figcaption>
</figure>

In the `object_filtering.target_object` namespace.

| Parameter                   | Unit | Type   | Description                                                                                       |
| --------------------------- | ---- | ------ | ------------------------------------------------------------------------------------------------- |
| `crosswalk_attention_range` | [m]  | double | the detection area is defined as -X meters before the crosswalk to +X meters behind the crosswalk |

#### Stop Position

First of all, `stop_distance_from_object` [m] is kept at least between the ego and the target object.

When the stop line exists in the lanelet map, the stop position is calculated based on the line.
When the stop line does **NOT** exist in the lanelet map, the stop position is calculated by keeping `stop_distance_from_crosswalk` between the ego and the crosswalk.

<figure markdown>
  ![stop_distance_from_object](docs/stop_margin.svg){width=1000}
  <figcaption>stop margin</figcaption>
</figure>

<figure markdown>
  ![stop_line](docs/stop_line.svg){width=700}
  <figcaption>explicit stop line</figcaption>
</figure>

<figure markdown>
  ![stop_distance_from_crosswalk](docs/stop_line_distance.svg){width=700}
  <figcaption>virtual stop point</figcaption>
</figure>

On the other hand, if pedestrian (bicycle) is crossing **wide** crosswalks seen in scramble intersections, and the pedestrian position is more than `far_object_threshold` meters away from the stop line, the actual stop position is determined to be `stop_distance_from_object` and pedestrian position, not at the stop line.

<figure markdown>
  ![far_object_threshold](docs/stop_line_margin.svg){width=1000}
  <figcaption>stop at wide crosswalk</figcaption>
</figure>

See the workflow in algorithms section.

| Parameter                                    | Type   | Description                                                                                                                                                               |
| -------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `stop_position.stop_distance_from_object`    | double | [m] the vehicle decelerates to be able to stop in front of object with margin                                                                                             |
| `stop_position.stop_distance_from_crosswalk` | double | [m] make stop line away from crosswalk when no explicit stop line exists                                                                                                  |
| `stop_position.far_object_threshold`         | double | [m] if objects cross X meters behind the stop line, the stop position is determined according to the object position (stop_distance_from_object meters before the object) |
| `stop_position.stop_position_threshold`      | double | [m] threshold for check whether the vehicle stop in front of crosswalk                                                                                                    |

#### Yield decision

The module make a decision to yield only when the pedestrian traffic ligh is **GREEN** or **UNKNOWN**.
Calculating the collision point, the decision is based on the following variables.

- TTC: Time-To-Collision which is the time for the **ego** to reach the virtual collision point.
- TTV: Time-To-Vehicle which is the time for the **object** to reach the virtual collision point.

<figure markdown>
  ![virtual_collision_point](docs/virtual_collision_point.svg){width=1000}
  <figcaption>virtual collision point</figcaption>
</figure>

Depending on the relative relationship between TTC and TTV, the ego's behavior at crosswalks can be classified into three categories based on [1]

- A. **TTC >> TTV**: The object has enough time to cross before the ego.
  - No stop planning.
- B. **TTC ≒ TTV**: There is a riskof collision.
  - **Stop point is inserted in the ego's path**.
- C. **TTC << TTV**: Ego has enough time to cross before the object.
  - No stop planning.

<figure markdown>
  ![ttc-ttv](docs/ttc-ttv.svg){width=1000}
  <figcaption>time-to-collision vs time-to-vehicle</figcaption>
</figure>

### Dead Lock Prevention

When the object is stopped around the crosswalk but has no intention to walk, the ego will yield the object forever.
To prevent this dead lock, the ego will cancel the yield depending on the situation.

#### When there is no traffic light

For the object is stopped around the crosswalk but has no intention to walk, when the ego keeps stopping to yield for a certain time (\*1), the ego cancels the yield and start driving.

<figure markdown>
  ![no-intension](docs/no-intension.svg){width=1000}
  <figcaption>dead lock situation</figcaption>
</figure>

\*1:
The time is calculated based on the distance between the object and crosswalk.
When `distance_map_for_no_intention_to_walk` is `[1.0, 5.0]` and `timeout_map_for_no_intention_to_walk` is `[3.0, 0.0]`, the time is calculated as follows.

#### When there is traffic light

For the object is stopped around the crosswalk but has no intention to walk, when the ego will cancel the yield without stopping.
This comes from the assumption that the object has no intention to walk since it is stopped even though the pedestrian traffic light is green.

<figure markdown>
  ![no-intension](docs/no-intension.svg){width=1000}
  <figcaption>dead lock situation</figcaption>
</figure>

#### New Object Handling

### Safety Slow Down Behavior

In current autoware implementation if there are no target objects around a crosswalk, ego vehicle
will not slow down for the crosswalk. However, if ego vehicle to slow down to a certain speed in
such cases is wanted then it is possible by adding some tags to the related crosswalk definition as
it is instructed
in [lanelet2_format_extension.md](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md)
document.

| Parameter             |         | Type   | Description                                                                                                           |
| --------------------- | ------- | ------ | --------------------------------------------------------------------------------------------------------------------- |
| `slow_velocity`       | [m/s]   | double | target vehicle velocity when module receive slow down command from FOA                                                |
| `max_slow_down_jerk`  | [m/sss] | double | minimum jerk deceleration for safe brake                                                                              |
| `max_slow_down_accel` | [m/ss]  | double | minimum accel deceleration for safe brake                                                                             |
| `no_relax_velocity`   | [m/s]   | double | if the current velocity is less than X m/s, ego always stops at the stop position(not relax deceleration constraints) |

### Stuck Vehicle Detection

The feature will make the ego not to stop on the crosswalk.
When there are low-speed or stopped vehicle ahead of the crosswalk, and there is not enough space between the crosswalk and the vehicle, the crosswalk module will plan to stop before the crosswalk even if there are no pedestrians or bicycles.

`min_acc`, `min_jerk`, and `max_jerk` are met. If the ego cannot stop before the crosswalk with these parameters, the stop position will move forward.

<figure markdown>
  ![stuck_vehicle_attention_range](docs/stuck_vehicle_attention_range.svg){width=1000}
  <figcaption>stuck vehicle attention range</figcaption>
</figure>

In the `stuck_vehicle` namespace,

| Parameter                          | Unit    | Type   | Description                                                             |
| ---------------------------------- | ------- | ------ | ----------------------------------------------------------------------- |
| `stuck_vehicle_velocity`           | [m/s]   | double | maximum velocity threshold whether the target vehicle is stopped or not |
| `max_stuck_vehicle_lateral_offset` | [m]     | double | maximum lateral offset of the target vehicle position                   |
| `stuck_vehicle_attention_range`    | [m]     | double | detection area length ahead of the crosswalk                            |
| `min_acc`                          | [m/ss]  | double | minimum acceleration to stop                                            |
| `min_jerk`                         | [m/sss] | double | minimum jerk to stop                                                    |
| `max_jerk`                         | [m/sss] | double | maximum jerk to stop                                                    |

#### Parameters for pass judge logic

Also see algorithm section.

| Parameter                                   | Type   | Description                                                                                                                                        |
| ------------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pass_judge.ego_pass_first_margin`          | double | [s] time margin for ego pass first situation                                                                                                       |
| `pass_judge.ego_pass_later_margin`          | double | [s] time margin for object pass first situation                                                                                                    |
| `pass_judge.stop_object_velocity_threshold` | double | [m/s] velocity threshold for the module to judge whether the objects is stopped                                                                    |
| `pass_judge.min_object_velocity`            | double | [m/s] minimum object velocity (compare the estimated velocity by perception module with this parameter and adopt the larger one to calculate TTV.) |
| `pass_judge.timeout_no_intention_to_walk`   | double | [s] if the pedestrian does not move for X seconds after stopping before the crosswalk, the module judge that ego is able to pass first.            |
| `pass_judge.timeout_ego_stop_for_yield`     | double | [s] the amount of time which ego should be stopping to query whether it yields or not.                                                             |

## Inner-workings / Algorithms

### Yield Decision

This module uses the larger value of estimated object velocity and `min_object_velocity` in calculating TTV in order to avoid division by zero.

```plantuml
start
if (Pedestrian's traffic light signal is **RED**?) then (yes)
else (no)
  if (There are objects around the crosswalk?) then (yes)
    :calculate TTC & TTV;
    if (TTC < TTV + **ego_pass_first_margin** && TTV < TTC + **ego_pass_later_margin**) then (yes)
      :STOP;
    else (no)
      :PASS;
    endif
  endif
endif
end
```

## Other Parameters

| Parameter                            | Type   | Description                                    |
| ------------------------------------ | ------ | ---------------------------------------------- |
| `common.show_processing_time`        | bool   | whether to show processing time                |
| `common.traffic_light_state_timeout` | double | [s] timeout threshold for traffic light signal |

## Known Issues

## Debugging

### Visualization of debug markers

### Visualization of Time-To-Collision

By `ros2 run behavior_velocity_crosswalk_module time_to_collision_plotter.py`, you can visualize the following figure of the ego and pedestrian's time to collision.
The label of each plot is `<crosswalk module id>-<pedestrian uuid>`.

<figure markdown>
  ![limitation](docs/time_to_collision_plot.png){width=1000}
  <figcaption>Plot of time to collision</figcaption>
</figure>

## References/External links

[1] 佐藤 みなみ, 早坂 祥一, 清水 政行, 村野 隆彦, 横断歩行者に対するドライバのリスク回避行動のモデル化, 自動車技術会論文集, 2013, 44 巻, 3 号, p. 931-936.
