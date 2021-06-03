# human_aware_rrt_example
This repo provides packages for testing and evaluating human-aware path planners.

## Packages

* **human_aware_test** : provides a test node for the evaluation of the human-aware path planner with a Panda Franka Emika arm.
* **dummy_occupancy** : publishes randomized human poses and pointclouds. These messages are used by the human_aware_test_node to plan human-aware paths.

## Usage

```
roslaunch human_aware_test human_aware_test_node.launch
```
## Result

The test node is expected to print a comparison of the planners as shown in the following picture:

TODO

A csv file containing the results of each execution of each query is saved at "~/results_hamp.csv".


## Configuration

In human_aware_test/launch/config.yaml, you can change:

- the moveIt! move group name

- the list of tested planners

- the maximum planning time for each query

- the size and position of the human workspace

In human_aware_test/launch/thor_safety.taml, you can set:

- the kinematic chain of the robot (base and tool frame)

- the links used to calculate the safety speed
