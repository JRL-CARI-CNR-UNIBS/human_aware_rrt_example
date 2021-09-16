/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021 Marco Faroni, CNR-STIIMA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Marco Faroni */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/String.h>
#include <moveit_planning_helper/manage_trajectories.h>

#include "std_srvs/Empty.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_tests_hamp");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();


  ros::Publisher planner_pub=pnh.advertise<std_msgs::String>("planner_id",1);
  std_msgs::String planner_id_msg;

  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::string test_name;
  if (!pnh.getParam("test_name",test_name))
  {
    ROS_ERROR("%s/test_name not defined",pnh.getNamespace().c_str());
    return 0;
  }

  int queries_number=10;
  if (!pnh.getParam("queries_number",queries_number))
  {
    ROS_ERROR("%s/queries_number not defined",pnh.getNamespace().c_str());
    return 0;
  }

  int repetitions=5;
  if (!pnh.getParam("repetitions",repetitions))
  {
    ROS_ERROR("%s/repetitions not defined",pnh.getNamespace().c_str());
    return 0;
  }

  double planning_time = 5.0;
  if (!pnh.getParam("planning_time",planning_time))
    ROS_WARN("planning_time not found. Default: %f", planning_time);

  std::vector<std::string> planner_ids;
  if (!pnh.getParam("planner_ids",planner_ids))
  {
    ROS_WARN("move_group not found. Exit.");
    return 0;
  }
  else
  {
    ROS_INFO("Planners loaded:");
    for (unsigned int idx=0;idx<planner_ids.size();idx++)
      std::cout << " - " << planner_ids.at(idx) << "\n";
  }

  std::string planner_baseline;
  if (!pnh.getParam("planner_baseline",planner_baseline))
  {
    planner_baseline = planner_ids.at(0);
    ROS_WARN("Baseline planner not found. Using %s",planner_baseline.c_str());
  }

  double planning_time_baseline=20;
  if (!pnh.getParam("planning_time_baseline",planning_time_baseline))
    ROS_WARN("planning_time_baseline not found. Default: %f", planning_time_baseline);

  /* add baseline planner as first planner */

  planner_ids.insert(planner_ids.begin(), planner_baseline);


  /* Check for occupancy updater */
  ros::ServiceClient occupancy_srv = nh.serviceClient<std_srvs::Empty>("update_occupancy");
  if (!occupancy_srv.waitForExistence())
  {
    ROS_ERROR("Occupancy service not advertised. Abort.");
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader=std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader->getModel();

  /* Create and update planning scene */
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene srv;
  ps_client.call(srv);
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  planning_scene->setPlanningSceneMsg(srv.response.scene);
  moveit_msgs::PlanningScene scene_msg;
  planning_scene->getPlanningSceneMsg(scene_msg);

  ros::Duration(2).sleep();
  move_group.startStateMonitor();
  move_group.setNumPlanningAttempts(1);

  moveit::core::RobotState state(*move_group.getCurrentState());
  move_group.setStartStateToCurrentState();

  moveit::core::RobotStatePtr start_state =  move_group.getCurrentState();
  moveit::core::RobotStatePtr target_state =  move_group.getCurrentState();



  for (int actual_query_number=0;actual_query_number<queries_number;actual_query_number++)
  {

    /* Generate goal config */

    bool is_valid=false;
    unsigned int i_try=0;
    while(!is_valid)
    {
      target_state->setToRandomPositions();
      target_state->update();
      target_state->updateCollisionBodyTransforms();
      if (!target_state->satisfiesBounds())
      {
        ROS_DEBUG("Target state is out of bound");
        continue;
      }
      else if (!planning_scene->isStateValid(*target_state,planning_group))
      {
        ROS_DEBUG("Target state is in collision");
        continue;
      }
      else if (planning_scene->isStateColliding(*target_state,planning_group))
      {
        ROS_DEBUG("Target state is in collision");
        continue;
      }
      else
        is_valid=true;
      if (i_try++>100)
      {
        ROS_FATAL("Cannot find valid target. Exit.");
        return 0;
      }
    }

    /* Change occupancy */

    std_srvs::Empty srv;
    if (!occupancy_srv.call(srv))
    {
      ROS_ERROR("occupancy service failed.");
    }

    for (std::size_t i_rep = 0; i_rep < repetitions; i_rep++)
    {
      for(std::size_t i_pl = 0; i_pl < planner_ids.size(); ++i_pl)
      {
        std::string planner_id = planner_ids[i_pl];

        move_group.setPlannerId(planner_id);
        move_group.setPlanningTime(planning_time);

        state.setJointGroupPositions(group_name,start_configuration);
        state.update();
        move_group.setStartState(state);

        state.setJointGroupPositions(group_name,goal_configuration);
        state.update();
        move_group.setJointValueTarget(state);

    }





    for (const std::string& planner_id: planner_ids.at(pipeline_id))
    {
          ROS_INFO("PIPELINE = %s, PLANNER = %s. Planning time = %f.  = Query %d.",pipeline_id.c_str(),planner_id.c_str(),planning_time,actual_query_number);
          move_group.setPlanningTime(planning_time);
          move_group.setPlannerId(planner_id);

          ROS_DEBUG("PIPELINE = %s, PLANNER = %s. query %d of %d",pipeline_id.c_str(),planner_id.c_str(),actual_query_number+1,queries_number);
          planner_id_msg.data=pipeline_id+"/"+planner_id+" query "+std::to_string(actual_query_number);
          planner_pub.publish(planner_id_msg);

          std::string query_name=test_name;
          std::vector<double> start_configuration;
          std::vector<double> goal_configuration;

          pnh.getParam(test_name+"/query_"+std::to_string(actual_query_number)+"/start_configuration",start_configuration);
          pnh.getParam(test_name+"/query_"+std::to_string(actual_query_number)+"/goal_configuration" ,goal_configuration);

          state.setJointGroupPositions(group_name,start_configuration);
          state.update();
          move_group.setStartState(state);

          state.setJointGroupPositions(group_name,goal_configuration);
          state.update();
          move_group.setJointValueTarget(state);

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          for (int repetition=0;repetition<repetitions;repetition++)
          {
            double length=std::numeric_limits<double>::infinity();

            int tmp_rep=(pipeline_id=="dirrt")?1:1; // ompl runs 4 thread in parallel
            for (int itmp=0;itmp<tmp_rep;itmp++)
            {
              moveit::planning_interface::MoveItErrorCode plan_exit_code = move_group.plan(plan);
              std::string result_prefix=test_name+"/query_"+std::to_string(actual_query_number)+"/"+pipeline_id+"/"+planner_id+"/iteration_"+std::to_string(repetition)+"/planning_time_ms_"+std::to_string((int)(1000.0*planning_time));

              bool improved=false;
              if (plan_exit_code)
              {
                double length1=trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
                if (length1<length)
                {
                  length=length1;
                  pnh.setParam(result_prefix+"/trajectory_time",plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
                  pnh.setParam(result_prefix+"/trajectory_length",length);
                }
              }
              if (improved || itmp==0)
              {
                pnh.setParam(result_prefix+"/planning_time",plan.planning_time_);
                pnh.setParam(result_prefix+"/error_code",plan_exit_code.val);
              }
            }
      }  // for each repetion
    }  // for each planner
    system("rosparam dump benchmark_result.yaml /benchmark");
  }  // for each query

  return 0;
}
