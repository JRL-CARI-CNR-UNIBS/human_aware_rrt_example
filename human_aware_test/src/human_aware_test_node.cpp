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
#include <moveit_planning_helper/manage_trajectories.h>

#include "std_srvs/Empty.h"


#include <fstream>

float meanFcn(float new_data,float old_mean, int samples)
{
  float delta = new_data - old_mean;
  return old_mean + delta / samples;
}

float varianceM2Fcn(float new_data, float old_mean, float old_varianceM2, int samples)
{
  float delta = new_data - old_mean;
  float delta2 = new_data - (old_mean + delta / samples);
  return old_varianceM2 + delta*delta2;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hamp_benchmark");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Params */

  double planning_time = 5.0;
  if (!nh.getParam("planning_time",planning_time))
    ROS_WARN("planning_time not found. Default: %f", planning_time);

  int planning_trials;
  if (!nh.getParam("planning_trials",planning_trials))
  {
    ROS_FATAL("planning_trials not set.");
    return 0;
  }

  std::string planning_group;
  if (!nh.getParam("move_group_name",planning_group))
  {
    ROS_WARN("move_group_name not set");
    return 0;
  }

  std::string tool_frame;
  if (!nh.getParam("tool_frame",tool_frame))
  {
    ROS_WARN("tool_frame not set");
    return 0;
  }

  std::vector<std::string> planners;
  if (!nh.getParam("planners",planners))
  {
    ROS_WARN("move_group not found. Exit.");
    return 0;
  }
  else
  {
    ROS_INFO("Planners loaded:");
    for (unsigned int idx=0;idx<planners.size();idx++)
      std::cout << " - " << planners.at(idx) << "\n";
  }

  std::string planner_baseline;
  if (!nh.getParam("planner_baseline",planner_baseline))
  {
    ROS_WARN("Baseline planner not found. Exit.");
    return 0;
  }

  /* add baseline planner as first planner */
  planners.insert(planners.begin(), planner_baseline);

  double planning_time_baseline=20;
  if (!nh.getParam("planning_time_baseline",planning_time_baseline))
    ROS_WARN("planning_time_baseline not found. Default: %f", planning_time_baseline);

  int reps_query=1;
  if (!nh.getParam("reps_query",reps_query))
    ROS_WARN("reps_query not found. Default: %d", reps_query);

  bool limit_goal_workspace=false;
  std::vector<double> box_center(3,0);
  std::vector<double> box_size(3,10000);
  if (nh.getParam("limit_goal_workspace",limit_goal_workspace))
  {
    if (!nh.getParam("ws_limits/box_center",box_center))
    {
      ROS_ERROR("limit_goal_workspace is active but no box_center found. Exit.");
      return 0;
    }
    if (!nh.getParam("ws_limits/box_size",box_size))
    {
      ROS_ERROR("limit_goal_workspace is active but no box_size found. Exit.");
      return 0;
    }
    ROS_INFO("workspace limitation activated");
  }

  double minimum_distance_from_start_to_goal;
  if (!nh.getParam("minimum_distance_from_start_to_goal",minimum_distance_from_start_to_goal))
    minimum_distance_from_start_to_goal=0.0;

  ros::ServiceClient occupancy_srv = nh.serviceClient<std_srvs::Empty>("update_occupancy");
  if (!occupancy_srv.waitForExistence())
  {
    ROS_ERROR("Occupancy service not advertised. Abort.");
    return 0;
  }

  /* Set static params */

  pnh.setParam("planners",planners);
  pnh.setParam("repetitions",reps_query);
  pnh.setParam("planning_group",planning_group);
  pnh.setParam("planning_time_baseline",planning_time_baseline);
  pnh.setParam("planning_time",planning_time);
  pnh.setParam("planning_trials",planning_trials);

  pnh.setParam("queries_executed",0);

  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  moveit::core::JointModelGroup* jmg=kinematic_model->getJointModelGroup(planning_group);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* Create and update planning scene */

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene srv;
  ps_client.call(srv);
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  planning_scene->setPlanningSceneMsg(srv.response.scene);
  moveit_msgs::PlanningScene scene_msg;
  planning_scene->getPlanningSceneMsg(scene_msg);

  moveit_visual_tools::MoveItVisualTools visual_tools(tool_frame);
  visual_tools.deleteAllMarkers();

  std::map<int, rviz_visual_tools::colors> color_map;
  color_map[0] = rviz_visual_tools::RED;
  color_map[1] = rviz_visual_tools::BLUE;
  color_map[2] = rviz_visual_tools::GREEN;
  color_map[3] = rviz_visual_tools::MAGENTA;

  /* Results */

  std::vector<int> failure;
  std::vector<double> path_length;
  std::vector<double> exec_time;
  for(const auto& planner: planners)
  {
    failure.push_back(0);
    path_length.push_back(0.0);
    exec_time.push_back(0.0);
  }
  std::vector<int> failure_cum = failure;
  std::vector<double> path_length_avg = path_length;
  std::vector<double> path_length_varM2 = path_length;
  std::vector<double> exec_time_avg = exec_time;
  std::vector<double> exec_time_varM2 = exec_time_avg;

  moveit::core::RobotStatePtr current_state =  move_group.getCurrentState();
  moveit::core::RobotStatePtr start_state =  move_group.getCurrentState();
  moveit::core::RobotStatePtr target_state =  move_group.getCurrentState();

  /* LOOP */

  move_group.setStartStateToCurrentState();

  ros::Duration(2.0).sleep();

  for (unsigned int i_trial=0;i_trial<planning_trials;i_trial++)
  {
    start_state=move_group.getCurrentState();

    bool is_valid=false;
    unsigned int i_try=0;
    while(!is_valid)
    {
      if (i_try++>1000)
      {
        ROS_FATAL("Cannot find valid target. Exit.");
        return 0;
      }

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
      {
        is_valid=true;
      }

      if (limit_goal_workspace)
      {
        Eigen::Isometry3d target_pose = target_state->getFrameTransform(tool_frame);
        Eigen::VectorXd target_position = target_pose.translation();

        if (not (target_position(0)<=box_center.at(0)+0.5*box_size.at(0) &&
            target_position(0)>=box_center.at(0)-0.5*box_size.at(0) &&
            target_position(1)<=box_center.at(1)+0.5*box_size.at(1) &&
            target_position(1)>=box_center.at(1)-0.5*box_size.at(1) &&
            target_position(2)<=box_center.at(2)+0.5*box_size.at(2) &&
            target_position(2)>=box_center.at(2)-0.5*box_size.at(2) ) )
        {
          ROS_DEBUG("Target state out of custom workspace limits");
          is_valid=false;
          continue;
        }
        else
        {
          is_valid=true;
        }
      }

      if (minimum_distance_from_start_to_goal>0)
      {
        Eigen::VectorXd target_position = target_state->getFrameTransform(tool_frame).translation();
        Eigen::VectorXd start_position = start_state->getFrameTransform(tool_frame).translation();

        if ((target_position-start_position).norm() <= minimum_distance_from_start_to_goal)
        {
          ROS_DEBUG("Target state out of custom workspace limits");
          is_valid=false;
          continue;
        }
        else
        {
          is_valid=true;
        }
      }
      ROS_INFO_STREAM("Valid goal found at iteration " << i_try);
    }

    /* Change occupancy */

    std_srvs::Empty srv;
    if (!occupancy_srv.call(srv))
    {
      ROS_ERROR("occupancy service failed.");
    }

    visual_tools.deleteAllMarkers();

    for (std::size_t i_rep = 0; i_rep < reps_query; i_rep++)
    {

      for(std::size_t i_pl = 0; i_pl < planners.size(); ++i_pl)
      {
        std::string planner = planners[i_pl];

        move_group.setPlannerId(planner);
        move_group.setPlanningTime(planning_time);

        current_state=move_group.getCurrentState();

        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget(*target_state);

        std::vector<double> tmp_std = move_group.getCurrentJointValues();
        Eigen::VectorXd tmp;
        tmp.resize(tmp_std.size());
        for (unsigned int idx=0;idx<tmp_std.size();idx++)
          tmp(idx) = tmp_std.at(idx);

        target_state->copyJointGroupPositions(planning_group,tmp);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        /* Plan and execute */

        moveit::planning_interface::MoveItErrorCode plan_exit_code = move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        bool success=0;
        if (plan_exit_code==1)
          success=1;
        else
          success=0;

        if (!success)
        {
          ROS_DEBUG("Planning %s failed at trial %u", planner.c_str(), i_trial);
        }
        else
        {
          visual_tools.publishTrajectoryLine(plan.trajectory_,jmg,color_map[i_pl]);
          visual_tools.trigger();

          ros::Time t0 = ros::Time::now();
          moveit::planning_interface::MoveItErrorCode exec_exit_code = move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

          if (exec_exit_code==1)
            success=1;
          else
            success=0;

          exec_time[i_pl] = (ros::Time::now() - t0).toSec();
          path_length[i_pl]=trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

          if (!success)
            ROS_DEBUG("Executing %s failed at trial %u", planner.c_str(), i_trial);
          else
            ROS_DEBUG("%s trial %u : %s", planner.c_str(), i_trial, success ? "SUCCESS" : "FAILED");
        }

        ros::Duration(1.0).sleep();
        move_group.stop();

        if (!success)
        {
          failure[i_pl]=1;
          exec_time[i_pl]=0.0;
        }
        else
        {
          failure[i_pl]=0;
        }

        /* Save results to param */

        std::string result_prefix="query_"+std::to_string(i_trial)+"/"+planner+"/repetition_"+std::to_string(i_rep);

        pnh.setParam(result_prefix+"/prefix",result_prefix);
        pnh.setParam(result_prefix+"/trajectory_nominal_time",plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
        pnh.setParam(result_prefix+"/trajectory_time",exec_time[i_pl]);
        pnh.setParam(result_prefix+"/trajectory_length",path_length[i_pl]);
        pnh.setParam(result_prefix+"/planning_time",plan.planning_time_);
        pnh.setParam(result_prefix+"/outcome",success);
        pnh.setParam(result_prefix+"/average_slowdown",exec_time[i_pl]/plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());

        ROS_INFO_STREAM("Query: " << i_trial << ".\t Rep: " << i_rep << ".\t Planner: " << planner << ".\t\t Outcome: " << success << ".\t Nominal time: " << plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() << ".\t Time: " << exec_time[i_pl] << ".\t Length: " << path_length[i_pl]);

        current_state = move_group.getCurrentState();
        move_group.setStartStateToCurrentState();

        /* Going back to previous starting point */

        if (!planner.compare(planners.back()) && i_rep==reps_query-1)
        {
          ROS_DEBUG_STREAM("not going home. Planner: " << planner << ", repetition: " << i_rep );
          std::cout << planner.compare(planners.back()) << "\n";
        }
        else
        {
          move_group.setJointValueTarget(*start_state);
          moveit::planning_interface::MoveItErrorCode exit_code = move_group.plan(plan);
          if (exit_code!=1)
          {
            ROS_INFO("Failing at planning back to start config, trial %u", i_trial);
            continue;
          }
          exit_code = move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
          if (exit_code!=1)
          {
            ROS_INFO("Failing at moving back to start config, trial %u", i_trial);
            continue;
          }
          else
          {
            ROS_DEBUG("Back to home ok, trial %u", i_trial);
          }
        }

        ros::Duration(1.0).sleep();
        move_group.stop();

      } // for planners

    } // for repetitions

    std::cout << std::endl;

    /* Calculate mean and variance for video print */

    for(int i_pl = 0; i_pl < planners.size(); ++i_pl)
    {
      if (failure[0]==0)
      {
        if (failure[i_pl]==0)
        {
          double exec_time_norm=exec_time[i_pl]/exec_time[0];
          double path_length_norm = path_length[i_pl]/path_length[0];
          exec_time_varM2[i_pl] = varianceM2Fcn(exec_time_norm,exec_time_avg[i_pl],exec_time_varM2[i_pl],i_trial-failure_cum[i_pl]+1);
          exec_time_avg[i_pl] = meanFcn(exec_time_norm,exec_time_avg[i_pl],i_trial-failure_cum[i_pl]+1);
          path_length_varM2[i_pl] = varianceM2Fcn(path_length_norm,path_length_avg[i_pl],path_length_varM2[i_pl],i_trial-failure_cum[i_pl]+1);
          path_length_avg[i_pl] = meanFcn(path_length_norm,path_length_avg[i_pl],i_trial-failure_cum[i_pl]+1);
        }
      }
      if (failure[i_pl]==1)
        failure_cum[i_pl]++;
    }

    pnh.setParam("queries_executed",int(i_trial+1)); // update param of executed queries

    system("rosparam dump hamp_result.yaml /hamp");

  } // for queries

  std::cout << std::setprecision(4) << std::fixed;
  std::cout << "\n****************************************************\n";
  std::cout <<   "*                    Results                       *\n";
  std::cout <<   "****************************************************\n";
  std::cout << "Number of trials: " << planning_trials << "\n";
  std::cout << "\t\t\t\t\tmean\t\tst.dev\n";
  for(std::size_t i_pl = 0; i_pl < planners.size(); ++i_pl)
  {
    std::cout << "Planner: " << planners[i_pl].c_str()  << "\n";
    std::cout << "Execution time [normalized]:\t\t" << exec_time_avg[i_pl] << "\t\t" << sqrt(exec_time_varM2[i_pl]/(planning_trials-failure_cum[i_pl]-1.0)) << "\n";
    std::cout << "Path length [normalized]:\t\t" << path_length_avg[i_pl] << "\t\t" << sqrt(path_length_varM2[i_pl]/(planning_trials-failure_cum[i_pl]-1.0)) << "\n";
    std::cout << "N. of failures: \t\t\t" << failure_cum[i_pl] << "\n";
    std::cout << "\n";
  }
  std::cout <<   "****************************************************\n";

  ros::shutdown();
  return 0;
}
