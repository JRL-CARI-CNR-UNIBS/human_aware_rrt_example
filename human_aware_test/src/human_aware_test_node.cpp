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
  ros::init(argc, argv, "human_aware_franka_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string filename = "/home/hypatia/Scrivania/results_hamp.csv";
  std::ofstream results_file;
  results_file.open (filename);
  results_file.close();

  /* Params */
  int n_goals=1;
  int planning_trials = 3;
  double planning_time = 5.0;
  std::string planning_group = "panda_arm";
  std::vector<std::string> planners;

  if (!node_handle.getParam("n_goals",n_goals))
    ROS_WARN("n_goals not found. Default: %d", n_goals);
  if (!node_handle.getParam("planning_time",planning_time))
    ROS_WARN("planning_time not found. Default: %f", planning_time);
  if (!node_handle.getParam("planning_trials",planning_trials))
    ROS_WARN("planning_trials not found. Default: %d", planning_trials);
  if (!node_handle.getParam("move_group_name",planning_group))
    ROS_WARN("move_group_name not found. Default: %s", planning_group);
  if (!node_handle.getParam("planners",planners))
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

  ros::ServiceClient occupancy_srv = node_handle.serviceClient<std_srvs::Empty>("update_occupancy");
  if (!occupancy_srv.waitForExistence())
  {
    ROS_ERROR("Occupancy service not advertised. Abort.");
    return 0;
  }


  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  move_group.setPlanningTime(planning_time);



//  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, pnh, "/move_group/planning_plugin", "/move_group/request_adapters"));
//  moveit::core::JointModelGroup* jmg=robot_model->getJointModelGroup(group_name);

//  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/"+group_name+"/follow_joint_trajectory",true);

//  if (!ac.waitForServer())
//  {
//    ROS_ERROR("unable to conenct with execution server");
//    return 0;
//  }

//  control_msgs::FollowJointTrajectoryGoal trj_goal;

//  std::string what;

//  planning_scene::PlanningScenePtr scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());
//  moveit_msgs::PlanningScene scene_msg;
//  scene->getPlanningSceneMsg(scene_msg);
//  ROS_INFO_STREAM("scene\n"<<scene_msg.robot_state.joint_state);
//  robot_state::RobotState state(scene->getCurrentState());




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
  std::vector<double> exec_time_avg = exec_time;
  std::vector<double> exec_time_var = exec_time_avg;
  std::vector<double> path_length_var = path_length;

  moveit::core::RobotStatePtr current_state =  move_group.getCurrentState();
  moveit::core::RobotStatePtr start_state =  move_group.getCurrentState();
  moveit::core::RobotStatePtr target_state =  move_group.getCurrentState();
  move_group.setStartState(*start_state);

  for (unsigned int i_trial=0;i_trial<planning_trials;i_trial++)
  {
    start_state=move_group.getCurrentState();

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

    /* Execute */

    for(std::size_t i_pl = 0; i_pl < planners.size(); ++i_pl)
    {
      std::string planner = planners[i_pl];

      move_group.setPlannerId(planner);
      std::string planner_id = move_group.getPlannerId();
      ROS_ERROR("planner_id=%s", planner_id.c_str());

      current_state=move_group.getCurrentState();
      move_group.setStartState(*start_state);
      move_group.setJointValueTarget(*target_state);

      moveit::planning_interface::MoveGroupInterface::Plan plan;

      /* Plan and execute */
      bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!success)
        ROS_INFO_NAMED("Hamp", "Planning %s failed at trial %u", planner.c_str(), i_trial);
      else
      {
        ros::Time t0 = ros::Time::now();
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        exec_time[i_pl] = (ros::Time::now() - t0).toSec();
        path_length[i_pl]=trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
        std::cout << "path length= " << path_length[i_pl];
        if (!success)
          ROS_INFO_NAMED("Hamp", "Executing %s failed at trial %u", planner.c_str(), i_trial);
        else
          ROS_INFO_NAMED("Hamp", "%s trial %u : %s", planner.c_str(), i_trial, success ? "SUCCESS" : "FAILED");
      }

      ros::Duration(1.0).sleep();

      if (!success)
      {
        failure[i_pl]=!success;
        exec_time[i_pl]=0.0;
        path_length[i_pl]=0.0;
      }

      current_state = move_group.getCurrentState();
      move_group.setStartState(*current_state);

      /* Going back to previous starting point */
      if (planner.compare(planners.back()))
      {
        move_group.setJointValueTarget(*start_state);
        success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
          ROS_INFO_NAMED("Hamp", "Failing at planning back to start config, trial %u", i_trial);
          continue;
        }
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
        {
          ROS_INFO_NAMED("Hamp", "Failing at moving back to start config, trial %u", i_trial);
          continue;
        }
      }

    }

    /* Save to file */
    results_file.open (filename, std::ios_base::app);
    for(int i_pl = 0; i_pl < planners.size(); ++i_pl)
    {
      results_file << exec_time[i_pl] << "\t" << path_length[i_pl] << "\t";
    }
    results_file << "\n";
    results_file.close();


    /* Calculate mean and variance */
    for(int i_pl = 0; i_pl < planners.size(); ++i_pl)
    {
      if (!failure[0])
      {
        if (!failure[i_pl])
        {
          double exec_time_norm=exec_time[i_pl]/exec_time[0];
          double path_length_norm = path_length[i_pl]/path_length[0];
          exec_time_var[i_pl] = varianceM2Fcn(exec_time_norm,exec_time_avg[i_pl],exec_time_var[i_pl],i_trial-failure_cum[i_pl]+1);
          exec_time_avg[i_pl] = meanFcn(exec_time_norm,exec_time_avg[i_pl],i_trial-failure_cum[i_pl]+1);
          path_length_var[i_pl] = varianceM2Fcn(path_length_norm,path_length_avg[i_pl],path_length_var[i_pl],i_trial-failure_cum[i_pl]+1);
          path_length_avg[i_pl] = meanFcn(path_length_norm,path_length_avg[i_pl],i_trial-failure_cum[i_pl]+1);
        }
      }
      if (failure[i_pl])
        failure_cum[i_pl]++;
    }
  }

  std::cout << std::setprecision(4) << std::fixed;
  std::cout << "\n****************************************************\n";
  std::cout <<   "*                    Results                       *\n";
  std::cout <<   "****************************************************\n";
  std::cout << "\t\t\t\t\tmean\t\tst.dev\n";
  for(std::size_t i_pl = 0; i_pl < planners.size(); ++i_pl)
  {
    std::cout << "Planner: " << planners[i_pl].c_str()  << "\n";
    std::cout << "Execution time [normalized]:\t\t" << exec_time_avg[i_pl] << "\t\t" << sqrt(exec_time_var[i_pl]/(planning_trials-failure_cum[i_pl]-1.0)) << "\n";
    std::cout << "Path length [normalized]:\t\t" << path_length_avg[i_pl] << "\t\t" << sqrt(path_length_var[i_pl]/(planning_trials-failure_cum[i_pl]-1.0)) << "\n";
    std::cout << "N. of failures: \t\t\t" << failure_cum[i_pl] << "\n";
    std::cout << "\n";
  }
  std::cout <<   "****************************************************\n";

  ros::shutdown();
  return 0;
}
