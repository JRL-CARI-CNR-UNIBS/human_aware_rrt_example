/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  robot_model_loader::RobotModelLoaderPtr robot_model_loader=std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();


  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);


  std::vector<ros::Publisher> pick_target_pubs;
  std::vector<ros::Publisher> place_target_pubs;

  planning_scene_monitor::PlanningSceneMonitor psm(robot_model_loader,tf_buffer);
  psm.startSceneMonitor();
  psm.startStateMonitor();

  ros::Duration(1).sleep();


  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name not defined",nh.getNamespace().c_str());
    return 0;
  }

  std::string tool_name;
  if (!pnh.getParam("tool_name",tool_name))
  {
    ROS_ERROR("%s/tool_name not defined",nh.getNamespace().c_str());
    return 0;
  }

  double planning_time;
  if (!pnh.getParam("planning_time",planning_time))
  {
    ROS_ERROR("%s/planning_time not defined",nh.getNamespace().c_str());
    return 0;
  }

  double approach_distance;
  if (!pnh.getParam("approach_distance",approach_distance))
  {
    ROS_ERROR("%s/approach_distance not defined",nh.getNamespace().c_str());
    return 0;
  }

  std::string planning_name="planning_plugin";
  if (!pnh.getParam("planning_plugin",planning_name))
  {
    ROS_ERROR("%s/planning_plugin not defined",nh.getNamespace().c_str());
    return 0;
  }
  else
  {
    ROS_INFO("%s/planning_plugin = %s",pnh.getNamespace().c_str(),planning_name.c_str());
  }
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, pnh, "/move_group/planning_plugin", "/move_group/request_adapters"));
  moveit::core::JointModelGroup* jmg=robot_model->getJointModelGroup(group_name);


  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/"+group_name+"/follow_joint_trajectory",true);

  if (!ac.waitForServer())
  {
    ROS_ERROR("unable to conenct with execution server");
    return 0;
  }

  control_msgs::FollowJointTrajectoryGoal trj_goal;

  std::string what;

  planning_scene::PlanningScenePtr scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());
  moveit_msgs::PlanningScene scene_msg;
  scene->getPlanningSceneMsg(scene_msg);
  ROS_INFO_STREAM("scene\n"<<scene_msg.robot_state.joint_state);
  robot_state::RobotState state(scene->getCurrentState());



  std::vector<Eigen::VectorXd> picking_configurations;
  std::vector<Eigen::VectorXd> approach_picking_configurations;
  std::vector<Eigen::VectorXd> candidate_picking_configurations;
  if (!rosparam_utilities::getParam(pnh,"picking_configurations",candidate_picking_configurations,what))
  {
    ROS_ERROR("Parameter %s/picking_configurations is not correct.",pnh.getNamespace().c_str());
    return 0;
  }

  int target_pub_number=0;
  for (const Eigen::VectorXd& conf: candidate_picking_configurations)
  {
    state.setJointGroupPositions(group_name,conf);
    state.updateCollisionBodyTransforms();
    if (!scene->isStateValid(state,group_name))
    {
      ROS_ERROR_STREAM("invalid picking configuration\n"<<conf.transpose()<<"\nskip it");
      continue;
    }

    Eigen::Isometry3d T_b_t=state.getFrameTransform(tool_name);
    T_b_t.translation()(2)+=approach_distance;

    if (!state.setFromIK(robot_model->getJointModelGroup(group_name),T_b_t,tool_name))
    {
      ROS_ERROR_STREAM("unable to compute IK for picking configuration\n"<<conf.transpose()<<"\nskip it");
      continue;
    }
    else
    {
      picking_configurations.push_back(conf);
      Eigen::VectorXd approach;
      state.copyJointGroupPositions(group_name,approach);
      approach_picking_configurations.push_back(approach);
      ROS_INFO_STREAM("Add approach pose for picking configuration\n"<<conf.transpose()<<"\napproach:\n"<<approach.transpose());

      ros::Publisher target_pub=nh.advertise<geometry_msgs::PoseStamped>("target"+std::to_string(target_pub_number++),1,true);
      pick_target_pubs.push_back(target_pub);
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id="world";
      tf::poseEigenToMsg(state.getFrameTransform(tool_name),msg.pose);
      pick_target_pubs.back().publish(msg);
      pick_target_pubs.back().publish(msg);

    }
  }

  std::vector<Eigen::VectorXd> placing_configurations;
  std::vector<Eigen::VectorXd> approach_placing_configurations;
  std::vector<Eigen::VectorXd> candidate_placing_configurations;
  if (!rosparam_utilities::getParam(pnh,"placing_configurations",candidate_placing_configurations,what))
  {
    ROS_ERROR("Parameter %s/placing_configurations is not correct.",pnh.getNamespace().c_str());
    return 0;
  }

  target_pub_number=0;
  for (const Eigen::VectorXd& conf: candidate_placing_configurations)
  {
    state.setJointGroupPositions(group_name,conf);
    if (!scene->isStateValid(state,group_name))
    {
      ROS_ERROR_STREAM("invalid placing configuration\n"<<conf.transpose()<<"\nskip it");
      continue;
    }

    Eigen::Isometry3d T_b_t=state.getFrameTransform(tool_name);
    T_b_t.translation()(2)+=approach_distance;

    if (!state.setFromIK(jmg,T_b_t,tool_name))
    {
      ROS_ERROR_STREAM("unable to compute IK for placing configuration\n"<<conf.transpose()<<"\nskip it");
      continue;
    }
    else
    {
      state.updateCollisionBodyTransforms();
      if (!scene->isStateValid(state,group_name))
      {
        ROS_ERROR_STREAM("unable to compute IK for placing configuration\n"<<conf.transpose()<<"\nskip it");
        continue;
      }
      placing_configurations.push_back(conf);
      Eigen::VectorXd approach;
      state.copyJointGroupPositions(group_name,approach);
      approach_placing_configurations.push_back(approach);
      ROS_INFO_STREAM("Add approach pose for placing configuration\n"<<conf.transpose()<<"\napproach:\n"<<approach.transpose());
      ros::Publisher target_pub=nh.advertise<geometry_msgs::PoseStamped>("place"+std::to_string(target_pub_number++),1,true);
      place_target_pubs.push_back(target_pub);
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id="world";
      tf::poseEigenToMsg(state.getFrameTransform(tool_name),msg.pose);
      place_target_pubs.back().publish(msg);
      place_target_pubs.back().publish(msg);
    }
  }

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = group_name;
  req.allowed_planning_time = planning_time;
  ROS_INFO("list of adapters:");
  for (const std::string adapter: planning_pipeline->getAdapterPluginNames())
    ROS_INFO("adapter: %s",adapter.c_str());

  ros::Rate lp(10);

  moveit_msgs::Constraints joint_goal;
  while (ros::ok())
  {
    lp.sleep();

    scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());
    robot_state::RobotState current_state(scene->getCurrentState());
    robot_state::RobotState goal_state(scene->getCurrentState());
    robot_state::RobotState pick_approach_state(scene->getCurrentState());
    robot_state::RobotState pick_state(scene->getCurrentState());
    robot_state::RobotState place_approach_state(scene->getCurrentState());
    robot_state::RobotState place_state(scene->getCurrentState());
    Eigen::VectorXd goal_configuration;


    /*
     * select one pick approach
     * move to pick approach
     */
    moveit::core::robotStateToRobotStateMsg(current_state,req.start_state);
    req.goal_constraints.clear();
    for (const Eigen::VectorXd& goal: approach_picking_configurations)
    {
      goal_state.setJointGroupPositions(group_name, goal);
      joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
      req.goal_constraints.push_back(joint_goal);
    }

    if (!planning_pipeline->generatePlan(scene,req,res))
    {
      ROS_ERROR("unable to plan to approach pick");
      break;
    }

    res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,goal_configuration);
    pick_approach_state.setJointGroupPositions(group_name, goal_configuration);

    moveit_msgs::RobotTrajectory trj_msg;
    res.trajectory_->getRobotTrajectoryMsg(trj_msg);
    trj_goal.trajectory=trj_msg.joint_trajectory;
    ROS_INFO("send goal");
    ac.sendGoalAndWait(trj_goal);
    ROS_INFO("executed");

    /*
     * move to pick
     */
    scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());
    moveit::core::robotStateToRobotStateMsg(pick_approach_state,req.start_state);

    req.goal_constraints.clear();
    for (size_t isol=0;isol<approach_picking_configurations.size();isol++)
    {
      if ((goal_configuration-approach_picking_configurations.at(isol)).norm()<1e-4)
      {
        pick_state.setJointGroupPositions(group_name, picking_configurations.at(isol));
        joint_goal = kinematic_constraints::constructGoalConstraints(pick_state, jmg);
        req.goal_constraints.push_back(joint_goal);
        break;
      }
    }

    if (!planning_pipeline->generatePlan(scene,req,res))
    {
      ROS_ERROR("unable to plan to pick");
      break;
    }
    res.trajectory_->getRobotTrajectoryMsg(trj_msg);
    trj_goal.trajectory=trj_msg.joint_trajectory;
    ROS_INFO("send goal");
    ac.sendGoalAndWait(trj_goal);
    ROS_INFO("executed");


    /*
     * move back to approach pick
     */
    scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());

    moveit::core::robotStateToRobotStateMsg(pick_state,req.start_state);
    req.goal_constraints.clear();
    joint_goal = kinematic_constraints::constructGoalConstraints(pick_approach_state, jmg);
    req.goal_constraints.push_back(joint_goal);

    if (!planning_pipeline->generatePlan(scene,req,res))
    {
      ROS_ERROR("unable to plan to approach place");
      break;
    }
    res.trajectory_->getRobotTrajectoryMsg(trj_msg);
    trj_goal.trajectory=trj_msg.joint_trajectory;
    ROS_INFO("send goal");
    ac.sendGoalAndWait(trj_goal);
    ROS_INFO("executed");


    /*
     * select one place approach
     * move to place approach
     */
    moveit::core::robotStateToRobotStateMsg(pick_approach_state,req.start_state);
    req.goal_constraints.clear();
    for (const Eigen::VectorXd& goal: approach_placing_configurations)
    {
      goal_state.setJointGroupPositions(group_name, goal);
      joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
      req.goal_constraints.push_back(joint_goal);
    }

    if (!planning_pipeline->generatePlan(scene,req,res))
    {
      ROS_ERROR("unable to plan to approach place");
      break;
    }

    res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,goal_configuration);
    place_approach_state.setJointGroupPositions(group_name, goal_configuration);

    res.trajectory_->getRobotTrajectoryMsg(trj_msg);
    trj_goal.trajectory=trj_msg.joint_trajectory;
    ROS_INFO("send goal");
    ac.sendGoalAndWait(trj_goal);
    ROS_INFO("executed");

    /*
     * move to place
     */
    scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());
    moveit::core::robotStateToRobotStateMsg(place_approach_state,req.start_state);

    req.goal_constraints.clear();
    for (size_t isol=0;isol<approach_placing_configurations.size();isol++)
    {
      if ((goal_configuration-approach_placing_configurations.at(isol)).norm()<1e-4)
      {
        place_state.setJointGroupPositions(group_name, placing_configurations.at(isol));
        joint_goal = kinematic_constraints::constructGoalConstraints(place_state, jmg);
        req.goal_constraints.push_back(joint_goal);
        break;
      }
    }

    if (!planning_pipeline->generatePlan(scene,req,res))
    {
      ROS_ERROR("unable to plan to place");
      break;
    }
    res.trajectory_->getRobotTrajectoryMsg(trj_msg);
    trj_goal.trajectory=trj_msg.joint_trajectory;
    ROS_INFO("send goal");
    ac.sendGoalAndWait(trj_goal);
    ROS_INFO("executed");


    /*
     * move back to approach place
     */
    scene=planning_scene::PlanningScene::clone(psm.getPlanningScene());

    moveit::core::robotStateToRobotStateMsg(place_state,req.start_state);
    req.goal_constraints.clear();
    joint_goal = kinematic_constraints::constructGoalConstraints(place_approach_state, jmg);
    req.goal_constraints.push_back(joint_goal);

    if (!planning_pipeline->generatePlan(scene,req,res))
    {
      ROS_ERROR("unable to plan to approach place");
      continue;
    }
    res.trajectory_->getRobotTrajectoryMsg(trj_msg);
    trj_goal.trajectory=trj_msg.joint_trajectory;
    ROS_INFO("send goal");
    ac.sendGoalAndWait(trj_goal);
    ROS_INFO("executed");
  }


  return 0;
}
