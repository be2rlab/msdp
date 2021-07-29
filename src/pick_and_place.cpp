#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros/ros.h"
#include "the_mainest/GoalPoses.h"
#include "the_mainest/JSPosition.h"
#include "the_mainest/CartPosition.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdio.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

static const std::string PLANNING_GROUP = "manipulator";

class Pick_and_place_class
{
private:
  ros::NodeHandle n;
  int state;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface *move_group_interface_ptr;
  const moveit::core::JointModelGroup *joint_model_group;

  //namespace rvt = rviz_visual_tools;
  ros::ServiceServer service;
  ros::ServiceServer js_service;
  ros::ServiceServer cart_service;

public:
  Pick_and_place_class() : n("~")
  {
    move_group_interface_ptr = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    joint_model_group = move_group_interface_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    state = 0;
    service = n.advertiseService("/GoalPoses", &Pick_and_place_class::is_ok, this);
    js_service = n.advertiseService("/js_position_cmd", &Pick_and_place_class::js_service_handler, this);
    cart_service = n.advertiseService("/cart_position_cmd", &Pick_and_place_class::cart_service_handler, this);
  }

  ~Pick_and_place_class()
  {
    delete move_group_interface_ptr;
  }

  bool Plan_and_execute_pregrasp_position(geometry_msgs::Pose &p)
  {
    ROS_INFO("plan: point setup");
    geometry_msgs::Pose pre_goal(p);
    pre_goal.position.z = pre_goal.position.z + 0.5;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group_interface_ptr->setPoseTarget(pre_goal);
    move_group_interface_ptr->allowReplanning(true);

    ROS_INFO("plan: planning");

    bool success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("plan: planning complete");

    // Если путь спланирован, то выполняем перемещение
    if (success == true)
    {
      ROS_INFO("Starting the trajectory execution ........................");
      bool success2 = (move_group_interface_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success2 == true)
      {
        return true;
        ROS_INFO("Trajectory executed!");
      }
      else
      {
        ROS_INFO("Trajectory didn't execute!");
        return false;
      }
    }
    else
    {
      ROS_INFO("Error when trying to execute trajectory. There are no plan!");
      return false;
    }
  }

  bool Plan_and_execute_grasp_position(geometry_msgs::Pose &p)
  {
    p.position.z = p.position.z + 0.4;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group_interface_ptr->setPoseTarget(p);
    bool success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // Если путь спланирован, то выполняем перемещение
    if (success == true)
    {
      bool success2 = (move_group_interface_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success2 == true)
      {
        ROS_INFO("Trajectory executed!");
        return true;
      }
      else
      {
        ROS_INFO("Trajectory didn't execute!");
        return false;
      }
    }
    else
    {
      ROS_INFO("Error when trying to execute trajectory. There are no plan!");
      return false;
    }
  }

  bool is_ok(the_mainest::GoalPoses::Request &req,
             the_mainest::GoalPoses::Response &res)
  {
    geometry_msgs::Pose p1(req.p1);
    geometry_msgs::Pose p2(req.p2);
    ROS_INFO("Service was called! The points is %5f and %5f ", p1.position.x, p2.position.x);
    bool at_place;
    move_group_interface_ptr->setPlannerId("RRTConnect");

    ROS_INFO("Wow");
    // Планируем путь к положению перед захватом объекта

    move_group_interface_ptr->setMaxAccelerationScalingFactor(0.5);
    move_group_interface_ptr->setMaxVelocityScalingFactor(0.5);

    at_place = Plan_and_execute_pregrasp_position(p1);
    if (at_place == true)
    {
      at_place = false;
      ROS_INFO("Planningto the pregrasp postition1 is successful!");
      move_group_interface_ptr->setMaxAccelerationScalingFactor(0.2);
      move_group_interface_ptr->setMaxVelocityScalingFactor(0.2);
      at_place = Plan_and_execute_grasp_position(p1);
      if (at_place == true)
      {
        at_place = false;
        ROS_INFO("Execution to the grasp postition1 is successful!");
        move_group_interface_ptr->setMaxAccelerationScalingFactor(0.5);
        move_group_interface_ptr->setMaxVelocityScalingFactor(0.5);
        at_place = Plan_and_execute_pregrasp_position(p2);
        if (at_place == true)
        {
          at_place = false;
          ROS_INFO("Planning to the pregrasp postition2 is successful!");
          move_group_interface_ptr->setMaxAccelerationScalingFactor(0.2);
          move_group_interface_ptr->setMaxVelocityScalingFactor(0.2);
          at_place = Plan_and_execute_grasp_position(p2);
          if (at_place == true)
          {
            ROS_INFO("Execution to the grasp postition1 is successful!");
            return true;
          }
          else
          {
            ROS_INFO("Execution to the pregrasp postition2 failed!");
            return false;
          }
        }
        else
        {
          ROS_INFO("Planning to the pregrasp postition2 failed!");
          return false;
        }
      }
      else
      {
        ROS_INFO("Planning to the grasp postition1 failed!");
        return false;
      }
    }
    else
    {
      ROS_INFO("Planning to the pregrasp postition1 failed!");
      return false;
    }
  }


  bool js_service_handler(the_mainest::JSPosition::Request &req, the_mainest::JSPosition::Response &res)
  {
    moveit::core::RobotStatePtr current_state = move_group_interface_ptr->getCurrentState();

    std::vector<double> joint_group_positions;

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    for (int i = 0; i < 6; ++i) {
      joint_group_positions[i] = req.data[i];
    }

    move_group_interface_ptr->setJointValueTarget(joint_group_positions);

    move_group_interface_ptr->setMaxVelocityScalingFactor(0.1);
    move_group_interface_ptr->setMaxAccelerationScalingFactor(0.1);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("JS POSITION", "plan (joint space goal) %s", success ? "" : "FAILED");
    bool success2 = (move_group_interface_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("JS POSITION", "execute (joint space goal) %s", success ? "" : "FAILED");
       
    if ((success == 1) && (success2 == 1)) {
      res.status = true;
      return true;
    }
    res.status = false;
    return false;
  }

  bool cart_service_handler(the_mainest::CartPosition::Request &req, the_mainest::CartPosition::Response &res)
  {
    move_group_interface_ptr->setPlannerId("RRTConnect");
    move_group_interface_ptr->setMaxAccelerationScalingFactor(0.5);
    move_group_interface_ptr->setMaxVelocityScalingFactor(0.5);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface_ptr->setPoseTarget(req.pose);
    move_group_interface_ptr->allowReplanning(true);

    bool success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success2 = (move_group_interface_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("CART POSITION", "plan (cart space goal) %s", success ? "" : "FAILED");

    if ((success == 1) && (success2 == 1)) {
      res.status = true;
      return true;
    }
    res.status = false;
    return false;

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_and_place");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  Pick_and_place_class go;
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}