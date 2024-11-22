#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drawing");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM_R = "left_arm";
  static const std::string PLANNING_GROUP_GRIPPER_R = "left_gripper";
  static const std::string PLANNING_GROUP_ARM_L = "right_arm";
  static const std::string PLANNING_GROUP_GRIPPER_L = "right_gripper";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface rightArm(PLANNING_GROUP_ARM_R);
  moveit::planning_interface::MoveGroupInterface rightGripper(PLANNING_GROUP_GRIPPER_R);
  moveit::planning_interface::MoveGroupInterface leftArm(PLANNING_GROUP_ARM_L);
  moveit::planning_interface::MoveGroupInterface leftGripper(PLANNING_GROUP_GRIPPER_L);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_r;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm_l;


  bool success;

  // init pose
  // set all the joint values to the init joint position
  rightArm.setStartStateToCurrentState();
  rightArm.setJointValueTarget("left_shoulder_pan_joint", -0.26179); //-15
  rightArm.setJointValueTarget("left_shoulder_lift_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("left_elbow_joint", 1.91986); //110
  rightArm.setJointValueTarget("left_wrist_1_joint", -1.3962634); //-80
  rightArm.setJointValueTarget("left_wrist_2_joint", 2.35619); //135
  rightArm.setJointValueTarget("left_wrist_3_joint", -0.523599);  //-30
  success = (rightArm.plan(my_plan_arm_r) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  rightArm.execute(my_plan_arm_r);

  leftArm.setStartStateToCurrentState();
  leftArm.setJointValueTarget("right_shoulder_pan_joint", 0.26179); //15
  leftArm.setJointValueTarget("right_shoulder_lift_joint", -1.74532925); //-100
  leftArm.setJointValueTarget("right_elbow_joint", -1.91986); //-110
  leftArm.setJointValueTarget("right_wrist_1_joint", -1.3962634); //-80
  leftArm.setJointValueTarget("right_wrist_2_joint", -2.35619); //-135
  leftArm.setJointValueTarget("right_wrist_3_joint", -1.0472);  //-60
  success = (leftArm.plan(my_plan_arm_l) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  leftArm.execute(my_plan_arm_l);
  ros::Duration(5).sleep(); // wait for 3 sec


  ros::shutdown();
  return 0;
}

