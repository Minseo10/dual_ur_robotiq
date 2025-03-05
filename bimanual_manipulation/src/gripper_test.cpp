#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle n;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "left_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "left_gripper";
    bool success;

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
    move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    std::vector<std::string> gripper_states = move_group_interface_gripper.getNamedTargets();
    std::cout << "Gripper states names: ";
    for (const auto& state : gripper_states) {
        std::cout << state << " ";
    }
    std::cout << std::endl;

    // 3. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("o"));
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_gripper.move();

    // 5. Close the  gripper
//    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));
//    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//
//    move_group_interface_gripper.move();
//
//    // 8. Open the gripper
//    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
//    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//
//    move_group_interface_gripper.move();

  ros::shutdown();
  return 0;
}
