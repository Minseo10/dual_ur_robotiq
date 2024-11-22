#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

//msgs
#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>

#define D2R M_PI/180
#define R2D 180/M_PI

shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z);
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow);
void graspsCallback(const agile_grasp::Grasps& msg);
std::string to_string(int value);
bool isInWorkspace(double x, double y, double zss);
std::vector<std::vector<double>> verticalScore(const agile_grasp::Grasps& grasps);
std::vector<std::vector<double>> apertureScore(const agile_grasp::Grasps& grasps);
std::vector<std::vector<double>> similarityScore(geometry_msgs::PoseArray grasps, geometry_msgs::Pose current);
bool compareScore(const std::vector<double>& v1, const std::vector<double>& v2);
bool compareScore2(const std::vector<double>& v1, const std::vector<double>& v2);
agile_grasp::Grasp rotateGrasp(const agile_grasp::Grasp& grasp, double angle);

geometry_msgs::PoseArray graspList; //list of grasp poses (x y y ox oy oz ow)
std::vector<std::vector<double>> width_scores;
std::vector<std::vector<double>> vertical_scores;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_perception");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::spin();
  ros::Publisher gripper_pub_left = n.advertise<std_msgs::Char>("left_gripper/gripper_left", 10);
  ros::Publisher gripper_pub_right = n.advertise<std_msgs::Char>("right_gripper/gripper_right", 10);
  ros::Subscriber grasps_sub = n.subscribe("find_grasps/grasps", 1000, graspsCallback); // subscribe grasp msgs


  // for visualizing grasp pose
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/world","/rviz_visual_markers"));
  visual_tools_->loadMarkerPub();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_(tf_buffer_);

  static const std::string PLANNING_GROUP_ARM_LEFT = "left_arm";
  moveit::planning_interface::MoveGroupInterface leftArm(PLANNING_GROUP_ARM_LEFT);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  leftArm.startStateMonitor(); //getCurrentState() error fix
  bool success;

  // for computing cartesian path
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction;
  moveit_msgs::RobotTrajectory trajectory;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(leftArm.getPlanningFrame().c_str());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(leftArm.getJointModelGroupNames().begin(),
  leftArm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  ROS_INFO_NAMED("tutorial", "\nEnd effector link: %s", leftArm.getEndEffectorLink().c_str());
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo\n");

  // activate left gripper
  std_msgs::Char gripper_msg_l, gripper_msg_r;
  gripper_msg_l.data = 'p'; // start with pinch mode
  gripper_pub_left.publish(gripper_msg_l);

  // change the base frame: camera_depth_optical_frame -> stand
  std::string robot_base = leftArm.getPoseReferenceFrame();
  geometry_msgs::PoseArray graspList_base; //grasp poses list w.r.t robot's base frame
  geometry_msgs::TransformStamped camera_to_base = tf_buffer_.lookupTransform(robot_base,
    "camera_depth_optical_frame", ros::Time(0));

  for(int i=0; i<graspList.poses.size() ; i++){
    geometry_msgs::PoseStamped graspPose_base; //grasp pose w.r.t robot's base frame
    graspPose_base.header.frame_id = robot_base;

    geometry_msgs::PoseStamped graspPose_cam; //grasp pose w.r.t camera frame
    graspPose_cam.header.frame_id = "camera_depth_optical_frame";
    graspPose_cam.pose = graspList.poses[i];
    tf2::doTransform(graspPose_cam, graspPose_base, camera_to_base);

    graspList_base.poses.push_back(graspPose_base.pose);
  }

  // 1. test motion
  geometry_msgs::PoseStamped current_pose_left = leftArm.getCurrentPose("left_gripper_tool0");
  std::cout << std::endl <<"current pos: " << current_pose_left.pose.position.x << ", " << current_pose_left.pose.position.y << ", " << current_pose_left.pose.position.z << std::endl;
  std::cout <<"ori: " << current_pose_left.pose.orientation.x << ", " << current_pose_left.pose.orientation.y << ", " << current_pose_left.pose.orientation.z << ", " << current_pose_left.pose.orientation.w << std::endl;

  geometry_msgs::Pose start_pose_left = current_pose_left.pose; //start pose

//  start_pose_left.position.x = -0.356859;
//  start_pose_left.position.y = -0.864936;
//  start_pose_left.position.z =  1.05599;
//  start_pose_left.orientation.x = -0.486791;
//  start_pose_left.orientation.y = 0.419529;
//  start_pose_left.orientation.z = 0.595481;
//  start_pose_left.orientation.w = 0.482113;

  leftArm.setStartStateToCurrentState();
  leftArm.setJointValueTarget("left_shoulder_pan_joint", 124*D2R);
  leftArm.setJointValueTarget("left_shoulder_lift_joint", -100*D2R);
  leftArm.setJointValueTarget("left_elbow_joint", 104*D2R);
  leftArm.setJointValueTarget("left_wrist_1_joint", -143*D2R);
  leftArm.setJointValueTarget("left_wrist_2_joint", 60*D2R);
  leftArm.setJointValueTarget("left_wrist_3_joint", 13*D2R);

    // bad example
//  leftArm.setStartStateToCurrentState();
//  leftArm.setJointValueTarget("left_shoulder_pan_joint", 160*D2R);
//  leftArm.setJointValueTarget("left_shoulder_lift_joint", -41*D2R);
//  leftArm.setJointValueTarget("left_elbow_joint", 24*D2R);
//  leftArm.setJointValueTarget("left_wrist_1_joint", -90*D2R);
//  leftArm.setJointValueTarget("left_wrist_2_joint", 47*D2R);
//  leftArm.setJointValueTarget("left_wrist_3_joint", -5*D2R);

  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)  ROS_INFO("Plan did not successed");
  else  leftArm.execute(my_plan);

  visual_tools.publishText(text_pose, "move test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("1. Press 'next' to move the left arm to the start pose\n");
  leftArm.move();


  // cosine similarity score
  std::vector<std::vector<double>> sim_scores = similarityScore(graspList_base, start_pose_left);
  std::sort(sim_scores.begin(), sim_scores.end(), compareScore2);

  // now sort the grasps
  geometry_msgs::PoseArray graspList_sorted;
  for(int i = 0; i < graspList_base.poses.size(); i++){
      graspList_sorted.poses.push_back(graspList_base.poses[sim_scores[i][0]]);
    }



  // Add Collision object
  ROS_INFO_NAMED("tutorial", "Add objects into the world");
  std::vector<moveit_msgs::CollisionObject> collision_objects; //list of collision objects
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = leftArm.getPlanningFrame();

  // table
  collision_object.id = "table";
  shape_msgs::SolidPrimitive primitive = setPrim(3, 0.6, 0.9, 0.04);
  geometry_msgs::Pose table_pose = setGeomPose(-0.55, -0.5, 0.785, 0.0, 1.0, 0.0, 0.0);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  // basket
  collision_object.id = "basket";
  primitive = setPrim(3, 0.49, 0.33, 0.13); //modified
  geometry_msgs::Pose basket_pose = setGeomPose(-0.5, -0.1, 0.86, 0.0, 0.0, 0.0, 1.0);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(basket_pose);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);

  // for all graspable objects (just to check whether grasp poses are correct)
//  for(int i=0; i < graspList_base.poses.size() ; i++){
//    //object_to_attach.header.frame_id = leftArm.getPlanningFrame();
//
//    moveit_msgs::CollisionObject object_to_attach;
//
//    object_to_attach.header.frame_id = leftArm.getPlanningFrame();
//    object_to_attach.id = "block" + to_string(i);
//    primitive = setPrim(3, 0.08, 0.08, 0.08);
//    geometry_msgs::Pose block_pose = graspList_base.poses[i]; //grasp poses - from msg
//
//    object_to_attach.primitives.push_back(primitive);
//    object_to_attach.primitive_poses.push_back(block_pose);
//    object_to_attach.operation = object_to_attach.ADD;
//    collision_objects.push_back(object_to_attach);
//  }

 planning_scene_interface.addCollisionObjects(collision_objects);

  visual_tools.publishText(text_pose, "Add objects", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("2. Press 'next' to see the best grasp pose\n");

  geometry_msgs::Pose init_pose_left = leftArm.getCurrentPose("left_gripper_tool0").pose;
  geometry_msgs::Pose pre_grasp_pose = init_pose_left;
  pre_grasp_pose = graspList_sorted.poses[0];

  // extract approach vector
  tf::Quaternion q; Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
  tf::quaternionMsgToTF(pre_grasp_pose.orientation, q);
  tf::Matrix3x3 tfR;
  tfR.setRotation(q);
  tf::matrixTFToEigen(tfR, R);

  Eigen::Vector3d approach = R.col(0);
  geometry_msgs::Vector3 approachVec;
  tf::vectorEigenToMsg(approach, approachVec);


  // visualize the grasp pose
  visual_tools_->publishAxis(pre_grasp_pose, 0.1);
  visual_tools_->trigger();

  // check the pose
  std::cout << std::endl <<"pre grasp pos: "<< pre_grasp_pose.position.x << ", " << pre_grasp_pose.position.y << ", " << pre_grasp_pose.position.z << std::endl;
  std::cout <<"ori: " << pre_grasp_pose.orientation.x << ", " << pre_grasp_pose.orientation.y << ", " << pre_grasp_pose.orientation.z << ", " << pre_grasp_pose.orientation.w << std::endl;

  std::vector<std::string> object_ids; //objects we will remove from collision objects
  // Remove all objects from the planning scene
//  for(int j=0; j<graspList_base.poses.size() ; j++){
//    std::string ith_id = "block" + to_string(j);
//    object_ids.push_back(ith_id);
//    planning_scene_interface.removeCollisionObjects(object_ids);
//  }

  // 3. execute the best grasp: Place the left EE above the wood block
  ROS_INFO("Executing grasp...");

  std::vector<geometry_msgs::Pose> linear_path;
  linear_path.push_back(pre_grasp_pose);
  leftArm.setEndEffectorLink("left_gripper_tool0");
  leftArm.setGoalOrientationTolerance(M_PI*20/180);
  fraction = leftArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory, false);
  my_plan.trajectory_ = trajectory;

//  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // move to pre_grasp_pose
  visual_tools.trigger();
  visual_tools.prompt("3. Press 'next' then the left arm will move to safe grasp pose\n");

  if (fraction == 1.0 ) leftArm.execute(my_plan);
  linear_path.clear();
  ros::Duration(3).sleep(); // wait for 3 sec


  // 4. move closer : move along approach vector
  geometry_msgs::Pose grasp_pose = pre_grasp_pose;
  grasp_pose.position.x += 0.08 * approachVec.x;
  grasp_pose.position.y += 0.08 * approachVec.y;
  grasp_pose.position.z += 0.08 * approachVec.z;

  linear_path.push_back(grasp_pose);
  leftArm.setEndEffectorLink("left_gripper_tool0");
  fraction = leftArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory, false);
  my_plan.trajectory_ = trajectory;

//  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  visual_tools.trigger();
  visual_tools.prompt("4. Press 'next' then the left arm will move closer to the object\n");

  std::cout << std::endl <<"grasp pos: "<< grasp_pose.position.x << ", " << grasp_pose.position.y << ", " << grasp_pose.position.z << std::endl;
  std::cout <<"ori: " << grasp_pose.orientation.x << ", " << grasp_pose.orientation.y << ", " << grasp_pose.orientation.z << ", " << grasp_pose.orientation.w << std::endl;

  if (fraction == 1.0 ) leftArm.execute(my_plan);
  linear_path.clear();
  ros::Duration(3).sleep(); // wait for 3 sec


  // 5. Close the left gripper
  visual_tools.trigger();
  visual_tools.prompt("5. Press 'next' to close the left gripper\n");
  gripper_msg_l.data = 'c';
  gripper_pub_left.publish(gripper_msg_l);

  // 6. pick up the block: move along z-axis
  geometry_msgs::Pose pose_left = leftArm.getCurrentPose("left_gripper_tool0").pose;
  pose_left.position.z += 0.20;
  linear_path.push_back(pose_left);
  leftArm.setEndEffectorLink("left_gripper_tool0");
  fraction = leftArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory, false);
  my_plan.trajectory_ = trajectory;

//  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  visual_tools.trigger();
  visual_tools.prompt("6. Press 'next' to pick up the object\n");

  if (fraction == 1.0 ) leftArm.execute(my_plan);
  linear_path.clear();
  ros::Duration(3).sleep();

  // remove basket from collision object
  std::vector<std::string> basket_id;
  basket_id.push_back("basket");
  planning_scene_interface.removeCollisionObjects(basket_id);

  // 7. Move the block above the basket: set basket position manually
  geometry_msgs::Pose drop_pose = leftArm.getCurrentPose("left_gripper_tool0").pose;
  drop_pose.position.x = -0.640338;
  drop_pose.position.y = -0.141369;
  drop_pose.position.z = 1.11277;
  drop_pose.orientation.x = -0.525852;
  drop_pose.orientation.y = 0.488792;
  drop_pose.orientation.z = 0.444622;
  drop_pose.orientation.w = 0.535605;

  std::cout << std::endl <<"drop pos: " << drop_pose.position.x << ", " << drop_pose.position.y << ", " << drop_pose.position.z << std::endl;
  std::cout <<"ori: " << drop_pose.orientation.x << ", " << drop_pose.orientation.y << ", " << drop_pose.orientation.z << ", " << drop_pose.orientation.w << std::endl;

  linear_path.push_back(drop_pose);
  leftArm.setEndEffectorLink("left_gripper_tool0");
  leftArm.setGoalTolerance(0.10); // sphere which radis is 10cm
  fraction = leftArm.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory, false);
  my_plan.trajectory_ = trajectory;

//  success = (leftArm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  visual_tools.trigger();
  visual_tools.prompt("7. Press 'next' then the left arm will move to the basket\n");

  if (fraction == 1.0 ) leftArm.execute(my_plan);
  linear_path.clear();
  ros::Duration(3).sleep();

  // 8. Open the left gripper: drop the object inside the basket and finish the execution
  visual_tools.trigger();
  visual_tools.prompt("8. Press 'next' to open\n");

  gripper_msg_l.data = 'o';
  gripper_pub_left.publish(gripper_msg_l);


  ROS_INFO("Grasp execution complete!");
  ros::shutdown();
  return 0;
}

// dimension & bounding box's size
shape_msgs::SolidPrimitive setPrim(int d, float x, float y, float z){
    shape_msgs::SolidPrimitive pr;
    pr.type = pr.BOX;
    pr.dimensions.resize(d);
    pr.dimensions[pr.BOX_X] = x;
    pr.dimensions[pr.BOX_Y] = y;
    pr.dimensions[pr.BOX_Z] = z;

    return pr;
}

// set position & orientation of the bounding box
geometry_msgs::Pose setGeomPose(float x, float y, float z, float ox, float oy, float oz, float ow){
    geometry_msgs::Pose p;

    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.x = ox;
    p.orientation.y = oy;
    p.orientation.z = oz;
    p.orientation.w = ow;

    return p;
}

// callback function: rotate grasps and create list of grasps
void graspsCallback(const agile_grasp::Grasps& grasps) {

    agile_grasp::Grasps grasps_added;
    grasps_added.header = grasps.header; grasps_added.grasps = grasps.grasps;

    // rotate grasp - M_PI/2, M_PI, M_PI*3/2
    for (int i = 0; i < grasps.grasps.size(); i++){
      agile_grasp::Grasp rotate90 = rotateGrasp(grasps.grasps[i], M_PI/2);
      agile_grasp::Grasp rotate180 = rotateGrasp(grasps.grasps[i], M_PI);
      agile_grasp::Grasp rotate270 = rotateGrasp(grasps.grasps[i], M_PI*3/2);

      grasps_added.grasps.push_back(rotate90);
      grasps_added.grasps.push_back(rotate180);
      grasps_added.grasps.push_back(rotate270);
    }

    // score grasps - vertical score, aperture score
    vertical_scores = verticalScore(grasps_added);
    std::sort(vertical_scores.begin(), vertical_scores.end(), compareScore2);

    width_scores = apertureScore(grasps_added);
    std::sort(width_scores.begin(), width_scores.end(), compareScore);

    // create grasp pose array
    graspList.poses[grasps.grasps.size()];
    graspList.header.frame_id = "camera_depth_optical_frame";
    graspList.header.stamp = ros::Time(0);
    //ROS_INFO("Received %d grasps!", grasps_added.grasps.size());
    for (int i = 0; i < grasps_added.grasps.size(); i++){
        const agile_grasp::Grasp& grasp = grasps_added.grasps[i];

        Eigen::Vector3d surfaceCenter;
        Eigen::Vector3d axis, approach, binormal, position;
        tf::vectorMsgToEigen(grasps_added.grasps[i].surface_center, surfaceCenter);
        tf::vectorMsgToEigen(grasps_added.grasps[i].axis, axis);
        tf::vectorMsgToEigen(grasps_added.grasps[i].approach, approach);

        surfaceCenter -= 0.05 * approach; //move along the approach vector
        binormal = approach.cross(axis);

        Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
        R.col(0) = approach;
        R.col(1) = axis;
        R.col(2) = binormal;

        tf::Matrix3x3 tfR;
        tf::matrixEigenToTF(R, tfR);
        tf::Quaternion q;
        tfR.getRotation(q);
        q.normalize();

        geometry_msgs::Pose graspPose;
        tf::pointEigenToMsg(surfaceCenter, graspPose.position);
        tf::quaternionTFToMsg(q, graspPose.orientation);

        graspList.poses.push_back(graspPose);
    }

}

// convert int to string
std::string to_string(int value){
  std::stringstream ss;
  ss << value;
  return ss.str();
}

// check is given position is in workspace
bool isInWorkspace(double x, double y, double z){
  if (x >= -0.6 && x <= -0.1 && y >= -0.35 && y <= 0.1	&& z >= 0.9 && z <= 1.45) {
		return true;
	}
	return false;
}

// rotate grasps with given angle
agile_grasp::Grasp rotateGrasp(const agile_grasp::Grasp& grasp, double angle){
    agile_grasp::Grasp rotatedGrasp;
    Eigen::Vector3d approach, axis;
    tf::vectorMsgToEigen(grasp.approach, approach);
    tf::vectorMsgToEigen(grasp.axis, axis);

   // rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
    Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(angle, approach));

    //rotate
    approach = T * approach;
    axis = T * axis;

    tf::vectorEigenToMsg(approach, rotatedGrasp.approach);
    tf::vectorEigenToMsg(axis, rotatedGrasp.axis);
    rotatedGrasp.center = grasp.center;
    rotatedGrasp.surface_center = grasp.surface_center;
    rotatedGrasp.width = grasp.width;

    return rotatedGrasp;
}

// calculate how much the approach vector is close to the z-axis
std::vector<std::vector<double>> verticalScore(const agile_grasp::Grasps& grasps){

  std::vector<std::vector<double>> scores;

  geometry_msgs::Vector3 z; // z-axis of camera_depth_optical_frame
  z.x = -0.6346780;
  z.y = 0.0168829;
  z.z = -0.7725922;

  for(int i = 0; i < grasps.grasps.size(); i++){
    std::vector<double> score(2);

    geometry_msgs::Vector3 n;
    n = grasps.grasps[i].approach;
    double dot = n.x * z.x + n.y * z.y + n.z * z.z;

    score[0] = i;
    score[1] = dot;
    scores.push_back(score);
  }
  return scores;
}

// calculate whether the grasp width is smaller than the robot hand aperture
std::vector<std::vector<double>> apertureScore(const agile_grasp::Grasps& grasps)
{
	std::vector<std::vector<double>> scores;
	for (int i = 0; i < grasps.grasps.size(); i++)
	{
		std::vector<double> score(2);
		score[0] = i; // save index of grasp
		float wid = grasps.grasps[i].width.data;
		double min = std::min(std::fabs(wid - 0.00), std::fabs(wid - 0.12));
		score[1] = std::max(0.00, 0.12 - min); //aperture score
		scores.push_back(score);
	}
	return scores;
}

// calculate cosine similarity btw 'left_gripper_tool0' and 'pre_grasp_pose'
std::vector<std::vector<double>> similarityScore(geometry_msgs::PoseArray grasps, geometry_msgs::Pose current){

  std::vector<std::vector<double>> scores;

  for(int i = 0; i < grasps.poses.size(); i++){
    std::vector<double> score(2);
    double sim = grasps.poses[i].orientation.x * current.orientation.x + grasps.poses[i].orientation.y * current.orientation.y
              + grasps.poses[i].orientation.z * current.orientation.z + grasps.poses[i].orientation.w * current.orientation.w;

    score[0] = i;
    score[1] = sim;
    scores.push_back(score);
  }
  return scores;
}

// ascending order
bool compareScore(const std::vector<double>& v1, const std::vector<double>& v2)
{
  return (v1[1] < v2[1]);
}

// descending order
bool compareScore2(const std::vector<double>& v1, const std::vector<double>& v2)
{
  return (v1[1] > v2[1]);
}
