#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuple>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include "pm_moveit_interfaces/srv/execute_plan.hpp"
#include "pm_moveit_interfaces/srv/move_cam1_tcp_to.hpp"
#include "pm_moveit_interfaces/srv/move_laser_tcp_to.hpp"
#include "pm_moveit_interfaces/srv/move_tool_tcp_to.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"


geometry_msgs::msg::Quaternion quaternion_multiply(geometry_msgs::msg::Quaternion q0, geometry_msgs::msg::Quaternion q1){
  // Extract the values from q0
  auto w0 = q0.w;
  auto x0 = q0.x;
  auto y0 = q0.y;
  auto z0 = q0.z;

  // Extract the values from q1
  auto w1 = q1.w;
  auto x1 = q1.x;
  auto y1 = q1.y;
  auto z1 = q1.z;

  // Computer the product of the two quaternions, term by term
  auto q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
  auto q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
  auto q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
  auto q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;

  auto result = geometry_msgs::msg::Quaternion();
  result.x = q0q1_x;
  result.y = q0q1_y;
  result.z = q0q1_z;
  result.w = q0q1_w;

  return result;
}


class PmMoveitServer : public rclcpp::Node
{
public:

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> laser_move_group;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Cam1_move_group;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> tool_move_group;
  
  rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service;
  rclcpp::Service<pm_moveit_interfaces::srv::MoveCam1TcpTo>::SharedPtr move_cam_one_service;
  rclcpp::Service<pm_moveit_interfaces::srv::MoveToolTcpTo>::SharedPtr move_tool_service;
  rclcpp::Service<pm_moveit_interfaces::srv::MoveLaserTcpTo>::SharedPtr move_laser_service;

  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> laser_grp_visual_tools;

  std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  sensor_msgs::msg::JointState::SharedPtr global_joint_state;
  // create shared pointer to node publisher
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> xyz_trajectory_publisher;
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> t_trajectory_publisher;

  PmMoveitServer()
  : Node("pm_moveit_server")
  {

    // RCLCPP_INFO(this->get_logger(), "Ready for operation1...");
    // plan = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
    // laser_grp_visual_tools =  std::make_shared<moveit_visual_tools::MoveItVisualTools>(this->shared_from_this(), "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
    // laser_grp_visual_tools->deleteAllMarkers();
    // laser_grp_visual_tools->loadRemoteControl();
    // PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(),"robot_description");
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    xyz_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_xyz_axis_controller/joint_trajectory", 10);
    t_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_t_axis_controller/joint_trajectory", 10);
    auto joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&PmMoveitServer::jointStateCallback, this, std::placeholders::_1));
    execute_plan_service = this->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&PmMoveitServer::execute_plan, this, std::placeholders::_1, std::placeholders::_2));
    move_cam_one_service = this->create_service<pm_moveit_interfaces::srv::MoveCam1TcpTo>("pm_moveit_server/move_cam1_to_frame", std::bind(&PmMoveitServer::move_group_cam1, this, std::placeholders::_1, std::placeholders::_2));
    move_tool_service = this->create_service<pm_moveit_interfaces::srv::MoveToolTcpTo>("pm_moveit_server/move_tool_to_frame", std::bind(&PmMoveitServer::move_group_tool, this, std::placeholders::_1, std::placeholders::_2));
    move_laser_service = this->create_service<pm_moveit_interfaces::srv::MoveLaserTcpTo>("pm_moveit_server/move_laser_to_frame", std::bind(&PmMoveitServer::move_group_laser, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready for operation...");
  }

std::tuple<bool, std::vector<std::string>, std::vector<double>> exec_move_group_service(std::string planning_group,
                                                                           std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                           std::string frame_name,
                                                                           geometry_msgs::msg::Pose move_to_pose,
                                                                           geometry_msgs::msg::Vector3 translation,
                                                                           geometry_msgs::msg::Quaternion rotation,
                                                                           bool exec_wait_for_user_input,
                                                                           bool execute)
{

  
  std::string endeffector = move_group->getEndEffectorLink();
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Endeffector Link: %s", endeffector.c_str());

  const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState(1.0)->getJointModelGroup(planning_group);

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  
  const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();
  auto state = moveit::core::RobotState(kinematic_model);

  geometry_msgs::msg::Pose target_pose;
  std::vector<double> target_joint_values;
  bool service_success;
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Model frame: %s", kinematic_model->getModelFrame().c_str());

  geometry_msgs::msg::Quaternion target_rotation;

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Retrieved Rotation x: %f, y: %f, z: %f, w: %f", rotation.x, rotation.y, rotation.z, rotation.w);

  // if rotation is not specified
  if (rotation.w == 0.0 && rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0)
  {
    rotation.w = 1;
  }

  // if rotation is not specified
  if (move_to_pose.orientation.w == 0.0 && move_to_pose.orientation.x == 0.0 && move_to_pose.orientation.y == 0.0 && move_to_pose.orientation.z == 0.0)
  {
    move_to_pose.orientation.w = 1;
  }

  geometry_msgs::msg::TransformStamped frame_transform;

  // if target position is (0,0,0) target pose is set the the endeffector; using the translation, this can be used for relative movement
  if ((move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0) && frame_name == "")
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "No pose or frame given! Executing relative movement with: x: %f, y: %f, z: %f", translation.x, translation.y, translation.z);
    try
    {
      frame_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);
      target_pose.position.x = frame_transform.transform.translation.x + translation.x;
      target_pose.position.y = frame_transform.transform.translation.y + translation.y;
      target_pose.position.z = frame_transform.transform.translation.z + translation.z;
      target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
      
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_FATAL(rclcpp::get_logger("pm_moveit"), "Could not transform %s to 'world': %s", endeffector.c_str(), ex.what());
      service_success = false;
      return {service_success, joint_names, target_joint_values};
    }
  }
  // if frame_name is empty
  else if (frame_name == "")
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame not given. Considering given Pose!");

    target_pose.position.x = move_to_pose.position.x + translation.x;
    target_pose.position.y = move_to_pose.position.y + translation.y;
    target_pose.position.z = move_to_pose.position.z + translation.z;
    target_pose.orientation = quaternion_multiply(move_to_pose.orientation, rotation);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame given. Ignoring given Pose!");
    std::string fromFrameRel = frame_name;
    std::string toFrameRel = "world";
    try
    {
      geometry_msgs::msg::TransformStamped frame_transform;
      frame_transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

      target_pose.position.x = frame_transform.transform.translation.x + translation.x;
      target_pose.position.y = frame_transform.transform.translation.y + translation.y;
      target_pose.position.z = frame_transform.transform.translation.z + translation.z;
      target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
      // target_pose.orientation.x = 0;
      // target_pose.orientation.y = 0;
      // target_pose.orientation.z = 0;
      // target_pose.orientation.w = 1;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      service_success = false;
      return {service_success, joint_names, target_joint_values};
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Calculated Endeffector Pose:");
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X %f", target_pose.position.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y %f", target_pose.position.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z %f", target_pose.position.z);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation W %f", target_pose.orientation.w);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation X %f", target_pose.orientation.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Y %f", target_pose.orientation.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Z %f", target_pose.orientation.z);

  double timeout = 0.1;
  // Calculate Inverse Kinematik Solution
  bool success_found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

  robot_state->updateLinkTransforms();

  const Eigen::Isometry3d end_effector_state = robot_state->getGlobalLinkTransform(endeffector);
  tf2::Transform tf2Transform;
  //tf2::doTransform(end_effector_state, end_effector_state, planned_eef_pose)
  tf2::convert(end_effector_state,tf2Transform);
  tf2::Vector3 endeffector_pose_planed = tf2Transform.getOrigin();

  // This is the same as the calculated Endeffector Pose
  //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose X %f", endeffector_pose_planed.getX());
  //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Y %f", endeffector_pose_planed.getY());
  //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Z %f", endeffector_pose_planed.getZ());

  bool success_calculate_plan = false;

  if (exec_wait_for_user_input)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "YES");
  }

  if (success_found_ik)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "IK solution found!");
    robot_state->copyJointGroupPositions(joint_model_group, target_joint_values);

    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Joint Values for %s: %f", joint_names[i].c_str(), target_joint_values[i]);
    }
    move_group->setPlanningTime(20);
    //move_group->setGoalPositionTolerance(1e-9); // 10 nm    
    move_group->setGoalPositionTolerance(0.000000001); // 1 nm    
    move_group->setStartStateToCurrentState();
    move_group->setJointValueTarget(target_joint_values);
    move_group->setNumPlanningAttempts(100);

    success_calculate_plan = (move_group->plan(*plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success_calculate_plan)
    {

      service_success = true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Planing failed!");
      service_success = false;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Did not find IK solution");
    service_success = false;
  }
  laser_grp_visual_tools->deleteAllMarkers();
  auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
  laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
  laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
  //laser_grp_visual_tools->prompt("next step");
  laser_grp_visual_tools->trigger();

  // Execute the plan
  if (success_calculate_plan && execute)
  {
    move_group->execute(*plan);
    // Checking delta;
    try
    {
      float delta = 0.0005;  // in meters
      while (check_goal_reached(joint_names, target_joint_values, 0.0005, 0.01) == false)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
      }
      auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      trajectory_msg->joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"}; // Specify joint names

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2]};  // Specify joint positions
      point.velocities = {0.0, 0.0, 0.0}; // Specify joint velocities
      point.accelerations = {0.0, 0.0, 0.0}; // Specify joint accelerations
      point.time_from_start.sec = 0.5; // Specify duration
      trajectory_msg->points.push_back(point);
      xyz_trajectory_publisher->publish(*trajectory_msg);

      // If planning group is PM_Robot_Tool_TCP, publish T_Axis_Joint trajectory
      if (planning_group == "PM_Robot_Tool_TCP")
      {
        auto t_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        t_trajectory_msg->joint_names = {"T_Axis_Joint"}; // Specify joint names
        trajectory_msgs::msg::JointTrajectoryPoint t_point;
        t_point.positions = {target_joint_values[3]};  // Specify joint positions
        t_point.velocities = {0.0}; // Specify joint velocities
        t_point.accelerations = {0.0}; // Specify joint accelerations
        t_point.time_from_start.sec = 0.5; // Specify duration
        t_trajectory_msg->points.push_back(t_point);
        t_trajectory_publisher->publish(*t_trajectory_msg);
      }

      while (check_goal_reached(joint_names, target_joint_values, 0.0000001, 0.0001) == false)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
      }

      geometry_msgs::msg::TransformStamped moved_to_transform;
      geometry_msgs::msg::Pose moved_to_pose;

      moved_to_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);

      moved_to_pose.position.x = moved_to_transform.transform.translation.x;
      moved_to_pose.position.y = moved_to_transform.transform.translation.y;
      moved_to_pose.position.z = moved_to_transform.transform.translation.z;

      RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Moved to Endeffector Pose: ");
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "X: %f", moved_to_pose.position.x);
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Y: %f", moved_to_pose.position.y);
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Z: %f", moved_to_pose.position.z);
      
      // auto this_pose = move_group->getCurrentPose(endeffector);

      // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X4 %f", this_pose.pose.position.x);
      // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y4 %f", this_pose.pose.position.y);
      // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z4 %f", this_pose.pose.position.z);

      // double deltaX = moved_to_pose.position.x - target_pose.position.x;
      // double deltaY = moved_to_pose.position.y - target_pose.position.y;
      // double deltaZ = moved_to_pose.position.z - target_pose.position.z;
      
      double deltaX = endeffector_pose_planed.getX() - moved_to_pose.position.x;
      double deltaY = endeffector_pose_planed.getY() - moved_to_pose.position.y;
      double deltaZ = endeffector_pose_planed.getZ() - moved_to_pose.position.z;
      RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Pose Deltas: ");
      RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "X: %f um", deltaX * 1000000);
      RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Y: %f um", deltaY * 1000000);
      RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Z: %f um", deltaZ * 1000000);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Transform Error !!!");
    }
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan not executed!");
  }
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");
  return {service_success, joint_names, target_joint_values};
}


  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    //Process the received joint state message
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::cout << "Joint Name: " << msg->name[i] << ", Position: " << msg->position[i] << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Joint Name: %s, Value: %f",msg->name[i].c_str(), msg->position[i]);
    }
    global_joint_state = msg;
  }

  void move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Response> response)
  {

    auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Cam1_TCP",
                                                         Cam1_move_group,
                                                         request->frame_name,
                                                         request->move_to_pose,
                                                         request->translation,
                                                         request->rotation,
                                                         request->exec_wait_for_user_input,
                                                         request->execute);

    response->success = success;
    response->joint_names = joint_names;
    std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
    response->joint_values = joint_values_float;

    return;
  }

  void move_group_tool(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Response> response)
  {

    auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Tool_TCP",
                                                         tool_move_group,
                                                         request->frame_name,
                                                         request->move_to_pose,
                                                         request->translation,
                                                         request->rotation,
                                                         request->exec_wait_for_user_input,
                                                         request->execute);

    response->success = success;
    response->joint_names = joint_names;
    std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
    response->joint_values = joint_values_float;

    return;
  }

  void move_group_laser(const std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Request> request,
                      std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Response> response)
  {

    auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Laser_TCP",
                                                         laser_move_group,
                                                         request->frame_name,
                                                         request->move_to_pose,
                                                         request->translation,
                                                         request->rotation,
                                                         request->exec_wait_for_user_input,
                                                         request->execute);

    response->success = success;
    response->joint_names = joint_names;
    std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
    response->joint_values = joint_values_float;

    return;
  }

  bool execute_plan(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request,
                  std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response)
  {
    if (request->run){
      RCLCPP_INFO(this->get_logger(), "Yes");
      response->success = true;
    }

    return true;
  }

  bool check_goal_reached(std::vector<std::string> target_joints, std::vector<double> target_joint_values, float delta_trans, float delta_rot)
  {
    float delta_value;
    for (size_t i = 0; i < target_joints.size(); i++)
    {
      if (target_joints[i] == "T_Axis_Joint")
      {
        // This means rotation
        delta_value = delta_rot;
      }
      else
      {
        // This means translation
        delta_value = delta_trans;
      }

      // find joint in joint state
      auto it = std::find(global_joint_state->name.begin(), global_joint_state->name.end(), target_joints[i]);
      if (it == global_joint_state->name.end()) {
          // Joint not found
          // Handle error or return false
          return false;
      }
      int current_joint_index = std::distance(global_joint_state->name.begin(), it);
      float current_joint_value = global_joint_state->position[current_joint_index];
      float differrence = std::abs(current_joint_value - target_joint_values[i]);
      
      RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint: %s, Target: %f, Current: %f, Delta: %f", target_joints[i].c_str(), target_joint_values[i], current_joint_value, differrence);
      
      
      if (differrence > delta_value)
      {
        return false;
      }
    }    
    return true;
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto pm_moveit_server_node = std::make_shared<PmMoveitServer>();
  pm_moveit_server_node->laser_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Laser_TCP");
  pm_moveit_server_node->Cam1_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Cam1_TCP");
  pm_moveit_server_node->tool_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Tool_TCP");
  pm_moveit_server_node->laser_grp_visual_tools =  std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,pm_moveit_server_node->laser_move_group->getRobotModel());
  pm_moveit_server_node->laser_grp_visual_tools->deleteAllMarkers();
  pm_moveit_server_node->laser_grp_visual_tools->loadRemoteControl();
  pm_moveit_server_node->PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(pm_moveit_server_node,"robot_description");


  //auto callback_group_re = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  //rclcpp::SubscriptionOptions node_options_1;
  //node_options_1.callback_group = callback_group_re;

  //auto callback_group_me = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //rclcpp::SubscriptionOptions node_options_2;
  //node_options_2.callback_group = callback_group_me;

  //rclcpp::ExecutorOptions exec_options;
  //exec_options->num_threads = 4;

  //auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pm_moveit_server_node);
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}












// // Global Variables
// std::shared_ptr<moveit::planning_interface::MoveGroupInterface> laser_move_group;
// std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Cam1_move_group;
// std::shared_ptr<moveit::planning_interface::MoveGroupInterface> tool_move_group;

// std::shared_ptr<moveit_visual_tools::MoveItVisualTools> laser_grp_visual_tools;

// std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
// std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan;
// std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
// std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
// sensor_msgs::msg::JointState::SharedPtr global_joint_state;
// // create shared pointer to node publisher
// std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> xyz_trajectory_publisher;
// std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> t_trajectory_publisher;

// bool execute_plan(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request,
//                   std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response)
// {
//   if (request->run){
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Yes");
//     response->success = true;
//   }

//   return true;
// }

// geometry_msgs::msg::Quaternion quaternion_multiply(geometry_msgs::msg::Quaternion q0, geometry_msgs::msg::Quaternion q1){
//   // Extract the values from q0
//   auto w0 = q0.w;
//   auto x0 = q0.x;
//   auto y0 = q0.y;
//   auto z0 = q0.z;

//   // Extract the values from q1
//   auto w1 = q1.w;
//   auto x1 = q1.x;
//   auto y1 = q1.y;
//   auto z1 = q1.z;

//   // Computer the product of the two quaternions, term by term
//   auto q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
//   auto q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
//   auto q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
//   auto q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;

//   auto result = geometry_msgs::msg::Quaternion();
//   result.x = q0q1_x;
//   result.y = q0q1_y;
//   result.z = q0q1_z;
//   result.w = q0q1_w;

//   return result;
// }

// void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
//     // Process the received joint state message
//     // For example, print the names and positions of the joints
//     for (size_t i = 0; i < msg->name.size(); ++i) {
//         std::cout << "Joint Name: " << msg->name[i] << ", Position: " << msg->position[i] << std::endl;
//         //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Joint Name: %s, Value: %f",msg->name[i].c_str(), msg->position[i]);
//     }
//     global_joint_state = msg;
// }

// bool check_goal_reached(std::vector<std::string> target_joints, std::vector<double> target_joint_values, float delta_trans, float delta_rot)
// {
//   float delta_value;
//   for (size_t i = 0; i < target_joints.size(); i++)
//   {
//     if (target_joints[i] == "T_Axis_Joint")
//     {
//       // This means rotation
//       delta_value = delta_rot;
//     }
//     else
//     {
//       // This means translation
//       delta_value = delta_trans;
//     }

//     // find joint in joint state
//     auto it = std::find(global_joint_state->name.begin(), global_joint_state->name.end(), target_joints[i]);
//     if (it == global_joint_state->name.end()) {
//         // Joint not found
//         // Handle error or return false
//         return false;
//     }
//     int current_joint_index = std::distance(global_joint_state->name.begin(), it);
//     float current_joint_value = global_joint_state->position[current_joint_index];
//     float differrence = std::abs(current_joint_value - target_joint_values[i]);
    
//     RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint: %s, Target: %f, Current: %f, Delta: %f", target_joints[i].c_str(), target_joint_values[i], current_joint_value, differrence);
    
    
//     if (differrence > delta_value)
//     {
//       return false;
//     }
//   }    
//   return true;
// }

// std::tuple<bool, std::vector<std::string>, std::vector<double>> exec_move_group_service(std::string planning_group,
//                                                                            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
//                                                                            std::string frame_name,
//                                                                            geometry_msgs::msg::Pose move_to_pose,
//                                                                            geometry_msgs::msg::Vector3 translation,
//                                                                            geometry_msgs::msg::Quaternion rotation,
//                                                                            bool exec_wait_for_user_input,
//                                                                            bool execute)
// {

  
//   std::string endeffector = move_group->getEndEffectorLink();
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Endeffector Link: %s", endeffector.c_str());

//   const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState(1.0)->getJointModelGroup(planning_group);

//   const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  
//   const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();

//   moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
//   robot_state->setToDefaultValues();
//   auto state = moveit::core::RobotState(kinematic_model);

//   geometry_msgs::msg::Pose target_pose;
//   std::vector<double> target_joint_values;
//   bool service_success;
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//   geometry_msgs::msg::Quaternion target_rotation;

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Retrieved Rotation x: %f, y: %f, z: %f, w: %f", rotation.x, rotation.y, rotation.z, rotation.w);

//   // if rotation is not specified
//   if (rotation.w == 0.0 && rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0)
//   {
//     rotation.w = 1;
//   }

//   // if rotation is not specified
//   if (move_to_pose.orientation.w == 0.0 && move_to_pose.orientation.x == 0.0 && move_to_pose.orientation.y == 0.0 && move_to_pose.orientation.z == 0.0)
//   {
//     move_to_pose.orientation.w = 1;
//   }

//   geometry_msgs::msg::TransformStamped frame_transform;

//   // if target position is (0,0,0) target pose is set the the endeffector; using the translation, this can be used for relative movement
//   if ((move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0) && frame_name == "")
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "No pose or frame given! Executing relative movement with: x: %f, y: %f, z: %f", translation.x, translation.y, translation.z);
//     try
//     {
//       frame_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);
//       target_pose.position.x = frame_transform.transform.translation.x + translation.x;
//       target_pose.position.y = frame_transform.transform.translation.y + translation.y;
//       target_pose.position.z = frame_transform.transform.translation.z + translation.z;
//       target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
      
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_FATAL(rclcpp::get_logger("pm_moveit"), "Could not transform %s to 'world': %s", endeffector.c_str(), ex.what());
//       service_success = false;
//       return {service_success, joint_names, target_joint_values};
//     }
//   }
//   // if frame_name is empty
//   else if (frame_name == "")
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame not given. Considering given Pose!");

//     target_pose.position.x = move_to_pose.position.x + translation.x;
//     target_pose.position.y = move_to_pose.position.y + translation.y;
//     target_pose.position.z = move_to_pose.position.z + translation.z;
//     target_pose.orientation = quaternion_multiply(move_to_pose.orientation, rotation);
//   }
//   else
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame given. Ignoring given Pose!");
//     std::string fromFrameRel = frame_name;
//     std::string toFrameRel = "world";
//     try
//     {
//       geometry_msgs::msg::TransformStamped frame_transform;
//       frame_transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

//       target_pose.position.x = frame_transform.transform.translation.x + translation.x;
//       target_pose.position.y = frame_transform.transform.translation.y + translation.y;
//       target_pose.position.z = frame_transform.transform.translation.z + translation.z;
//       target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
//       // target_pose.orientation.x = 0;
//       // target_pose.orientation.y = 0;
//       // target_pose.orientation.z = 0;
//       // target_pose.orientation.w = 1;
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
//       service_success = false;
//       return {service_success, joint_names, target_joint_values};
//     }
//   }

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Calculated Endeffector Pose:");
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X %f", target_pose.position.x);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y %f", target_pose.position.y);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z %f", target_pose.position.z);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation W %f", target_pose.orientation.w);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation X %f", target_pose.orientation.x);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Y %f", target_pose.orientation.y);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Z %f", target_pose.orientation.z);

//   double timeout = 0.1;
//   // Calculate Inverse Kinematik Solution
//   bool success_found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

//   robot_state->updateLinkTransforms();

//   const Eigen::Isometry3d end_effector_state = robot_state->getGlobalLinkTransform(endeffector);
//   tf2::Transform tf2Transform;
//   //tf2::doTransform(end_effector_state, end_effector_state, planned_eef_pose)
//   tf2::convert(end_effector_state,tf2Transform);
//   tf2::Vector3 endeffector_pose_planed = tf2Transform.getOrigin();

//   // This is the same as the calculated Endeffector Pose
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose X %f", endeffector_pose_planed.getX());
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Y %f", endeffector_pose_planed.getY());
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Z %f", endeffector_pose_planed.getZ());

//   bool success_calculate_plan = false;

//   if (exec_wait_for_user_input)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "YES");
//   }

//   if (success_found_ik)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "IK solution found!");
//     robot_state->copyJointGroupPositions(joint_model_group, target_joint_values);

//     for (std::size_t i = 0; i < joint_names.size(); ++i)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Joint Values for %s: %f", joint_names[i].c_str(), target_joint_values[i]);
//     }
//     move_group->setPlanningTime(20);
//     //move_group->setGoalPositionTolerance(1e-9); // 10 nm    
//     move_group->setGoalPositionTolerance(0.000000001); // 1 nm    
//     move_group->setStartStateToCurrentState();
//     move_group->setJointValueTarget(target_joint_values);
//     move_group->setNumPlanningAttempts(100);

//     success_calculate_plan = (move_group->plan(*plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success_calculate_plan)
//     {

//       service_success = true;
//     }
//     else
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Planing failed!");
//       service_success = false;
//     }
//   }
//   else
//   {
//     RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Did not find IK solution");
//     service_success = false;
//   }
//   laser_grp_visual_tools->deleteAllMarkers();
//   auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
//   laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
//   laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
//   //laser_grp_visual_tools->prompt("next step");
//   laser_grp_visual_tools->trigger();

//   // Execute the plan
//   if (success_calculate_plan && execute)
//   {
//     move_group->execute(*plan);
//     // Checking delta;
//     try
//     {
//       float delta = 0.0005;  // in meters
//       while (check_goal_reached(joint_names, target_joint_values, 0.0005, 0.01) == false)
//       {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
//       }
//       auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
//       trajectory_msg->joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"}; // Specify joint names

//       trajectory_msgs::msg::JointTrajectoryPoint point;
//       point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2]};  // Specify joint positions
//       point.velocities = {0.0, 0.0, 0.0}; // Specify joint velocities
//       point.accelerations = {0.0, 0.0, 0.0}; // Specify joint accelerations
//       point.time_from_start.sec = 0.5; // Specify duration
//       trajectory_msg->points.push_back(point);
//       xyz_trajectory_publisher->publish(*trajectory_msg);

//       // If planning group is PM_Robot_Tool_TCP, publish T_Axis_Joint trajectory
//       if (planning_group == "PM_Robot_Tool_TCP")
//       {
//         auto t_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
//         t_trajectory_msg->joint_names = {"T_Axis_Joint"}; // Specify joint names
//         trajectory_msgs::msg::JointTrajectoryPoint t_point;
//         t_point.positions = {target_joint_values[3]};  // Specify joint positions
//         t_point.velocities = {0.0}; // Specify joint velocities
//         t_point.accelerations = {0.0}; // Specify joint accelerations
//         t_point.time_from_start.sec = 0.5; // Specify duration
//         t_trajectory_msg->points.push_back(t_point);
//         t_trajectory_publisher->publish(*t_trajectory_msg);
//       }

//       while (check_goal_reached(joint_names, target_joint_values, 0.0000001, 0.0001) == false)
//       {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
//       }

//       geometry_msgs::msg::TransformStamped moved_to_transform;
//       geometry_msgs::msg::Pose moved_to_pose;

//       moved_to_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);

//       moved_to_pose.position.x = moved_to_transform.transform.translation.x;
//       moved_to_pose.position.y = moved_to_transform.transform.translation.y;
//       moved_to_pose.position.z = moved_to_transform.transform.translation.z;

//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Moved to Endeffector Pose: ");
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "X: %f", moved_to_pose.position.x);
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Y: %f", moved_to_pose.position.y);
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Z: %f", moved_to_pose.position.z);
      
//       // auto this_pose = move_group->getCurrentPose(endeffector);

//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X4 %f", this_pose.pose.position.x);
//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y4 %f", this_pose.pose.position.y);
//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z4 %f", this_pose.pose.position.z);

//       // double deltaX = moved_to_pose.position.x - target_pose.position.x;
//       // double deltaY = moved_to_pose.position.y - target_pose.position.y;
//       // double deltaZ = moved_to_pose.position.z - target_pose.position.z;
      
//       double deltaX = endeffector_pose_planed.getX() - moved_to_pose.position.x;
//       double deltaY = endeffector_pose_planed.getY() - moved_to_pose.position.y;
//       double deltaZ = endeffector_pose_planed.getZ() - moved_to_pose.position.z;
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Pose Deltas: ");
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "X: %f um", deltaX * 1000000);
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Y: %f um", deltaY * 1000000);
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Z: %f um", deltaZ * 1000000);
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Transform Error !!!");
//     }
//   }
//   else
//   {
//     RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan not executed!");
//   }
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");
//   return {service_success, joint_names, target_joint_values};
// }




// void move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Request> request,
//                      std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Response> response)
// {

//   auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Cam1_TCP",
//                                                          Cam1_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//   response->success = success;
//   response->joint_names = joint_names;
//   std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//   response->joint_values = joint_values_float;

//   return;
// }

// void move_group_tool(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Request> request,
//                      std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Response> response)
// {

//   auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Tool_TCP",
//                                                          tool_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//   response->success = success;
//   response->joint_names = joint_names;
//   std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//   response->joint_values = joint_values_float;

//   return;
// }

// void move_group_laser(const std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Request> request,
//                       std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Response> response)
// {

//   auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Laser_TCP",
//                                                          laser_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//   response->success = success;
//   response->joint_names = joint_names;
//   std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//   response->joint_values = joint_values_float;

//   return;
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   auto const pm_moveit_server_node = std::make_shared<rclcpp::Node>(
//       "pm_moveit_server",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   auto callback_group_re = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//   rclcpp::SubscriptionOptions node_options_1;
//   node_options_1.callback_group = callback_group_re;

//   auto callback_group_me = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//   rclcpp::SubscriptionOptions node_options_2;
//   node_options_2.callback_group = callback_group_me;

//   rclcpp::ExecutorOptions exec_options;
//   //exec_options->num_threads = 4;

//   //auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();


//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(pm_moveit_server_node);
//   // auto spin_thread = std::make_unique<std::thread>([&executor, &pm_moveit_server_node]()
//   //                                                  {
//   //   executor.add_node(pm_moveit_server_node);
//   //   executor.spin();
//   //   executor.remove_node(pm_moveit_server_node); });
  
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   laser_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Laser_TCP");
//   tool_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Tool_TCP");
//   Cam1_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Cam1_TCP");
//   plan = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
//   //auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
//   //psm->startSceneMonitor("/move_group/monitored_planning_scene");
//   //auto test =  moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, laser_move_group->getRobotModel());
//   //auto test =  moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", "moveit_cpp_tutorial", laser_move_group->getRobotModel());

//   laser_grp_visual_tools =  std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
//   laser_grp_visual_tools->deleteAllMarkers();
//   laser_grp_visual_tools->loadRemoteControl();
//   //laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node,"world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
//   //auto test = moveit_visual_tools::MoveItVisualTools::MoveItVisualTools("/world", rviz_visual_tools::RVIZ_MARKER_TOPIC);
//   //auto test = moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node);

//   //laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(psm);

//   PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(pm_moveit_server_node, "robot_description");

//   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(pm_moveit_server_node->get_clock());
//   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//   // Dont know why this has to be called at this point, but otherwith the service callbacks get stuck.
//   //auto current_state_cam1 = Cam1_move_group->getCurrentState(1.0);
//   //auto current_state_tool = tool_move_group->getCurrentState(1.0);
//   //auto current_state_laser = laser_move_group->getCurrentState(1.0);

//   rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&execute_plan,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
//   rclcpp::Service<pm_moveit_interfaces::srv::MoveCam1TcpTo>::SharedPtr move_cam_one_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveCam1TcpTo>("pm_moveit_server/move_cam1_to_frame", std::bind(&move_group_cam1,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
//   rclcpp::Service<pm_moveit_interfaces::srv::MoveToolTcpTo>::SharedPtr move_tool_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToolTcpTo>("pm_moveit_server/move_tool_to_frame", std::bind(&move_group_tool,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
//   rclcpp::Service<pm_moveit_interfaces::srv::MoveLaserTcpTo>::SharedPtr move_laser_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveLaserTcpTo>("pm_moveit_server/move_laser_to_frame", std::bind(&move_group_laser,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);

//   xyz_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_xyz_axis_controller/joint_trajectory", 10);
//   t_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_t_axis_controller/joint_trajectory", 10);
//   auto joint_state_subscriber = pm_moveit_server_node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, jointStateCallback, node_options_1);
  
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Ready for operation...");
//   //spin_thread->join();
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }






























// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <thread>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include "std_msgs/msg/string.hpp"
// #include <tf2_msgs/msg/tf_message.hpp>

// #include "pm_moveit_interfaces/srv/execute_plan.hpp"

// #include <moveit/planning_scene/planning_scene.h>

// using std::placeholders::_1;

// class PmMoveitServer : public rclcpp::Node
//   {
//   public:

//     std::vector<std::string> obj_names_list;
//     std::vector<std::string> obj_stl_paths_list;
//     rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service;
//     moveit::planning_interface::MoveGroupInterface move_group_interface;
//     const moveit::core::JointModelGroup *joint_model_group;

//     // PmMoveitServer(
//     //   const std::string & name = "maker",
//     //   const rclcpp::NodeOptions & options = (
//     //     rclcpp::NodeOptions()
//     //     .allow_undeclared_parameters(true)
//     //     .automatically_declare_parameters_from_overrides(true)
//     // ))
//     // : Node("teste", options) //, move_group_interface(this->shared_from_this(), "PM_Robot_Cam1_TCP")

//     PmMoveitServer(std::shared_ptr<rclcpp::Node> move_group_node) : Node("teste"),
//     move_group_interface(move_group_node,"PM_Robot_Cam1_TCP"),
//     joint_model_group(move_group_interface.getCurrentState(1.0)->getJointModelGroup("PM_Robot_Cam1_TCP")) //, move_group_interface(this->shared_from_this(), "PM_Robot_Cam1_TCP")
//     {
//       RCLCPP_INFO(this->get_logger(), "Moveit Object Spawner started!");
//       auto service_cbg = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//       execute_plan_service = this->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&PmMoveitServer::move_group_cam1, this, std::placeholders::_1, std::placeholders::_2)/*,rmw_qos_profile_services_default, service_cbg*/);

//       //tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1));
//       // planning_scene_diff_publisher =this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

//       // while (planning_scene_diff_publisher->get_subscription_count() < 1)
//       // {
//       //   rclcpp::sleep_for(std::chrono::milliseconds(500));
//       //   RCLCPP_WARN(this->get_logger(), "Waiting for planing scene...");
//       // }

//       //rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue(true));
//       //this->set_parameter( simTime );
//     }

//   private:
//     //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
//     // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//     // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
//     // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
//     // std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_list;
//     // moveit_msgs::msg::PlanningScene planning_scene;

//   bool move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request, std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response){
//     static const std::string PLANNING_GROUP = "PM_Robot_Cam1_TCP";

//     // auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
//     // // For current state monitor
//     // rclcpp::executors::SingleThreadedExecutor executor;
//     // executor.add_node(move_group_node);
//     // std::thread([&executor]() { executor.spin(); }).detach();

//     // // auto node_ptr = shared_from_this();
//     // moveit::planning_interface::MoveGroupInterface move_group_interface(move_group_node, PLANNING_GROUP);
//     // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//     RCLCPP_INFO(this->get_logger(), "Test1");

//     auto move_group_interface = moveit::planning_interface::MoveGroupInterface(this->shared_from_this(), PLANNING_GROUP);

//     RCLCPP_INFO(this->get_logger(), "Test12");

//     std::string endeffector = move_group_interface.getEndEffectorLink();
//     RCLCPP_INFO(this->get_logger(), "Endeffector: %s", endeffector.c_str());

//     RCLCPP_INFO(this->get_logger(), "Test15");

//     moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10.0);

//     RCLCPP_INFO(this->get_logger(), "Test2");

//     const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState(1.0)-> getJointModelGroup(PLANNING_GROUP);

//     RCLCPP_INFO(this->get_logger(), "Test25");

//     const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

//     RCLCPP_INFO(this->get_logger(), "Test3");

//     robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(),"robot_description");
//     const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//     moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
//     robot_state->setToDefaultValues();

//     RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//     std::vector<double> joint_values;

//     // Set a target Pose
//     auto const my_pose = []{
//       geometry_msgs::msg::Pose msg;
//       msg.orientation.w = 1.0;
//       msg.position.x = 0.5325;
//       msg.position.y = 0.40528;
//       msg.position.z = 1.3;
//       msg.orientation.x = 0;
//       msg.orientation.y = 0;
//       msg.orientation.z = 0;
//       return msg;
//     }();

//     double timeout = 0.1;
//     bool found_ik = robot_state->setFromIK(joint_model_group, my_pose, timeout);

//     if (found_ik)
//     {
//       robot_state->copyJointGroupPositions(joint_model_group, joint_values);
//       for (std::size_t i = 0; i < joint_names.size(); ++i)
//         {
//           RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

//           RCLCPP_INFO(this->get_logger(), "SUCCCCCCCCCCCCEEEEEEEEEEEEESSSSSSSSSSSSSSss");
//         }
//     }
//     else
//     {
//       RCLCPP_ERROR(this->get_logger(), "Did not find IK solution");
//     }

//     response->success = true;

//     return true;
//   }

//   };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);

//   auto move_group_node = rclcpp::Node::make_shared("move_group_demo", node_options);

//   rclcpp::executors::MultiThreadedExecutor executor;

//   std::shared_ptr<PmMoveitServer> planner_node = std::make_shared<PmMoveitServer>(move_group_node);

//   executor.add_node(planner_node);
//   executor.spin();
//   // auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
//   //   executor.add_node(planner_node/*->get_node_base_interface()*/);
//   //   executor.spin();
//   //   executor.remove_node(planner_node/*->get_node_base_interface()*/);
//   // });

//   //spin_thread->join();
//   //rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <thread>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include "std_msgs/msg/string.hpp"
// #include <tf2_msgs/msg/tf_message.hpp>

// #include "pm_moveit_interfaces/srv/execute_plan.hpp"

// #include <moveit/planning_scene/planning_scene.h>

// using std::placeholders::_1;


// class PmMoveitServer : public moveit::planning_interface::MoveGroupInterface
//   {
//   public:

//     std::vector<std::string> obj_names_list;
//     std::vector<std::string> obj_stl_paths_list;
//     rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service;

//     PmMoveitServer(rclcpp::Node::SharedPtr node) : moveit::planning_interface::MoveGroupInterface(node, "PM_Robot_Cam1_TCP")
//     {
//       //RCLCPP_INFO(node->get_logger(), "Moveit Object Spawner started!");

//       execute_plan_service = node->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&PmMoveitServer::move_group_cam1, this,std::placeholders::_1, std::placeholders::_2));
//       //destroy_object_service = this->create_service<spawn_object_interfaces::srv::DestroyObject>("moveit_object_handler/destroy_object", std::bind(&MoveitObjectSpawnerNode::destroy_object, this,std::placeholders::_1, std::placeholders::_2));

//       //tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1));
//       // planning_scene_diff_publisher =this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

//       // while (planning_scene_diff_publisher->get_subscription_count() < 1)
//       // {
//       //   rclcpp::sleep_for(std::chrono::milliseconds(500));
//       //   RCLCPP_WARN(this->get_logger(), "Waiting for planing scene...");
//       // }
//       rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue(true));
//       node->set_parameter( simTime );

//       std::string endeffector = this->getEndEffector();
//       moveit::core::RobotStatePtr current_state = this->getCurrentState(10.0);
//       RCLCPP_INFO(node->get_logger(),"Endeffector: %s", endeffector.c_str());
//       const moveit::core::JointModelGroup* joint_model_group = this->getCurrentState(1.0)-> getJointModelGroup("PM_Robot_Cam1_TCP");

//       RCLCPP_INFO(node->get_logger(), "Test25");

//       const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

//       RCLCPP_INFO(node->get_logger(), "Test3");

//       robot_model_loader::RobotModelLoader robot_model_loader(node,"robot_description");
//       const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//       moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
//       robot_state->setToDefaultValues();

//       RCLCPP_INFO(node->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//     }

//   private:
//     //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
//     // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//     // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
//     // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
//     // std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_list;
//     // moveit_msgs::msg::PlanningScene planning_scene;

//   bool move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request, std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response){

//     static const std::string PLANNING_GROUP = "PM_Robot_Cam1_TCP";

//     //RCLCPP_INFO(node->get_logger(), "Test1");

//     //auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node->shared_from_this(), PLANNING_GROUP);
//     std::string endeffector = this->getEndEffector();
//     //RCLCPP_INFO(node->get_logger(), "Endeffector: %s", endeffector.c_str());

//     //RCLCPP_INFO(node->get_logger(), "Test15");

//     moveit::core::RobotStatePtr current_state = this->getCurrentState(10.0);

//     //RCLCPP_INFO(node->get_logger(), "Test2");

//     const moveit::core::JointModelGroup* joint_model_group = this->getCurrentState(1.0)-> getJointModelGroup(PLANNING_GROUP);

//     // RCLCPP_INFO(this->get_logger(), "Test25");

//     // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

//     // RCLCPP_INFO(this->get_logger(), "Test3");

//     // robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(),"robot_description");
//     // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
//     // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
//     // robot_state->setToDefaultValues();

//     // RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//     // std::vector<double> joint_values;

//     // // Set a target Pose
//     // auto const my_pose = []{
//     //   geometry_msgs::msg::Pose msg;
//     //   msg.orientation.w = 1.0;
//     //   msg.position.x = 0.5325;
//     //   msg.position.y = 0.40528;
//     //   msg.position.z = 1.3;
//     //   msg.orientation.x = 0;
//     //   msg.orientation.y = 0;
//     //   msg.orientation.z = 0;
//     //   return msg;
//     // }();

//     // double timeout = 0.1;
//     // bool found_ik = robot_state->setFromIK(joint_model_group, my_pose, timeout);

//     // if (found_ik)
//     // {
//     //   robot_state->copyJointGroupPositions(joint_model_group, joint_values);
//     //   for (std::size_t i = 0; i < joint_names.size(); ++i)
//     //     {
//     //       RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

//     //       RCLCPP_INFO(this->get_logger(), "SUCCCCCCCCCCCCEEEEEEEEEEEEESSSSSSSSSSSSSSss");
//     //     }
//     // }
//     // else
//     // {
//     //   RCLCPP_ERROR(this->get_logger(), "Did not find IK solution");
//     // }

//     response->success = true;

//     return true;
//   }

//   };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);

//   auto node = rclcpp::Node::make_shared("add_srv_node"/*, node_options*/);

//   rclcpp::executors::MultiThreadedExecutor executor;
//   auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
//     executor.add_node(node);
//     executor.spin();
//     executor.remove_node(node);
//   });

//   PmMoveitServer test(node);

//   spin_thread->join();
//   //rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

/*
ros2 service call /pm_moveit_server/move_tool_to_frame pm_moveit_interfaces/srv/MoveToolTcpTo "{
  frame_name: 'Camera_Station_TCP',
  move_to_pose: {
    position: {
      x: 0.530,
      y: 0.40,
      z: 1.3
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    }},
    translation: {
      x: 0.0,
      y: 0.0,
      z: 0.015
    },
    exec_wait_for_user_input: false,
    execute: true

}"


ros2 service call /pm_moveit_server/move_cam1_to_frame pm_moveit_interfaces/srv/MoveCam1TcpTo "{
  frame_name: 'Camera_Station_TCP',
  move_to_pose: {
    position: {
      x: 0.530,
      y: 0.40,
      z: 1.3
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    }},
    translation: {
      x: 0.0,
      y: 0.0,
      z: 0.015
    },
    exec_wait_for_user_input: false,
    execute: true

}"

ros2 service call /pm_moveit_server/move_laser_to_frame pm_moveit_interfaces/srv/MoveLaserTcpTo "{
  frame_name: 'Camera_Station_TCP',
  move_to_pose: {
    position: {
      x: 0.530,
      y: 0.40,
      z: 1.3
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    }},
    translation: {
      x: 0.0,
      y: 0.0,
      z: 0.015
    },
    exec_wait_for_user_input: false,
    execute: true

}"


*/