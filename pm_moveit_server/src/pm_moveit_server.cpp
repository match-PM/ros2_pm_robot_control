
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "pm_moveit_interfaces/srv/execute_plan.hpp"
#include "pm_moveit_interfaces/srv/move_cam1_tcp_to.hpp"
#include "pm_moveit_interfaces/srv/move_laser_tcp_to.hpp"
#include "pm_moveit_interfaces/srv/move_tool_tcp_to.hpp"

#include <moveit/planning_scene/planning_scene.h>

using std::placeholders::_1;

// moveit::planning_interface::MoveGroupInterface Cam1_move_group;

// Global Variables
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> laser_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Cam1_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> tool_move_group;

std::shared_ptr<moveit_visual_tools::MoveItVisualTools> laser_grp_visual_tools;

std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
moveit::planning_interface::MoveGroupInterface::Plan plan;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

bool execute_plan(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request,
                  std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response)
{
  return true;
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> myfunction(std::string planning_group,
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

  geometry_msgs::msg::Pose target_pose;
  std::vector<double> target_joint_values;
  bool service_success;
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Model frame: %s", kinematic_model->getModelFrame().c_str());

  geometry_msgs::msg::Quaternion target_rotation;
  tf2::Quaternion rotation_tf, frame_rotation_tf, target_rotation_tf;

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Retrieved Rotation x: %f, y: %f, z: %f, w: %f", rotation.x, rotation.y, rotation.z, rotation.w);

  // if rotation is not specified
  if (rotation.w == 0.0 && rotation.w == 0.0 && rotation.w == 0.0 && rotation.w == 0.0)
  {

    target_rotation.w = 1;
    target_rotation.x = 0;
    target_rotation.y = 0;
    target_rotation.z = 0;
    tf2::convert(target_rotation, rotation_tf); // convert msg::quaternion to tf2::quaternion
  }
  else
  {
    tf2::convert(rotation, rotation_tf); // convert msg::quaternion to tf2::quaternion
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

      // target_pose.orientation = frame_transform.transform.rotation * target_rotation;
      tf2::convert(frame_transform.transform.rotation, frame_rotation_tf); // convert msg::quaternion to tf2::quaternio
      target_rotation_tf = rotation_tf * frame_rotation_tf;
      target_rotation_tf.normalize();
      tf2::convert(target_rotation_tf, target_pose.orientation);
      // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation x: %f, y: %f, z: %f, w: %f",target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);

      // target_pose.orientation.x = frame_transform.transform.rotation.x;
      // target_pose.orientation.y = frame_transform.transform.rotation.y;
      // target_pose.orientation.z = frame_transform.transform.rotation.z;
      // target_pose.orientation.w = frame_transform.transform.rotation.w;
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
    // target_pose.orientation = move_to_pose.orientation * target_rotation;
    target_pose.orientation.x = 0;
    target_pose.orientation.y = 0;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 1;
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
      target_pose.orientation.x = 0;
      target_pose.orientation.y = 0;
      target_pose.orientation.z = 0;
      target_pose.orientation.w = 1;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      service_success = false;
      return {service_success, joint_names, target_joint_values};
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Pose:");
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

  bool success_calculate_plan = false;

  if (success_found_ik)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "IK solution found!");
    robot_state->copyJointGroupPositions(joint_model_group, target_joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Joint Values for %s: %f", joint_names[i].c_str(), target_joint_values[i]);
    }

    move_group->setStartStateToCurrentState();
    move_group->setJointValueTarget(target_joint_values);
    success_calculate_plan = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

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

  // Execute the plan
  if (success_calculate_plan && execute)
  {
    move_group->execute(plan);
    // Checking delta;
    try
    {

      geometry_msgs::msg::TransformStamped moved_to_transform;
      geometry_msgs::msg::Pose moved_to_pose;

      moved_to_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);

      moved_to_pose.position.x = moved_to_transform.transform.translation.x;
      moved_to_pose.position.y = moved_to_transform.transform.translation.y;
      moved_to_pose.position.z = moved_to_transform.transform.translation.z;
      double deltaX = moved_to_pose.position.x - target_pose.position.x;
      double deltaY = moved_to_pose.position.y - target_pose.position.y;
      double deltaZ = moved_to_pose.position.z - target_pose.position.z;
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

void move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Response> response)
{

  auto [success, joint_names, joint_values] = myfunction("PM_Robot_Cam1_TCP",
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

  auto [success, joint_names, joint_values] = myfunction("PM_Robot_Tool_TCP",
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

  auto [success, joint_names, joint_values] = myfunction("PM_Robot_Laser_TCP",
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto const pm_moveit_server_node = std::make_shared<rclcpp::Node>(
      "pm_moveit_server",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &pm_moveit_server_node]()
                                                   {
    executor.add_node(pm_moveit_server_node);
    executor.spin();
    executor.remove_node(pm_moveit_server_node); });

  auto const logger = rclcpp::get_logger("hello_moveit");

  laser_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Laser_TCP");
  tool_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Tool_TCP");
  Cam1_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Cam1_TCP");
  
  //auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  //psm->startSceneMonitor("/move_group/monitored_planning_scene");
  const std::string tests="world";
  auto test = moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());

  //laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node,"world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
  //auto test = moveit_visual_tools::MoveItVisualTools::MoveItVisualTools("/world", rviz_visual_tools::RVIZ_MARKER_TOPIC);
  //auto test = moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node);

  //laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(psm);

  PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(pm_moveit_server_node, "robot_description");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(pm_moveit_server_node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Dont know why this has to be called at this point, but otherwith the service callbacks get stuck.
  auto current_state_cam1 = Cam1_move_group->getCurrentState(1.0);
  auto current_state_tool = tool_move_group->getCurrentState(1.0);
  auto current_state_laser = laser_move_group->getCurrentState(1.0);

  rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", &execute_plan);
  rclcpp::Service<pm_moveit_interfaces::srv::MoveCam1TcpTo>::SharedPtr move_cam_one_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveCam1TcpTo>("pm_moveit_server/move_cam1_to_frame", &move_group_cam1);
  rclcpp::Service<pm_moveit_interfaces::srv::MoveToolTcpTo>::SharedPtr move_tool_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToolTcpTo>("pm_moveit_server/move_tool_to_frame", &move_group_tool);
  rclcpp::Service<pm_moveit_interfaces::srv::MoveLaserTcpTo>::SharedPtr move_laser_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveLaserTcpTo>("pm_moveit_server/move_laser_to_frame", &move_group_laser);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Ready for operation...");
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
