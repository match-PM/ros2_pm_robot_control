
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2_msgs/msg/tf_message.hpp>

#include "pm_moveit_interfaces/srv/execute_plan.hpp"

#include <moveit/planning_scene/planning_scene.h>

using std::placeholders::_1;

class PmMoveitServer : public rclcpp::Node
  {
  public:

    std::vector<std::string> obj_names_list;
    std::vector<std::string> obj_stl_paths_list;
    rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    const moveit::core::JointModelGroup *joint_model_group;

    // PmMoveitServer(
    //   const std::string & name = "maker",
    //   const rclcpp::NodeOptions & options = (
    //     rclcpp::NodeOptions()
    //     .allow_undeclared_parameters(true)
    //     .automatically_declare_parameters_from_overrides(true)
    // )) 
    // : Node("teste", options) //, move_group_interface(this->shared_from_this(), "PM_Robot_Cam1_TCP")

    PmMoveitServer(std::shared_ptr<rclcpp::Node> move_group_node) : Node("teste"),
    move_group_interface(move_group_node,"PM_Robot_Cam1_TCP"),
    joint_model_group(move_group_interface.getCurrentState(1.0)->getJointModelGroup("PM_Robot_Cam1_TCP")) //, move_group_interface(this->shared_from_this(), "PM_Robot_Cam1_TCP")
    {
      RCLCPP_INFO(this->get_logger(), "Moveit Object Spawner started!");
      auto service_cbg = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      execute_plan_service = this->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&PmMoveitServer::move_group_cam1, this, std::placeholders::_1, std::placeholders::_2)/*,rmw_qos_profile_services_default, service_cbg*/);

      //tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1));
      // planning_scene_diff_publisher =this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

      // while (planning_scene_diff_publisher->get_subscription_count() < 1)
      // {
      //   rclcpp::sleep_for(std::chrono::milliseconds(500));
      //   RCLCPP_WARN(this->get_logger(), "Waiting for planing scene...");
      // }


      //rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue(true));
      //this->set_parameter( simTime );
    }

  private:
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
    // std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_list;
    // moveit_msgs::msg::PlanningScene planning_scene;

  bool move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request, std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response){
    static const std::string PLANNING_GROUP = "PM_Robot_Cam1_TCP";


    // auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
    // // For current state monitor
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(move_group_node);
    // std::thread([&executor]() { executor.spin(); }).detach();



    // // auto node_ptr = shared_from_this();
    // moveit::planning_interface::MoveGroupInterface move_group_interface(move_group_node, PLANNING_GROUP);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    RCLCPP_INFO(this->get_logger(), "Test1");

    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(this->shared_from_this(), PLANNING_GROUP);

    RCLCPP_INFO(this->get_logger(), "Test12");

    std::string endeffector = move_group_interface.getEndEffectorLink();
    RCLCPP_INFO(this->get_logger(), "Endeffector: %s", endeffector.c_str());

    RCLCPP_INFO(this->get_logger(), "Test15");

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10.0);

    RCLCPP_INFO(this->get_logger(), "Test2");


    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState(1.0)-> getJointModelGroup(PLANNING_GROUP);

    RCLCPP_INFO(this->get_logger(), "Test25");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    RCLCPP_INFO(this->get_logger(), "Test3");

    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(),"robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    robot_state->setToDefaultValues();

    RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    std::vector<double> joint_values;

    // Set a target Pose
    auto const my_pose = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.5325;
      msg.position.y = 0.40528;
      msg.position.z = 1.3;
      msg.orientation.x = 0;
      msg.orientation.y = 0;
      msg.orientation.z = 0;
      return msg;
    }();

    double timeout = 0.1;
    bool found_ik = robot_state->setFromIK(joint_model_group, my_pose, timeout);

    if (found_ik)
    { 
      robot_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

          RCLCPP_INFO(this->get_logger(), "SUCCCCCCCCCCCCEEEEEEEEEEEEESSSSSSSSSSSSSSss");
        }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Did not find IK solution");
    }

    response->success = true;

    return true;
  }
    
  };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node = rclcpp::Node::make_shared("move_group_demo", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<PmMoveitServer> planner_node = std::make_shared<PmMoveitServer>(move_group_node);

  executor.add_node(planner_node);
  executor.spin();
  // auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
  //   executor.add_node(planner_node/*->get_node_base_interface()*/);
  //   executor.spin();
  //   executor.remove_node(planner_node/*->get_node_base_interface()*/);
  // });

  //spin_thread->join();
  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}





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


