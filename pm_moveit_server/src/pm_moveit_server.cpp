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

#include <moveit/planning_scene/planning_scene.h>

using std::placeholders::_1;

class PmMoveitServer : public rclcpp::Node
  {
  public:

    std::vector<std::string> obj_names_list;
    std::vector<std::string> obj_stl_paths_list;


    PmMoveitServer() : Node("my_node")
    {
      RCLCPP_INFO(this->get_logger(), "Moveit Object Spawner started!");

      //spawn_object_service = this->create_service<spawn_object_interfaces::srv::SpawnObject>("moveit_object_handler/spawn_object", std::bind(&MoveitObjectSpawnerNode::spawn_object, this,std::placeholders::_1, std::placeholders::_2));
      //destroy_object_service = this->create_service<spawn_object_interfaces::srv::DestroyObject>("moveit_object_handler/destroy_object", std::bind(&MoveitObjectSpawnerNode::destroy_object, this,std::placeholders::_1, std::placeholders::_2));

      //tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1));
      // planning_scene_diff_publisher =this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

      // while (planning_scene_diff_publisher->get_subscription_count() < 1)
      // {
      //   rclcpp::sleep_for(std::chrono::milliseconds(500));
      //   RCLCPP_WARN(this->get_logger(), "Waiting for planing scene...");
      // }
      
    }

  private:
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
    // std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_list;
    // moveit_msgs::msg::PlanningScene planning_scene;

  bool move_group_cam1(){

    static const std::string PLANNING_GROUP = "PM_Robot_Cam1_TCP";

    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(this->shared_from_this(),PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()-> getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    

    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
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
    return true;
  }
    
  };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PmMoveitServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

