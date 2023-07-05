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

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

using std::placeholders::_1;

class MoveitObjectSpawnerNode : public rclcpp::Node
  {
  public:

    std::string parent_frame;
    std::string child_frame;
    std::string previous_parent_frame_;
    std::string subsription_topic;
    std::string MeshFilePath;
    std::vector<double> translation;
    std::vector<double> rotation;
    geometry_msgs::msg::Pose object_pose;

    MoveitObjectSpawnerNode() : Node("my_node")
    {
      //tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Declare parameters
      this->declare_parameter<std::string>("obj_frame_name");
      this->declare_parameter<std::string>("STL_path");
      this->declare_parameter<std::string>("obj_namespace");

      std::string obj_namespace= this->get_parameter("obj_namespace").as_string();
      
      child_frame = this->get_parameter("obj_frame_name").as_string();


      tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1));

      MeshFilePath = "file://" + this->get_parameter("STL_path").as_string();
      RCLCPP_INFO(this->get_logger(), "Mesh_filename: %s", MeshFilePath.c_str());

    }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
      // Process the received TFMessage
      for (const auto& transform : msg->transforms)
      {
        // Access the transform data
        std::string p_frame = transform.header.frame_id;
        std::string c_frame = transform.child_frame_id;
        if (c_frame == child_frame)
        {
          RCLCPP_INFO(this->get_logger(), "Object Frame: '%s'", c_frame.c_str());
          geometry_msgs::msg::Transform tf = transform.transform;
          object_pose.position.x = tf.translation.x;
          object_pose.position.y = tf.translation.y;
          object_pose.position.z = tf.translation.z;
          object_pose.orientation.x = tf.rotation.x;
          object_pose.orientation.y = tf.rotation.y;
          object_pose.orientation.z = tf.rotation.z;
          object_pose.orientation.w = tf.rotation.w;
          parent_frame = p_frame;
          apply_object_to_moveit();
        }
      
        // Process the transform data...
      }
    }

    void apply_object_to_moveit() 
      {
        // robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
        // const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
        RCLCPP_INFO(this->get_logger(), "Moveit Object Reference: '%s'", parent_frame.c_str());
        moveit_msgs::msg::CollisionObject spawining_object;
        spawining_object.header.frame_id = parent_frame;
        spawining_object.id = child_frame;

        shapes::Mesh * m = shapes::createMeshFromResource(MeshFilePath);
        shape_msgs::msg::Mesh shelf_mesh;
        shapes::ShapeMsg shelf_mesh_msg;
        bool success = shapes::constructMsgFromShape(m,shelf_mesh_msg);
        shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

        spawining_object.meshes.push_back(shelf_mesh);
        spawining_object.mesh_poses.push_back(object_pose);
        spawining_object.operation = spawining_object.ADD;

        moveit_msgs::msg::AttachedCollisionObject attached_object;

        attached_object.object = spawining_object;
        attached_object.link_name = parent_frame;
        //attached_object.transform = transform_stamped.transform;
        planning_scene_interface_.applyAttachedCollisionObject(attached_object);

        //planning_scene_interface.applyCollisionObject(spawining_object);
        delete m;  // Cleanup the mesh object
      }
  };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveitObjectSpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

