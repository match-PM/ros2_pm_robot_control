#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("object_topics_publisher")
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.declare_parameter('parent_frame_name', 'world')

        self.declare_parameter('child_frame_name', 'child_frame_test')

        self.parent_frame = self.get_parameter('parent_frame_name').get_parameter_value().string_value
        
        self.declare_parameter('translation', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation',[0.0, 0.0, 0.0, 1.0])

        self.logger = self.get_logger()

        self.child_frame = self.get_parameter('child_frame_name').get_parameter_value().string_value

        namespace = self.get_namespace()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.transform_x = self.get_parameter('translation').value[0]
        self.transform_y = self.get_parameter('translation').value[1]
        self.transform_z = self.get_parameter('translation').value[2]

        self.transform_qx = self.get_parameter('rotation').value[0]
        self.transform_qy = self.get_parameter('rotation').value[1]
        self.transform_qz = self.get_parameter('rotation').value[2]
        self.transform_qw = self.get_parameter('rotation').value[3]

        self.pose_publisher = self.create_publisher(Pose, 'pose', 10)
        self.parent_frame_publisher = self.create_publisher(String, 'parent_frame', 10)

        self.publish_parent_frame()
        self.publish_pose()
        self.publish_transform_TF()
        
        self.timer_ = self.create_timer(10.0, self.publish_parent_frame)
        self.timer_ = self.create_timer(10.0, self.publish_pose)
        
        self.parent_frame_subscription = self.create_subscription(String, 'parent_frame', self.update_transform_frame_change, 10)
        self.pose_subscription = self.create_subscription(Pose, 'pose', self.update_pose, 10)
        
        self.previous_pose_message_ = None
        self.previous_parent_frame_message_ = None


    def check_frame_exists(self, frame_id):
        try:
            self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
            #self.get_logger().info("Frame '{}' exists.".format(frame_id))
            return True
        except Exception as e:
            print(e)
            self.get_logger().warn("Frame '{}' does not exist.".format(frame_id))
            return False


    def publish_parent_frame(self):
        msg = String()
        msg.data = self.parent_frame
        self.parent_frame_publisher.publish(msg)
        self.logger.info("Parent frame published!")

    def publish_pose(self):
        pose_msg = Pose()
        pose_msg.position.x = self.transform_x
        pose_msg.position.y = self.transform_y
        pose_msg.position.z = self.transform_z
        pose_msg.orientation.x = self.transform_qx
        pose_msg.orientation.y = self.transform_qy
        pose_msg.orientation.z = self.transform_qz
        pose_msg.orientation.w = self.transform_qw
        self.pose_publisher.publish(pose_msg)
        self.logger.info("Pose published!")

    def update_pose(self, msg):
        if self.previous_pose_message_ != msg:
            self.previous_pose_message_ = msg
            self.transform_x = msg.position.x
            self.transform_y = msg.position.y
            self.transform_z = msg.position.z

            self.transform_qx = msg.orientation.x
            self.transform_qy = msg.orientation.y
            self.transform_qz = msg.orientation.z
            self.transform_qw = msg.orientation.w
            self.logger.info("Pose updated!")
            self.publish_pose()
            self.publish_transform_TF()
        
    def update_transform_frame_change(self, msg):
        if self.previous_parent_frame_message_ != msg:
            if self.check_frame_exists(msg.data):
                self.adapt_tf_for_new_parent_frame(msg.data)
                self.previous_parent_frame_message_ = msg
                self.parent_frame = msg.data
                self.logger.info("Parent Frame updated!")
                self.publish_parent_frame()
                self.publish_transform_TF()
    
    def adapt_tf_for_new_parent_frame(self, new_parent_frame):
        t = self.tf_buffer.lookup_transform(self.child_frame, new_parent_frame,rclpy.time.Time())
        self.transform_x = -t.transform.translation.x
        self.transform_y = -t.transform.translation.y
        self.transform_z = -t.transform.translation.z

        self.transform_qx = t.transform.rotation.x
        self.transform_qy = t.transform.rotation.y
        self.transform_qz = t.transform.rotation.z
        self.transform_qw = t.transform.rotation.w


    def publish_transform_TF(self):
        # Create a static transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()  # Use current timestamp
        transform_stamped.header.frame_id = self.parent_frame
        transform_stamped.child_frame_id = self.child_frame
        # Set the translation
        transform_stamped.transform.translation.x = self.transform_x
        transform_stamped.transform.translation.y = self.transform_y
        transform_stamped.transform.translation.z = self.transform_z
        # Set the rotation (quaternion)
        transform_stamped.transform.rotation.x = self.transform_qx
        transform_stamped.transform.rotation.y = self.transform_qy
        transform_stamped.transform.rotation.z = self.transform_qz
        transform_stamped.transform.rotation.w = self.transform_qw
        # Publish the static transform
        self.tf_broadcaster.sendTransform(transform_stamped)
        self.logger.info("TF published!")

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    node.check_frame_exists("world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
