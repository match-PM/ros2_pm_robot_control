#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from spawn_object_interfaces.srv import DestroyObject
from spawn_object_interfaces.srv import SpawnObject
from spawn_object_interfaces.msg import ObjectMsg

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("object_topics_publisher")

        #ros2 service call /spawn_object spawn_object_interfaces/srv/SpawnObject "{obj_name: my_test_object, parent_frame: world, translation:[1.0,1.0,3.0], rotation:[1.0,2.0,3.0,4.0]}"

        self.spawn_object_srv = self.create_service(SpawnObject,'object_publisher/spawn_object',self.spawn_object_callback)
        self.destroy_object_srv = self.create_service(DestroyObject,'object_publisher/destroy_object',self.destroy_object_callback)
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.logger = self.get_logger()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer_ = self.create_timer(5.0, self.publish_object_message)

        self.object_names_list=[]
        self.object_parent_frames_list=[]
        self.object_stl_paths_list=[]
        self.object_translations_list=[]
        self.object_rotations_list=[]

        self.object_subscriptions_list=[]
        self.object_publisher_list=[]

        self.logger.info("Object Topic publisher started!")

    def destroy_object_callback(self, request, response):
        response.success = False
        #ros2 service call /destroy_object spawn_object_interfaces/srv/DestroyObject "{obj_name: my_tessst_obje}"

        for index, (obj_name) in enumerate(self.object_names_list):
            if request.obj_name == obj_name:
                # destroy TF !!!! A static TF cant be destroyed. Instead it is detached from the world.
                self.publish_transform_TF(self.object_names_list[index],'unused_frame',self.object_translations_list[index],self.object_rotations_list[index])
                
                del self.object_names_list[index]
                del self.object_parent_frames_list[index]
                del self.object_stl_paths_list[index]
                del self.object_translations_list[index]
                del self.object_rotations_list[index]

                self.destroy_publisher(self.object_publisher_list[index])
                self.destroy_subscription(self.object_subscriptions_list[index])

                del self.object_publisher_list[index]
                del self.object_subscriptions_list[index]

                response.success = True

        return response


    def spawn_object_callback(self, request, response):

        obj_valid = self.check_object_exists(request.obj_name)

        if obj_valid:
            obj_name=request.obj_name
            stl_path=request.stl_path
            translation = request.translation
            rotation = request.rotation
            obj_parent_frame = request.parent_frame

            self.object_names_list.append(obj_name)
            self.object_stl_paths_list.append(stl_path)
            self.object_translations_list.append(translation)
            self.object_rotations_list.append(rotation)
            self.object_parent_frames_list.append(obj_parent_frame)

            topic_str='Object/'+obj_name
            self.object_publisher_list.append(self.create_publisher(ObjectMsg,topic_str,10))
            self.object_subscriptions_list.append(self.create_subscription(ObjectMsg,topic_str,self.update_object_message,10))

            self.publish_object_message()
            frame_exists = self.check_frame_exists(obj_parent_frame)
            
            if not frame_exists:
                self.logger.warn(f'TF for {obj_name} published but parent frame {obj_parent_frame} does not currently exist!')

            self.publish_transform_TF(obj_name,obj_parent_frame,translation,rotation)

            response.success = True

        else:
            for index, (obj_name) in enumerate(self.object_names_list):
                if request.obj_name == obj_name:
                    self.object_parent_frames_list[index]   = request.parent_frame
                    self.object_stl_paths_list[index]       = request.stl_path
                    self.object_translations_list[index]    = request.translation
                    self.object_rotations_list[index]       = request.rotation
                    self.get_logger().warn(f'Service for spawning {request.obj_name} was called, but object does already exist!')
                    self.get_logger().warn(f'Information for {request.obj_name} updated!')
                    response.success = True

        return response

    def check_frame_exists(self, frame_id):
        try:
            self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
            #self.get_logger().info("Frame '{}' exists.".format(frame_id))
            return True
        except Exception as e:
            print(e)
            self.get_logger().warn("Frame '{}' does not exist.".format(frame_id))
            return False

    def check_object_exists(self,name_new_obj):
        for obj_name in self.object_names_list:
            if obj_name == name_new_obj:
                return False
        return True
    
    def publish_object_message(self):
        self.logger.info(f'{len(self.object_names_list)} objects registred!') 
        for index, (obj_name) in enumerate(self.object_names_list):
            print(f'{index+1}. {obj_name}')
        for obj_name, obj_parent_frame, obj_stl_p, obj_trans, obj_rot, obj_pub in zip(self.object_names_list, self.object_parent_frames_list, self.object_stl_paths_list, self.object_translations_list,self.object_rotations_list,self.object_publisher_list):
          
            obj_m=ObjectMsg()
            obj_m.obj_name=obj_name
            obj_m.pose.position.x=float(obj_trans[0])
            obj_m.pose.position.y=float(obj_trans[1])
            obj_m.pose.position.z=float(obj_trans[2])

            obj_m.pose.orientation.x=float(obj_rot[0])
            obj_m.pose.orientation.y=float(obj_rot[1])
            obj_m.pose.orientation.z=float(obj_rot[2])
            obj_m.pose.orientation.w=float(obj_rot[3])

            obj_m.parent_frame=obj_parent_frame
            obj_m.stl_path=obj_stl_p
            obj_pub.publish(obj_m)
            #self.logger.info(f'Message for {obj_name} published!') 

    def update_object_message(self, msg):

        for index, (obj_name, obj_parent_frame, obj_stl_p, obj_trans, obj_rot, obj_pub) in enumerate(zip(self.object_names_list, self.object_parent_frames_list, self.object_stl_paths_list, self.object_translations_list,self.object_rotations_list,self.object_publisher_list)):
            # If Änderung!!!

            if msg.obj_name == obj_name:

                self.object_translations_list[index][0] = msg.pose.position.x
                self.object_translations_list[index][1] = msg.pose.position.y
                self.object_translations_list[index][2] = msg.pose.position.z

                self.object_rotations_list[index][0] = msg.pose.orientation.x
                self.object_rotations_list[index][1] = msg.pose.orientation.y
                self.object_rotations_list[index][2] = msg.pose.orientation.z
                self.object_rotations_list[index][3] = msg.pose.orientation.w
                
                if self.object_parent_frames_list[index] != msg.parent_frame:

                    if self.check_frame_exists(msg.parent_frame):
                        new_trans, new_rot = self.adapt_tf_for_new_parent_frame(child_frame=obj_name,new_parent_frame=msg.parent_frame)
                        self.object_translations_list[index] = new_trans
                        self.object_rotations_list[index] = new_rot
                        self.object_parent_frames_list[index] = msg.parent_frame
                        self.logger.info(f'TF updated!') 
                    else:
                        self.logger.warn(f'Attemd to change parent frame of {obj_name} but frame {msg.parent_frame} does not exist! No change to TF executed!')

                self.publish_transform_TF(obj_name,self.object_parent_frames_list[index],self.object_translations_list[index],self.object_rotations_list[index])

                self.object_stl_paths_list[index] = msg.stl_path
                self.logger.info(f'Message for {obj_name} updated!') 

 
    def adapt_tf_for_new_parent_frame(self,child_frame, new_parent_frame):

        t = self.tf_buffer.lookup_transform(child_frame, new_parent_frame,rclpy.time.Time())
        trans=[]
        rot=[]

        trans.append(-t.transform.translation.x)
        trans.append(-t.transform.translation.y)
        trans.append(-t.transform.translation.z)

        rot.append(t.transform.rotation.x)
        rot.append(t.transform.rotation.y)
        rot.append(t.transform.rotation.z)
        rot.append(t.transform.rotation.w)

        return trans, rot


    def publish_transform_TF(self, child_frame, parent_frame, translation, rotations):
        # Create a static transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()  # Use current timestamp
        transform_stamped.header.frame_id = parent_frame
        transform_stamped.child_frame_id = child_frame
        # Set the translation
        transform_stamped.transform.translation.x = float(translation[0])
        transform_stamped.transform.translation.y = float(translation[1])
        transform_stamped.transform.translation.z = float(translation[2])
        # Set the rotation (quaternion)
        transform_stamped.transform.rotation.x = float(rotations[0])
        transform_stamped.transform.rotation.y = float(rotations[1])
        transform_stamped.transform.rotation.z = float(rotations[2])
        transform_stamped.transform.rotation.w = float(rotations[3])
        # Publish the static transform
        self.tf_broadcaster.sendTransform(transform_stamped)
        #self.logger.info(f'TF for {child_frame} published!')

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    #node.check_frame_exists("world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
