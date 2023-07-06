#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from spawn_object_interfaces.srv import DestroyObject
from spawn_object_interfaces.srv import SpawnObject
from spawn_object_interfaces.msg import ObjectMsg
from spawn_object_interfaces.msg import RefFrameMsg

from spawn_object_interfaces.srv import ChangeParentFrame
from spawn_object_interfaces.srv import CreateRefFrame
from spawn_object_interfaces.srv import DeleteRefFrame
from spawn_object_interfaces.srv import GetInfo
from spawn_object_interfaces.srv import ModifyPose

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("object_topics_publisher")

        self.callback_group = MutuallyExclusiveCallbackGroup()

        #ros2 service call /spawn_object spawn_object_interfaces/srv/SpawnObject "{obj_name: my_test_object, parent_frame: world, translation:[1.0,1.0,3.0], rotation:[1.0,2.0,3.0,4.0]}"

        self.spawn_object_srv = self.create_service(SpawnObject,'object_publisher/spawn_object',self.spawn_object_callback,callback_group=self.callback_group)
        self.destroy_object_srv = self.create_service(DestroyObject,'object_publisher/destroy_object',self.destroy_object_callback,callback_group=self.callback_group)

        self.create_ref_frame_srv = self.create_service(CreateRefFrame,'object_manager/create_ref_frame',self.create_ref_frame,callback_group=self.callback_group)
        self.delete_ref_frame_srv = self.create_service(DeleteRefFrame,'object_manager/delete_ref_frame',self.delete_ref_frame,callback_group=self.callback_group)  

        self.change_parent_frame_srv = self.create_service(ChangeParentFrame,'object_manager/change_obj_parent_frame',self.change_obj_parent_frame,callback_group=self.callback_group)  
        
        self.change_parent_frame_srv = self.create_service(ModifyPose,'object_manager/modify_pose',self.modify_pose,callback_group=self.callback_group)  

        self.get_info_srv = self.create_service(GetInfo,'object_manager/get_info',self.get_info,callback_group=self.callback_group)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.logger = self.get_logger()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer_ = self.create_timer(5.0, self.publsih_topics,callback_group=self.callback_group)

        self.object_names_list=[]
        self.object_parent_frames_list=[]
        self.object_cad_paths_list=[]
        self.object_translations_list=[]
        self.object_rotations_list=[]

        self.object_subscriptions_list=[]
        self.object_publisher_list=[]

        self.ref_frame_names_list = []
        self.ref_frame_parent_names_list = []
        self.ref_frame_poses_list = []
        self.ref_frame_publisher_list = []

        self.logger.info("Object Topic publisher started!")

    def get_info(self, request, response):

        response.obj_names = self.object_names_list
        response.ref_frame_names = self.ref_frame_names_list

        return response

    def change_obj_parent_frame(self, request, response):

        for index, (obj_name, obj_parent_frame, obj_trans, obj_rot) in enumerate(zip(self.object_names_list, self.object_parent_frames_list, self.object_translations_list,self.object_rotations_list)):

            if request.obj_name == obj_name:
                if self.object_parent_frames_list[index] != request.parent_frame:

                    if obj_name == request.parent_frame:
                        self.logger.error(f'Parent and child frame can not be the same! parent_frame = child_frame ')
                        response.success = False
                        return response                       

                    if self.check_frame_exists(request.parent_frame):
                        new_trans, new_rot = self.adapt_tf_for_new_parent_frame(child_frame=obj_name,new_parent_frame=request.parent_frame)
                        self.object_translations_list[index] = new_trans
                        self.object_rotations_list[index] = new_rot
                        self.object_parent_frames_list[index] = request.parent_frame
                        self.logger.info(f'Parent Frame updated!') 
                        self.publish_transform_TF(obj_name,self.object_parent_frames_list[index],self.object_translations_list[index],self.object_rotations_list[index])
                        self.publsih_topics()
                        response.success = True
                        return response
                    else:
                        self.logger.warn(f'Attemd to change parent frame of {obj_name} but frame {request.parent_frame} does not exist! No change to TF executed!')
                        response.success = False
                        return response
                
                self.logger.warn(f'Parent frame is already set!')
                response.success = True
                return response

    def create_ref_frame(self, request, response):

        frame_existend = self.check_frame_exists(request.frame_name)

        name_conflict = self.check_object_exists(request.frame_name)

        if name_conflict:
            self.get_logger().error(f'Frame can not have the same name as an existing object!')
            response.success = False
            return response

        if not frame_existend:
            parent_frame_exists = self.check_frame_exists(request.parent_frame)

            if parent_frame_exists:
                frame_name=request.frame_name
                frame_parent_name = request.parent_frame
                pose = request.pose   

                self.ref_frame_names_list.append(request.frame_name)
                self.ref_frame_parent_names_list.append(request.parent_frame)
                self.ref_frame_poses_list.append(request.pose)
                
                topic_str='Object/'+frame_parent_name+'/'+frame_name
                self.ref_frame_publisher_list.append(self.create_publisher(RefFrameMsg,topic_str,10))

                self.publsih_topics()

                translation = [pose.position.x, pose.position.y, pose.position.z]
                rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                self.publish_transform_TF(frame_name,frame_parent_name,translation,rotation)
                response.success = True
            else:
                self.logger.warn(f'Parent frame {request.parent_frame} does not exists! Aborted!')
                response.success = False

        else:
            for index, (frame_name) in enumerate(self.ref_frame_names_list):
                if request.frame_name == frame_name:
                    self.ref_frame_parent_names_list[index]   = request.parent_frame
                    self.ref_frame_poses_list[index]       = request.pose
                    self.get_logger().warn(f'Service for creating {request.frame_name} was called, but frame does already exist!')
                    self.get_logger().warn(f'Information for {request.frame_name} updated!')
                    response.success = True

        return response

    def delete_ref_frame(self, request, response):

        for index, (ref_frame_name) in enumerate(self.ref_frame_names_list):
            if request.frame_name == ref_frame_name:
                # destroy TF !!!! A static TF cant be destroyed. Instead it is detached from the world.
                self.publish_transform_TF(self.ref_frame_names_list[index],'unused_frame', (1,1,1), (1,1,1,1))
                
                del self.ref_frame_names_list[index]
                del self.ref_frame_parent_names_list[index]
                del self.ref_frame_poses_list[index]

                self.destroy_publisher(self.ref_frame_publisher_list[index])

                del self.ref_frame_publisher_list[index]
                self.logger.info(f'Frame {ref_frame_name} destroyed!!!')
                self.publsih_topics()
                response.success = True
                return response
            
        self.get_logger().warn(f'Frame {request.frame_name} could not be deleted! Frame not found!')
        return response
        

    def destroy_object_callback(self, request, response):
        response.success = False

        for index, (obj_name) in enumerate(self.object_names_list):
            if request.obj_name == obj_name:
                # destroy TF !!!! A static TF cant be destroyed. Instead it is detached from the world.
                self.publish_transform_TF(self.object_names_list[index],'unused_frame',self.object_translations_list[index],self.object_rotations_list[index])
                
                del self.object_names_list[index]
                del self.object_parent_frames_list[index]
                del self.object_cad_paths_list[index]
                del self.object_translations_list[index]
                del self.object_rotations_list[index]
                self.destroy_publisher(self.object_publisher_list[index])
                self.destroy_subscription(self.object_subscriptions_list[index])
                del self.object_publisher_list[index]
                del self.object_subscriptions_list[index]

                self.publsih_topics()
                self.logger.info(f'{obj_name} destroyed!!!')
                response.success = True

        return response


    def spawn_object_callback(self, request, response):

        name_conflict = self.check_ref_frame_exists(request.obj_name)

        if name_conflict:
            self.get_logger().error(f'Object can not have the same name as an existing reference frame!')
            response.success = False
            return response
        
        # Check if object exists in the object list
        obj_existend = self.check_object_exists(request.obj_name)

        # if not exists create
        if not obj_existend:
            obj_name=request.obj_name
            cad_path=request.cad_data
            translation = request.translation
            rotation = request.rotation
            obj_parent_frame = request.parent_frame

            self.object_names_list.append(obj_name)
            self.object_cad_paths_list.append(cad_path)
            self.object_translations_list.append(translation)
            self.object_rotations_list.append(rotation)
            self.object_parent_frames_list.append(obj_parent_frame)

            topic_str='Object/'+obj_name
            self.object_publisher_list.append(self.create_publisher(ObjectMsg,topic_str,10))
            self.object_subscriptions_list.append(self.create_subscription(ObjectMsg,topic_str,self.update_object_message,10))

            self.publsih_topics()
            frame_exists = self.check_frame_exists(obj_parent_frame)
            
            if not frame_exists:
                self.logger.warn(f'TF for {obj_name} published but parent frame {obj_parent_frame} does not currently exist!')

            self.publish_transform_TF(obj_name,obj_parent_frame,translation,rotation)

            response.success = True

        # if exists updated values
        else:
            for index, (obj_name) in enumerate(self.object_names_list):
                if request.obj_name == obj_name:
                    self.object_parent_frames_list[index]   = request.parent_frame
                    self.object_cad_paths_list[index]       = request.cad_data
                    self.object_translations_list[index]    = request.translation
                    self.object_rotations_list[index]       = request.rotation
                    self.get_logger().warn(f'Service for spawning {request.obj_name} was called, but object does already exist!')
                    self.get_logger().warn(f'Information for {request.obj_name} updated!')
                    response.success = True

        return response

    def check_frame_exists(self, frame_id):
        # This function checks if a tf exists in the tf buffer
        try:
            self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
            return True
        except Exception as e:
            print(e)
            return False

    def check_object_exists(self,name_new_obj):
        # this function checks if an object is in the objects list
        for obj_name in self.object_names_list:
            if obj_name == name_new_obj:
                return True
        return False
    
    def check_ref_frame_exists(self,name_new_frame):
        # this function checks if an frame esixts in the frame list
        for frame_name in self.ref_frame_names_list:
            if frame_name == name_new_frame:
                return True
        return False
    
    def publsih_topics(self):
        # this function publishes the object and frame messages
        self.logger.info(f'Publishing topics!') 
        self.logger.info(f'Registred Objects (total {len(self.object_names_list)}):') 
        for index, (obj_name) in enumerate(self.object_names_list):
            self.logger.info(f'{index+1}. {obj_name}')
        for obj_name, obj_parent_frame, obj_cad_p, obj_trans, obj_rot, obj_pub in zip(self.object_names_list, self.object_parent_frames_list, self.object_cad_paths_list, self.object_translations_list,self.object_rotations_list,self.object_publisher_list):
          
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
            obj_m.cad_data=obj_cad_p
            obj_pub.publish(obj_m)

        # Reference Frames
        self.logger.info(f'Registred Reference Frames (total {len(self.ref_frame_names_list)}):') 
        for index, (frame_name) in enumerate(self.ref_frame_names_list):
            self.logger.info(f'{index+1}. {frame_name}')

        for frame_name, parent_frame, pose, frame_pub in zip(self.ref_frame_names_list, self.ref_frame_parent_names_list, self.ref_frame_poses_list, self.ref_frame_publisher_list):
            frame_m=RefFrameMsg()
            frame_m.frame_name=frame_name
            frame_m.pose=pose
            frame_m.parent_frame=parent_frame
            frame_pub.publish(frame_m)


    def update_object_message(self, msg):
        # This function updates the values of objects if the topic has been changed. 
        # If an external publisher changes the topic this node updates the object accordingly.
        # Maybe to delete in the future

        for index, (obj_name, obj_parent_frame, obj_cad_p, obj_trans, obj_rot, obj_pub) in enumerate(zip(self.object_names_list, self.object_parent_frames_list, self.object_cad_paths_list, self.object_translations_list,self.object_rotations_list,self.object_publisher_list)):

            if msg.obj_name == obj_name:

                self.object_translations_list[index][0] = msg.pose.position.x
                self.object_translations_list[index][1] = msg.pose.position.y
                self.object_translations_list[index][2] = msg.pose.position.z

                self.object_rotations_list[index][0] = msg.pose.orientation.x
                self.object_rotations_list[index][1] = msg.pose.orientation.y
                self.object_rotations_list[index][2] = msg.pose.orientation.z
                self.object_rotations_list[index][3] = msg.pose.orientation.w
                self.object_parent_frames_list[index] = msg.parent_frame
                self.object_cad_paths_list[index] = msg.cad_data

                self.publish_transform_TF(self.object_names_list[index],self.object_parent_frames_list[index],self.object_translations_list[index],self.object_rotations_list[index])
                
                #self.logger.info(f'Message for {obj_name} updated!') 

    def adapt_tf_for_new_parent_frame(self,child_frame, new_parent_frame):
        # this function adapts the tf for parent_frame changes
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

    def modify_pose(self, request, response):
        
        for index, (obj_name, obj_trans, obj_rot) in enumerate(zip(self.object_names_list, self.object_translations_list,self.object_rotations_list)):

            if request.frame_name == obj_name:

                self.object_translations_list[index][0] += request.rel_pose.position.x
                self.object_translations_list[index][1] += request.rel_pose.position.y
                self.object_translations_list[index][2] += request.rel_pose.position.z

                # TODO: Add relative rotation
                #self.object_rotations_list[index][0] += request.rel_pose.orientation.x
                #self.object_rotations_list[index][1] += request.rel_pose.orientation.y
                #self.object_rotations_list[index][2] += request.rel_pose.orientation.z
                #self.object_rotations_list[index][3] += request.rel_pose.orientation.w

                self.publish_transform_TF(self.object_names_list[index],self.object_parent_frames_list[index],self.object_translations_list[index],self.object_rotations_list[index])
                self.publsih_topics()
                self.logger.info(f'Pose for object {request.frame_name} updated!')
                response.success = True
                return response
                #self.logger.info(f'Message for {obj_name} updated!') 

        for index, (frame_name, frame_pose) in enumerate(zip(self.ref_frame_names_list, self.ref_frame_poses_list)):

            if request.frame_name == frame_name:

                self.ref_frame_poses_list[index].position.x += request.rel_pose.position.x
                self.ref_frame_poses_list[index].position.y += request.rel_pose.position.y
                self.ref_frame_poses_list[index].position.z += request.rel_pose.position.z

                # TODO: Add relative rotation
                #self.ref_frame_poses_list[index].orientation.x += request.rel_pose.orientation.x
                #self.ref_frame_poses_list[index].orientation.y += request.rel_pose.orientation.y
                #self.ref_frame_poses_list[index].orientation.z += request.rel_pose.orientation.z
                #self.ref_frame_poses_list[index].orientation.w += request.rel_pose.orientation.w


                self.publsih_topics()
                translation = [self.ref_frame_poses_list[index].position.x, self.ref_frame_poses_list[index].position.y, self.ref_frame_poses_list[index].position.z]
                rotation = [self.ref_frame_poses_list[index].orientation.x, self.ref_frame_poses_list[index].orientation.y, self.ref_frame_poses_list[index].orientation.z, self.ref_frame_poses_list[index].orientation.w]

                self.publish_transform_TF(self.ref_frame_names_list[index],self.ref_frame_parent_names_list[index],translation,rotation)

                self.logger.info(f'Pose for frame {request.frame_name} updated!')
                response.success = True
                return response

        self.logger.warn("Pose could not be updated. Frame not found!")
        response.success = False
        return response
        
    
def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    #node.check_frame_exists("world")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
