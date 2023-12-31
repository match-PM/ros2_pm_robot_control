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
from geometry_msgs.msg import Vector3, Quaternion

def quaternion_multiply(q0:Quaternion, q1:Quaternion)->Quaternion:
    """
    Multiplies two quaternions.

    Input
    :param q0: 
    :param q1: 

    Output
    :return: Quaternion

    """

    #q0.w = -q0.w

    # Extract the values from q0
    w0 = q0.w
    x0 = q0.x
    y0 = q0.y
    z0 = q0.z

    # Extract the values from q1
    w1 = q1.w
    x1 = q1.x
    y1 = q1.y
    z1 = q1.z

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    result = Quaternion()
    result.x = q0q1_x
    result.y = q0q1_y
    result.z = q0q1_z
    result.w = q0q1_w

    return result


def check_and_return_quaternion(object_to_check,logger=None):
    """This function checks if a given quaternion is valid. This is the case if x=0, y=0, z=0, w=0. 
    In this case the function will set the quaterion to x=0, y=0, z=0, w=1.
    This function accepts quaternions as well as poses as input. According to its input the equivaltent type is returned. """

    if isinstance(object_to_check,Quaternion):
        tmp_q = object_to_check
    elif isinstance(object_to_check,Pose):
        tmp_q = object_to_check.orientation

    # This case will probably never happen because the node would crash at some point
    else:
        if logger is not None:
            logger.error("Fatal error")
        return object_to_check
    
    if tmp_q.x==0 and tmp_q.y==0 and tmp_q.z==0 and tmp_q.w==0:
        tmp_q.x=0.0
        tmp_q.y=0.0
        tmp_q.z=0.0
        tmp_q.w=1.0
        if logger is not None:
            logger.info("Assuming Quaternion: x=0.0, y=0.0, z=0.0, w=1.0!")
    
    if isinstance(object_to_check,Quaternion):
        return tmp_q
    elif isinstance(object_to_check,Pose):
        object_to_check.orientation = tmp_q
        return object_to_check


class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("object_topics_publisher")

        self.callback_group = MutuallyExclusiveCallbackGroup()

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
        '''This function is the callback function for the /ChangeParentFrame Service. It iterates through the objects list, 
        and in case it finds the valid object it calculates the transformation to a new given parent frame. It then publishes the calculated.'''
        for index, (obj_name, obj_parent_frame, obj_trans, obj_rot) in enumerate(zip(self.object_names_list, self.object_parent_frames_list, self.object_translations_list,self.object_rotations_list)):

            if request.obj_name == obj_name:
                if self.object_parent_frames_list[index] != request.parent_frame:

                    if obj_name == request.parent_frame:
                        self.logger.error(f'Parent and child frame can not be the same! parent_frame = child_frame ')
                        response.success = False
                        return response                       

                    # If the new parent frame exists
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

    def create_ref_frame(self, request:CreateRefFrame.Request, response:CreateRefFrame.Response):
        """This is the callback function for the /CreateRefFrame service. If the TF Frame does not exist, the function creates a new TF Frame. 
        If the frame already exists, the inforamtion will be updated!"""
        frame_existend = self.check_frame_exists(request.frame_name)

        name_conflict = self.check_object_exists(request.frame_name)

        if name_conflict:
            self.get_logger().error(f'Frame can not have the same name as an existing object!')
            response.success = False
            return response

        # if the ref frame does not exist yet
        if not frame_existend:
            parent_frame_exists = self.check_frame_exists(request.parent_frame)

            if parent_frame_exists:
                frame_name=request.frame_name
                frame_parent_name = request.parent_frame
                pose = check_and_return_quaternion(request.pose,self.logger)

                self.ref_frame_names_list.append(request.frame_name)
                self.ref_frame_parent_names_list.append(request.parent_frame)
                self.ref_frame_poses_list.append(pose)
                
                topic_str='Object/'+frame_parent_name+'/'+frame_name
                self.ref_frame_publisher_list.append(self.create_publisher(RefFrameMsg,topic_str,10))

                self.publsih_topics()
                self.publish_transform_TF(frame_name,frame_parent_name,pose.position,pose.orientation)
                response.success = True
            else:
                self.logger.warn(f'Tried to spawn {request.frame_name}, but parent frame {request.parent_frame} does not exists! Aborted!')
                response.success = False

        # if the ref frame already exists
        else:
            for index, (frame_name) in enumerate(self.ref_frame_names_list):
                if request.frame_name == frame_name:
                    self.ref_frame_parent_names_list[index]   = request.parent_frame
                    self.ref_frame_poses_list[index]       = check_and_return_quaternion(request.pose,self.logger)
                    self.publsih_topics()

                    self.publish_transform_TF(request.frame_name,
                                              self.ref_frame_parent_names_list[index],
                                              self.ref_frame_poses_list[index].position,
                                              self.ref_frame_poses_list[index].orientation)

                    self.get_logger().warn(f'Service for creating {request.frame_name} was called, but frame does already exist! Information for {request.frame_name} updated!')
                    response.success = True

        return response

    def delete_ref_frame(self, request, response):

        for index, (ref_frame_name) in enumerate(self.ref_frame_names_list):
            if request.frame_name == ref_frame_name:
                # destroy TF !!!! A static TF cant be destroyed. Instead it is detached from the world. 'unused_frame
                t = Vector3()
                r = Quaternion()
                t.x=1.0
                t.y=1.0
                t.z=1.0

                r.x=0.0
                r.y=0.0
                r.z=0.0
                r.w=1.0
                self.publish_transform_TF(self.ref_frame_names_list[index],'unused_frame', t, r)
                
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
            
        self.logger.error(f'Tried to destroy {request.obj_name}, but object does not exist!')
        return response


    def spawn_object_callback(self, request:SpawnObject.Request, response:SpawnObject.Response):
        """This method spawns an object. This means that all the relevant object information are stored in a list that contains all the objects. 
        It then publishes the information as topics and also publishes a Frame in TF"""
        name_conflict = self.check_ref_frame_exists(request.obj_name)

        if name_conflict:
            self.get_logger().error(f'Object can not have the same name as an existing reference frame!')
            response.success = False
            return response

        parent_frame_exists = self.check_frame_exists(request.parent_frame)
            
        if not parent_frame_exists:
            self.logger.warn(f'Tried to spawn object {request.obj_name}, but parent frame {request.parent_frame} does not exist!')
            response.success = False
            return response
        
        # Check if object exists in the object list
        obj_existend = self.check_object_exists(request.obj_name)

        # if not exists create
        if not obj_existend:
            obj_name=request.obj_name
            cad_path=request.cad_data

            rotation = check_and_return_quaternion(request.rotation,self.logger)

            obj_parent_frame = request.parent_frame

            self.object_names_list.append(obj_name)
            self.object_cad_paths_list.append(cad_path)
            self.object_translations_list.append(request.translation)
            self.object_rotations_list.append(rotation)
            self.object_parent_frames_list.append(obj_parent_frame)
            
            topic_str='Object/'+obj_name
            self.object_publisher_list.append(self.create_publisher(ObjectMsg,topic_str,10))
            self.object_subscriptions_list.append(self.create_subscription(ObjectMsg,topic_str,self.update_object_message,10))

            self.publsih_topics()
            self.publish_transform_TF(obj_name,obj_parent_frame,request.translation,rotation)

            response.success = True

        # if exists updated values
        else:
            for index, (obj_name) in enumerate(self.object_names_list):
                if request.obj_name == obj_name:
                    self.object_parent_frames_list[index]   = request.parent_frame
                    self.object_cad_paths_list[index]       = request.cad_data
                    self.object_translations_list[index]    = request.translation
                    self.object_rotations_list[index]       = check_and_return_quaternion(request.rotation,self.logger)
                    self.get_logger().warn(f'Service for spawning {request.obj_name} was called, but object does already exist!')
                    self.get_logger().warn(f'Information for {request.obj_name} updated!')
                    self.publsih_topics()
                    self.publish_transform_TF(obj_name,request.parent_frame,self.object_translations_list[index],self.object_rotations_list[index])
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

            obj_m.pose.orientation=obj_rot
            obj_m.pose.position.x=float(obj_trans.x)
            obj_m.pose.position.y=float(obj_trans.y)
            obj_m.pose.position.z=float(obj_trans.z)

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
                self.object_translations_list[index].x = msg.pose.position.x
                self.object_translations_list[index].y = msg.pose.position.y
                self.object_translations_list[index].z = msg.pose.position.z

                self.object_rotations_list[index].x = msg.pose.orientation.x
                self.object_rotations_list[index].y = msg.pose.orientation.y
                self.object_rotations_list[index].z = msg.pose.orientation.z
                self.object_rotations_list[index].w = msg.pose.orientation.w

                self.object_parent_frames_list[index] = msg.parent_frame
                self.object_cad_paths_list[index] = msg.cad_data

                self.publish_transform_TF(self.object_names_list[index],self.object_parent_frames_list[index],self.object_translations_list[index],self.object_rotations_list[index])
                
                #self.logger.info(f'Message for {obj_name} updated!') 

    def adapt_tf_for_new_parent_frame(self,child_frame, new_parent_frame):
        # this function adapts the tf for parent_frame changes
        t = self.tf_buffer.lookup_transform(child_frame, new_parent_frame,rclpy.time.Time())
        trans = Vector3()
        rot = Quaternion()
        trans.x = -t.transform.translation.x
        trans.y = -t.transform.translation.y
        trans.z = -t.transform.translation.z

        rot.x = t.transform.rotation.x
        rot.y = t.transform.rotation.y
        rot.z = t.transform.rotation.z
        rot.w = t.transform.rotation.w

        return trans, rot


    def publish_transform_TF(self, child_frame, parent_frame, translation:Vector3, rotations:Quaternion):
        # Create a static transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()  # Use current timestamp
        transform_stamped.header.frame_id = parent_frame
        transform_stamped.child_frame_id = child_frame

        # # Set the translation
        transform_stamped.transform.translation.x = float(translation.x)
        transform_stamped.transform.translation.y = float(translation.y)
        transform_stamped.transform.translation.z = float(translation.z)

        # Set the rotation (quaternion)
        transform_stamped.transform.rotation.x = float(rotations.x)
        transform_stamped.transform.rotation.y = float(rotations.y)
        transform_stamped.transform.rotation.z = float(rotations.z)
        transform_stamped.transform.rotation.w = float(rotations.w)

        # Publish the static transform
        self.tf_broadcaster.sendTransform(transform_stamped)
        #self.logger.info(f'TF for {child_frame} published!')

    def modify_pose(self, request: ModifyPose.Request, response: ModifyPose.Response):
        
        # check if frame name is an object
        for index, (obj_name, obj_trans, obj_rot) in enumerate(zip(self.object_names_list, self.object_translations_list,self.object_rotations_list)):

            if request.frame_name == obj_name:


                self.object_translations_list[index].x += request.rel_pose.position.x
                self.object_translations_list[index].y += request.rel_pose.position.y
                self.object_translations_list[index].z += request.rel_pose.position.z
              
                self.object_rotations_list[index] = quaternion_multiply(self.object_rotations_list[index],request.rel_pose.orientation)

                self.publish_transform_TF(self.object_names_list[index],self.object_parent_frames_list[index],self.object_translations_list[index],self.object_rotations_list[index])
                self.publsih_topics()
                self.logger.info(f'Pose for object {request.frame_name} updated!')
                response.success = True
                return response

        # check if frame name is a ref_frame
        for index, (frame_name, frame_pose) in enumerate(zip(self.ref_frame_names_list, self.ref_frame_poses_list)):

            if request.frame_name == frame_name:

                self.ref_frame_poses_list[index].position.x += request.rel_pose.position.x
                self.ref_frame_poses_list[index].position.y += request.rel_pose.position.y
                self.ref_frame_poses_list[index].position.z += request.rel_pose.position.z

                rel_orientation = check_and_return_quaternion(request.rel_pose.orientation,self.logger)

                self.ref_frame_poses_list[index].orientation = quaternion_multiply(self.ref_frame_poses_list[index].orientation,rel_orientation)

                self.publsih_topics()
                self.publish_transform_TF(self.ref_frame_names_list[index],
                                          self.ref_frame_parent_names_list[index],
                                          self.ref_frame_poses_list[index].position, 
                                          self.ref_frame_poses_list[index].orientation)
                
                self.logger.info(f'Pose for frame {request.frame_name} updated!')
                response.success = True
                return response

        self.logger.warn("Pose could not be updated. Frame not found!")
        response.success = False
        return response
    

    
def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
