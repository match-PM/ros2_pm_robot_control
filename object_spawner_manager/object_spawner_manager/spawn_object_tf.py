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
from spawn_object_interfaces.msg import Object
from spawn_object_interfaces.msg import RefFrame

import spawn_object_interfaces.msg as soi_msgs

import spawn_object_interfaces.srv as soi_srvs

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

def adapt_transform_for_new_parent_frame(child_frame, new_parent_frame, tf_buffer: Buffer):
    # this function adapts the tf for parent_frame changes
    t = tf_buffer.lookup_transform(child_frame, new_parent_frame,rclpy.time.Time())
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

def publish_transform_tf_static(node: Node, 
                                tf_broadcaster:StaticTransformBroadcaster, 
                                child_frame:str, 
                                parent_frame:str, 
                                translation:Vector3, 
                                rotations:Quaternion):
    # Create a static transform
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = node.get_clock().now().to_msg()  # Use current timestamp
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
    tf_broadcaster.sendTransform(transform_stamped)

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("object_topics_publisher")

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.object_scene = ObjScene(self)

        self.spawn_object_srv = self.create_service(SpawnObject,f'object_spawner_publisher/spawn_object',self.spawn_object_callback,callback_group=self.callback_group)
        self.destroy_object_srv = self.create_service(DestroyObject,f'object_spawner_publisher/destroy_object',self.destroy_object_callback,callback_group=self.callback_group)

        self.create_ref_frame_srv = self.create_service(CreateRefFrame,f'object_spawner_manager/create_ref_frame',self.create_ref_frame,callback_group=self.callback_group)
        self.delete_ref_frame_srv = self.create_service(DeleteRefFrame,f'object_spawner_manager/delete_ref_frame',self.destroy_ref_frame,callback_group=self.callback_group)  

        self.change_parent_frame_srv = self.create_service(ChangeParentFrame,f'object_spawner_manager/change_obj_parent_frame',self.change_obj_parent_frame,callback_group=self.callback_group)  
        
        self.change_parent_frame_srv = self.create_service(ModifyPose,'object_spawner_manager/modify_pose',self.modify_pose,callback_group=self.callback_group)  

        self.get_info_srv = self.create_service(soi_srvs.GetScene,f'object_spawner_manager/get_scene',self.get_scene,callback_group=self.callback_group)      

        self.create_ref_plane_srv = self.create_service(soi_srvs.CreateRefPlane,f'object_spawner_manager/create_ref_plane',self.create_ref_plane,callback_group=self.callback_group)      

        self.timer = self.create_timer(5.0, self.object_scene.publish_information,callback_group=self.callback_group)
       
        self.get_logger().info("Object Topic publisher started!")
    
    def get_scene(self, request :soi_srvs.GetScene.Request, response:soi_srvs.GetScene.Response):
        response.scene = self.object_scene.scene
        response.success = True
        return response

    def change_obj_parent_frame(self, request:soi_srvs.ChangeParentFrame.Request, response:soi_srvs.ChangeParentFrame.Response):
        '''This function is the callback function for the /ChangeParentFrame Service. It iterates through the objects list, 
        and in case it finds the valid object it calculates the transformation to a new given parent frame. It then publishes the calculated.'''
        change_success = self.object_scene.change_obj_parent_frame(     obj_id=request.obj_name,
                                                                        new_parent_frame=request.new_parent_frame)
        response.success = change_success
        return response

    def create_ref_frame(self, request:CreateRefFrame.Request, response:CreateRefFrame.Response):
        """This is the callback function for the /CreateRefFrame service. If the TF Frame does not exist, the function creates a new TF Frame. 
        If the frame already exists, the inforamtion will be updated!"""
        
        add_success = self.object_scene.add_ref_frame_to_scene(request.ref_frame)
        response.success = add_success

        return response

    def create_ref_plane(self,request:soi_srvs.CreateRefPlane.Request, response:soi_srvs.CreateRefPlane.Response):
        self.get_logger().warn("Starting call")
        create_success = self.object_scene.create_ref_plane(plane = request.ref_plane)
        response.success = create_success
        return response

    def destroy_ref_frame(self, request:DeleteRefFrame.Request, response:DeleteRefFrame.Response):
        """This is the callback function for the /CreateRefFrame service. If the TF Frame does not exist, the function creates a new TF Frame. 
        If the frame already exists, the inforamtion will be updated!"""
        
        add_success = self.object_scene.destroy_ref_frame(frame_id = request.frame_name)
        response.success = add_success

        return response   

    def spawn_object_callback(self, request:SpawnObject.Request, response:SpawnObject.Response):
        """This method spawns an object. This means that all the relevant object information are stored in a list that contains all the objects. 
        It then publishes the information as topics and also publishes a Frame in TF"""
        new_obj = soi_msgs.Object()
        new_obj.cad_data = request.cad_data
        new_obj.obj_name = request.obj_name
        new_obj.parent_frame = request.parent_frame
        new_obj.obj_pose.orientation = request.rotation
        new_obj.obj_pose.position.x = request.translation.x
        new_obj.obj_pose.position.y = request.translation.y
        new_obj.obj_pose.position.z = request.translation.z

        add_success = self.object_scene.add_obj_to_scene(new_obj)
        response.success = add_success

        return response
    
    def destroy_object_callback(self, request:soi_srvs.DestroyObject.Request, response:soi_srvs.DestroyObject.Response):
        del_success = self.object_scene.destroy_object(request.obj_name)
        response.success = del_success
        return response


    def modify_pose(self, request: ModifyPose.Request, response: ModifyPose.Response):
        modify_success = self.object_scene.modify_pose(frame_obj_name= request.frame_name,
                                                       rel_pose = request.rel_pose)
        response.success = modify_success
        return response
    

class ObjScene():
    UNUSED_FRAME_CONST = 'unused_frame'
    def __init__(self, node: Node):
        self.scene = soi_msgs.ObjectScene()
        self.node = node

        self._scene_publisher = node.create_publisher(soi_msgs.ObjectScene,'/object_spawner_manager/scene',10)
        self.logger = node.get_logger()
        self.tf_broadcaster = StaticTransformBroadcaster(node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def publish_information(self):
        self.publish_scene()
        self.publish_to_tf()

    def publish_scene(self):
        self._scene_publisher.publish(self.scene)
        self.logger.info("Object Scene has been published")

    def add_obj_to_scene(self, new_obj:soi_msgs.Object)-> bool:

        name_conflict = self.check_ref_frame_exists(new_obj.obj_name)

        if name_conflict:
            self.logger.error(f'Object can not have the same name as an existing reference frame!')
            return False

        parent_frame_exists = self.check_if_frame_exists(new_obj.parent_frame)
            
        if not parent_frame_exists:
            self.logger.warn(f"Tried to spawn object {new_obj.obj_name}, but parent frame '{new_obj.parent_frame}' does not exist!")
            return False

        obj_existend = self.check_object_exists(new_obj.obj_name)

        if not obj_existend:
            self.scene.objects_in_scene.append(new_obj)
            self.publish_scene()
            self.publish_to_tf()
            return True

        # if exists updated values
        else:
            for index, obj in enumerate(self.scene.objects_in_scene):
                obj:soi_msgs.Object
                if obj.obj_name == new_obj.obj_name:
                    del self.scene.objects_in_scene[index]
                    new_obj.obj_pose.orientation = check_and_return_quaternion(new_obj.obj_pose.orientation,self.logger)
                    self.scene.objects_in_scene.append(new_obj)

                    self.logger.warn(f'Service for spawning {new_obj.obj_name} was called, but object does already exist!')
                    self.logger.warn(f'Information for {new_obj.obj_name} updated!')
                    self.publish_scene()
                    self.publish_to_tf()
                    return True
            # eventually 
            return False

    def add_ref_frame_to_scene(self, new_ref_frame:soi_msgs.RefFrame)-> bool:

        ref_frame_existend = self.check_ref_frame_exists(new_ref_frame.frame_name)
        
        name_conflict_1 = self.check_object_exists(new_ref_frame.frame_name)
        name_conflict_2 = self.check_if_frame_exists(new_ref_frame.frame_name)

        if not ref_frame_existend and (name_conflict_1 or name_conflict_2):
            self.logger.error(f'Ref frame can not have the same name as an existing reference frame or object!')
            return False

        parent_frame_exists = self.check_if_frame_exists(new_ref_frame.parent_frame)
            
        if not parent_frame_exists:
            self.logger.warn(f'Tried to spawn ref frame {new_ref_frame.frame_name}, but parent frame {new_ref_frame.parent_frame} does not exist!')
            return False

        # Check if the new ref frame should be connected to an existing object or not.
        frame_is_obj_frame = self.check_object_exists(new_ref_frame.parent_frame)

        frame_list_to_append_to = []

        if frame_is_obj_frame:
            frame_list_to_append_to = self.get_obj_by_name(new_ref_frame.parent_frame).ref_frames
        else:
            frame_list_to_append_to = self.scene.ref_frames_in_scene

        if not ref_frame_existend:
            frame_list_to_append_to.append(new_ref_frame)
        else:
            for index, frame in enumerate(frame_list_to_append_to):
                frame: soi_msgs.RefFrame
                if frame.frame_name == new_ref_frame.frame_name:
                    del frame_list_to_append_to[index]
                    frame_list_to_append_to.append(new_ref_frame)
                self.logger.warn(f'Service for creating {new_ref_frame.frame_name} was called, but frame does already exist! Information for {new_ref_frame.frame_name} updated!')
        
        self.publish_information()
        return True

    def destroy_object(self, obj_id:str)-> bool:
        """
        This method will delete an object from the objects list
        """

        for index, obj in enumerate(self.scene.objects_in_scene):
            obj:soi_msgs.Object
            if obj.obj_name == obj_id:
                # change the parent frame for the ref frames connected to the object. This is necessary because the ref frame would reapear if a new obj with the same name would be spawned. 
                for ref_frame in obj.ref_frames:
                    ref_frame:soi_msgs.RefFrame
                    ref_frame.parent_frame = self.UNUSED_FRAME_CONST
                # publish the changes from the ref frames
                self.publish_information()

                # destroy TF !!!! A static TF cant be destroyed. Instead it is detached from the world.
                publish_transform_tf_static(node= self.node,
                                            tf_broadcaster= self.tf_broadcaster,
                                            child_frame=obj.obj_name,
                                            parent_frame=self.UNUSED_FRAME_CONST,
                                            translation=obj.obj_pose.position,
                                            rotations=obj.obj_pose.orientation)
                                
                del self.scene.objects_in_scene[index]

                self.publish_information()
                self.logger.info(f'{obj_id} destroyed!!!')
                return True
            
        self.logger.error(f'Tried to destroy {obj_id}, but object does not exist!')
        return False
    
    def destroy_ref_frame(self, frame_id:str)-> bool:

        # Check if frame is refevert to an object or not.
        for index, frame in enumerate(self.scene.ref_frames_in_scene):
            frame: soi_msgs.RefFrame
            if frame.frame_name == frame_id:
                publish_transform_tf_static(node= self.node,
                                            tf_broadcaster= self.tf_broadcaster,
                                            child_frame=frame.frame_name,
                                            parent_frame='unused_frame',
                                            translation=frame.pose.position,
                                            rotations=frame.pose.orientation)
                del self.scene.ref_frames_in_scene[index]
                self.logger.info(f"Frame '{frame_id}' destroyed!")
                return True
            
        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            for index, frame in enumerate(obj.ref_frames):
                frame: soi_msgs.RefFrame
                if frame.frame_name == frame_id:
                    publish_transform_tf_static(node= self.node,
                            tf_broadcaster= self.tf_broadcaster,
                            child_frame=frame.frame_name,
                            parent_frame='unused_frame',
                            translation=frame.pose.position,
                            rotations=frame.pose.orientation)
                    del self.scene.ref_frames_in_scene[index]
                    self.logger.info(f'Frame {frame_id} destroyed!')
                    return True
        
        self.logger().error(f"Frame '{frame_id}' could not be deleted! Frame does not exist!")
        return False
 
    def get_obj_by_name(self, obj_name:str) -> soi_msgs.Object:
        """
        Returns the object from the objects list by the given obj name
        """
        for obj in self.scene.objects_in_scene:
            obj:soi_msgs.Object
            if obj.obj_name == obj_name:
                return obj
        return None

    def change_obj_parent_frame(self, obj_id: str, new_parent_frame:str) -> bool :
        if obj_id == new_parent_frame:
            self.logger.error(f'Parent and child frame can not be the same! parent_frame = child_frame ')
            return False
        
        new_parent_frame_exists = self.check_if_frame_exists(new_parent_frame)
        if not new_parent_frame_exists:
            self.logger.error(f"The given parent frame '{new_parent_frame}' does not exist! Frame could not be changed!")
            return False
        
        obj_to_change = self.get_obj_by_name(obj_name=obj_id)   # returns not if not found

        # if obj is not None
        if obj_to_change is not None:
            if obj_to_change.parent_frame == new_parent_frame:
                self.logger.warn(f'Parent frame is already set!')
                return True
            new_trans, new_rot = adapt_transform_for_new_parent_frame(  child_frame=obj_to_change.obj_name,
                                                                        new_parent_frame=new_parent_frame,
                                                                        tf_buffer=self.tf_buffer)
            obj_to_change.obj_pose.position.x = new_trans.x
            obj_to_change.obj_pose.position.y = new_trans.y
            obj_to_change.obj_pose.position.z = new_trans.z
            obj_to_change.obj_pose.orientation = new_rot
            obj_to_change.parent_frame = new_parent_frame
            self.logger.info(f'Parent Frame updated!') 
            self.publish_information()
            return True
        else:
            self.logger.error(f'Given obj_id is not an existing object!')
            return False
            
    def publish_to_tf(self):
        # Create a static transform
        for ref_frame in self.scene.ref_frames_in_scene:
            transform = TransformStamped()
            ref_frame:soi_msgs.RefFrame
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id = ref_frame.parent_frame
            transform.child_frame_id = ref_frame.frame_name
            transform.transform.rotation = ref_frame.pose.orientation
            transform.transform.translation.x=ref_frame.pose.position.x
            transform.transform.translation.y=ref_frame.pose.position.y
            transform.transform.translation.z=ref_frame.pose.position.z

            self.tf_broadcaster.sendTransform(transform)
            self.logger.info(f"TF for '{ref_frame.frame_name}' published!")

        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
            transform_stamped.child_frame_id = obj.obj_name
            transform_stamped.header.frame_id = obj.parent_frame
            transform_stamped.transform.translation.x = obj.obj_pose.position.x
            transform_stamped.transform.translation.y = obj.obj_pose.position.y
            transform_stamped.transform.translation.z = obj.obj_pose.position.z
            transform_stamped.transform.rotation = obj.obj_pose.orientation
            self.tf_broadcaster.sendTransform(transform_stamped)

            self.logger.info(f"TF for object'{obj.obj_name}' published!")

        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            for ref_frame in obj.ref_frames:
                transform = TransformStamped()
                ref_frame:soi_msgs.RefFrame
                transform.header.stamp = self.node.get_clock().now().to_msg()
                transform.header.frame_id = ref_frame.parent_frame
                transform.child_frame_id = ref_frame.frame_name
                transform.transform.rotation = ref_frame.pose.orientation
                transform.transform.translation.x=ref_frame.pose.position.x
                transform.transform.translation.y=ref_frame.pose.position.y
                transform.transform.translation.z=ref_frame.pose.position.z

                self.tf_broadcaster.sendTransform(transform)
                self.logger.info(f"TF for '{ref_frame.frame_name}' published!")

    def check_if_frame_exists(self, frame_id:str) -> bool:
        # This function checks if a tf exists in the tf buffer
        try:
            self.tf_buffer.lookup_transform("world", frame_id, rclpy.time.Time())
            return True
        except Exception as e:
            print(e)
            return False

    def check_object_exists(self,name_new_obj:str) -> bool:
        # this function checks if an object is in the objects list
        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            if obj.obj_name == name_new_obj:
                return True
        return False
    
    def check_ref_frame_exists(self,name_new_frame:str) -> bool:
        # this function checks if an frame esixts in the frame list
        # iterate over ref_frames
        for frame_name in self.scene.ref_frames_in_scene:
            frame_name:TransformStamped
            if frame_name == name_new_frame:
                return True
        
        # iterate over objects
        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            # iterate over ref_frames in object
            for ref_frame in obj.ref_frames:
                ref_frame:soi_msgs.RefFrame
                if ref_frame.frame_name == name_new_frame:
                    return True

        return False
    
    def create_ref_plane(self, plane: soi_msgs.Plane) -> bool:
        try:
            if not len(plane.point_names) == 3:
                self.logger.error(f"Not enough input arguments. Plane could not be created!")
                return False
            
            parent_frame_1 = self.get_parent_frame_for_ref_frame(plane.point_names[0])
            parent_frame_2 = self.get_parent_frame_for_ref_frame(plane.point_names[1])
            parent_frame_3 = self.get_parent_frame_for_ref_frame(plane.point_names[2])

            if (not (parent_frame_1 == parent_frame_2 == parent_frame_3) or 
                parent_frame_1 is None or 
                parent_frame_2 is None or 
                parent_frame_3 is None):
                self.logger.error(f"Given frames do not have the same parent frame or ref frame does not exist. Plane could not be created!")
                return False

            list_to_append_plane = []

            # try to get parent object of ref frame 
            obj = self.get_obj_by_name(parent_frame_1)
            
            if obj is None:
                list_to_append_plane = self.scene.planes_in_scene
            else:
                list_to_append_plane = obj.ref_planes

            for ind, _plane in enumerate(list_to_append_plane):
                _plane:soi_msgs.Plane
                if _plane.ref_plane_name == plane.ref_plane_name:
                    list_to_append_plane[ind] = plane
                    return True
            
            # if above for loop executes without returning append the plane because it does not yet excist.
            list_to_append_plane.append(plane)
            self.logger.info(f"Plane '{plane.ref_plane_name}' created! Plane is defined by frames 1.{plane.point_names[0]}, 2.{plane.point_names[1]}, 3.{plane.point_names[2]}.")
            return True

        except Exception as e:
            self.logger.error(e)
            self.logger.error(f"Plane could not be created. Invalid message!")
            return False
    
    def get_parent_frame_for_ref_frame(self,frame_name:str)->str:
        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            for ref_frame in obj.ref_frames:
                ref_frame: soi_msgs.RefFrame
                if ref_frame.frame_name == frame_name:
                    return ref_frame.parent_frame
                
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame: soi_msgs.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame.parent_frame    

        return None 

    # def is_ref_frame_connected_to_obj(self, ref_frame_name:str)->(bool,str):
    #     pass
    
    def modify_pose(self,frame_obj_name:str, rel_pose: Pose)-> bool:

        pose_to_modify = None

        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            if obj.obj_name == frame_obj_name:
                pose_to_modify = obj.obj_pose
                break
            for ref_frame in obj.ref_frames:
                ref_frame:soi_msgs.RefFrame
                if ref_frame.frame_name == frame_obj_name:
                    pose_to_modify = ref_frame.pose
                    break
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame:soi_msgs.RefFrame
            if ref_frame.frame_name == frame_obj_name:
                pose_to_modify = ref_frame.pose
                break
        
        if not pose_to_modify is None:
            pose_to_modify.position.x += rel_pose.position.x
            pose_to_modify.position.y += rel_pose.position.y
            pose_to_modify.position.z += rel_pose.position.z
            pose_to_modify.orientation = quaternion_multiply(pose_to_modify.orientation,rel_pose.orientation)
            self.publish_information()
            self.logger.info(f'Pose for object {frame_obj_name} updated!')
            return True
        else:
            self.logger.warn(f"Pose could not be updated. Frame '{frame_obj_name}' not found!")
            return False
    
def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
