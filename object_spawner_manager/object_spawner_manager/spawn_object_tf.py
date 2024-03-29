#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np
from spawn_object_interfaces.srv import DestroyObject
from spawn_object_interfaces.srv import SpawnObject
from spawn_object_interfaces.msg import Object
from spawn_object_interfaces.msg import RefFrame
import spawn_object_interfaces.msg as soi_msgs
import tf2_geometry_msgs
import spawn_object_interfaces.srv as soi_srvs
import copy
from spawn_object_interfaces.srv import ChangeParentFrame
from spawn_object_interfaces.srv import CreateRefFrame
from spawn_object_interfaces.srv import DeleteRefFrame
from spawn_object_interfaces.srv import GetInfo
from spawn_object_interfaces.srv import ModifyPose

from geometry_msgs.msg import Vector3, Quaternion
import sympy as sp
from typing import Union
from scipy.optimize import minimize, least_squares

def get_euler_rotation_matrix(alpha, beta, gamma):
    rotation_z = sp.Matrix([
        [sp.cos(alpha), -sp.sin(alpha), 0],
        [sp.sin(alpha), sp.cos(alpha), 0],
        [0, 0, 1]
    ])

    rotation_y = sp.Matrix([
        [sp.cos(beta), 0, sp.sin(beta)],
        [0, 1, 0],
        [-sp.sin(beta), 0, sp.cos(beta)]
    ])

    rotation_x = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(gamma), -sp.sin(gamma)],
        [0, sp.sin(gamma), sp.cos(gamma)]
    ])

    rotation_matrix = rotation_z * rotation_y * rotation_x
    return rotation_matrix

def rotation_matrix_to_quaternion(R)-> sp.Matrix:
    trace_R = R[0, 0] + R[1, 1] + R[2, 2]
    w = sp.sqrt(1 + trace_R) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    return sp.Matrix([w, x, y, z])

def quaternion_to_rotation_matrix(quaternion:Quaternion)-> sp.Matrix:
    w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
    r = sp.Matrix([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return r

def euler_to_quaternion(roll:float, pitch:float, yaw:float)-> Quaternion:
    """
    Convert Euler angles to quaternion.

    Parameters:
    - roll: Rotation angle around the x-axis (in radians)
    - pitch: Rotation angle around the y-axis (in radians)
    - yaw: Rotation angle around the z-axis (in radians)

    Returns:
    - Quaternion
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    result = Quaternion()
    result.w = w
    result.x = x
    result.y = y
    result.z = z
    return result

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
    t:TransformStamped = tf_buffer.lookup_transform(child_frame, new_parent_frame,rclpy.time.Time())
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


def get_transform_for_frame_in_world(frame_name: str, tf_buffer: Buffer) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    transform:TransformStamped = tf_buffer.lookup_transform('world', frame_name, rclpy.time.Time())
    return transform

def get_point_from_ros_obj(position: Union[Vector3, Point]) -> sp.Point3D:
    
    if isinstance(position, Vector3):
        position:Vector3
        point = sp.Point3D(position.x, position.y, position.z, evaluate=False)
    elif isinstance(position, Point):
        position:Point
        point = sp.Point3D(position.x, position.y, position.z, evaluate=False)
    else:
        raise ValueError
    
    return point

def get_plane_from_frame_names(frames: list[str], tf_buffer: Buffer)-> sp.Plane:

    if len(frames)!=3:
        raise ValueError
    
    t1:TransformStamped = get_transform_for_frame_in_world(frames[0], tf_buffer)
    t2:TransformStamped = get_transform_for_frame_in_world(frames[1], tf_buffer)
    t3:TransformStamped = get_transform_for_frame_in_world(frames[2], tf_buffer)

    p1 = get_point_from_ros_obj(t1.transform.translation)
    p2 = get_point_from_ros_obj(t2.transform.translation)
    p3 = get_point_from_ros_obj(t3.transform.translation)

    plane = sp.Plane(p1,p2,p3)

    return plane

def get_line3d_from_frame_names(frames: list[str], tf_buffer: Buffer)-> sp.Line3D:

    if len(frames)!=2:
        raise ValueError
    
    t1:TransformStamped = get_transform_for_frame_in_world(frames[0], tf_buffer)
    t2:TransformStamped = get_transform_for_frame_in_world(frames[1], tf_buffer)

    p1 = get_point_from_ros_obj(t1.transform.translation)
    p2 = get_point_from_ros_obj(t2.transform.translation)

    line = sp.Line3D(p1,p2)

    return line

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

def get_transform_matrix_from_tf(tf: Union[Pose, TransformStamped])-> sp.Matrix:
    if isinstance(tf, Pose):
        tf: Pose
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.orientation)
        t = sp.Matrix([tf.position.x, tf.position.y, tf.position.z]) 
    elif isinstance(tf, TransformStamped):
        tf: TransformStamped
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = sp.Matrix([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])         
    else:
        return None
    transform_matrix = sp.eye(4)

    transform_matrix[:3, :3] = r
    transform_matrix[:3, 3] = t
    return transform_matrix
    
def transform_matrix_to_pose(transform_matrix:sp.Matrix)-> Pose:
    # Extract the rotation matrix and translation vector from the transformation matrix
    rotation_matrix = transform_matrix[:3, :3]
    translation_vector = transform_matrix[:3, 3]

    # Convert the rotation matrix to a quaternion
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    # Create a Pose message
    pose_msg = Pose(
        position=Point(x=float(translation_vector[0]), y=float(translation_vector[1]), z=float(translation_vector[2])),
        orientation=Quaternion(x=float(quaternion[1]), y=float(quaternion[2]), z=float(quaternion[3]), w=float(quaternion[0])))
    return pose_msg 

def euler_to_matrix(angles:list):
    Rz = sp.Matrix([[sp.cos(angles[2]), -1*sp.sin(angles[2]), 0],
                [sp.sin(angles[2]), sp.cos(angles[2]), 0],
                [0, 0, 1]])

    Ry = sp.Matrix([[sp.cos(angles[1]), 0, sp.sin(angles[1])],
                [0, 1, 0],
                [-1*sp.sin(angles[1]), 0, sp.cos(angles[1])]])

    Rx = sp.Matrix([[1, 0, 0],
                [0, sp.cos(angles[0]), -1*sp.sin(angles[0])],
                [0, sp.sin(angles[0]), sp.cos(angles[0])]])

    return Rz * Ry * Rx

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
        
        self.create_axis_srv = self.create_service(soi_srvs.CreateAxis,f'object_spawner_manager/create_axis',self.create_axis,callback_group=self.callback_group)      

        self.create_assembly_instructions_srv = self.create_service(soi_srvs.CreateAssemblyInstructions,f'object_spawner_manager/create_assembly_instructions',self.srv_create_assembly_instructions,callback_group=self.callback_group)      

        self.calculate_assembly_instructions_srv = self.create_service(soi_srvs.CalculateAssemblyInstructions,f'object_spawner_manager/calculate_assembly_instructions',self.calculate_assembly_instructions,callback_group=self.callback_group)      

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
        create_success = self.object_scene.create_ref_plane(plane = request.ref_plane)
        response.success = create_success
        return response

    def create_axis(self,request:soi_srvs.CreateAxis.Request, response:soi_srvs.CreateAxis.Response):
        create_success = self.object_scene.create_axis(axis = request.axis)
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
    
    def srv_create_assembly_instructions(self, request: soi_srvs.CreateAssemblyInstructions.Request, response: soi_srvs.CreateAssemblyInstructions.Response):
        create_success = self.object_scene.create_assembly_instructions(instruction=request.assembly_instruction)
        response.success = create_success
        return response
    
    def calculate_assembly_instructions(self, request: soi_srvs.CalculateAssemblyInstructions.Request, response: soi_srvs.CalculateAssemblyInstructions.Response):
        
        transfrom = self.object_scene.get_assembly_transformation_by_id(request.instruction_id)
        if transfrom == None:
            response.success = False
        else:
            response.success = True
            response.assembly_transform = transfrom
        return response
    
def get_point_of_plane_intersection(plane1: sp.Plane, plane2: sp.Plane, plane3: sp.Plane) -> sp.Point3D:
    line = plane1.intersection(plane2)
    # Get the first point of intersection, should also be the only one
    #inter:sp.Point3D = plane3.intersection(line[0])
    inter:sp.Point3D = plane3.intersection(line[0])[0]

    if not isinstance(inter, sp.Point3D):
        raise ValueError(f"Given planes (1.{plane1}, 2.{plane2}, 3.{plane3}) do not have a single point of intersection. Invalid plane selection!")
    
    # Value Error if not a point

    return inter

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
        #self.logger.info("Object Scene has been published")

    def add_obj_to_scene(self, new_obj:soi_msgs.Object)-> bool:

        if new_obj.obj_name == "":
            self.logger.error(f"Name of the component should not be empty. Aboarted!")
            return False
        
        name_conflict, _ = self.check_ref_frame_exists(new_obj.obj_name)

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
                    new_obj.obj_pose.orientation = check_and_return_quaternion(new_obj.obj_pose.orientation,self.logger)
                    obj.cad_data = new_obj.cad_data
                    obj.obj_pose = new_obj.obj_pose
                    obj.parent_frame = new_obj.parent_frame

                    self.logger.warn(f'Service for spawning {new_obj.obj_name} was called, but object does already exist!')
                    self.logger.warn(f'Information for {new_obj.obj_name} updated!')
                    self.publish_scene()
                    self.publish_to_tf()
                    return True
            # eventually 
            return False

    def add_ref_frame_to_scene(self, new_ref_frame:soi_msgs.RefFrame)-> bool:

        # checks if ref frame frame exists
        ref_frame_existend, parent_frame = self.check_ref_frame_exists(new_ref_frame.frame_name)

        # checks if object with same name exists
        name_conflict_1 = self.check_object_exists(new_ref_frame.frame_name)

        # checks if tf frame exists
        name_conflict_2 = self.check_if_frame_exists(new_ref_frame.frame_name)

        if not ref_frame_existend and (name_conflict_1 or name_conflict_2):
            self.logger.error(f"Ref frame '{new_ref_frame.frame_name}' can not have the same name as an existing reference frame or object!")
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
            if parent_frame != new_ref_frame.parent_frame:
                frame_list_to_delete = self.get_obj_by_name(parent_frame).ref_frames
                for index, frame in enumerate(frame_list_to_delete):
                    frame: soi_msgs.RefFrame
                    if frame.frame_name == new_ref_frame.frame_name:
                        del frame_list_to_delete[index]
                        self.logger.warn(f'Service for creating {new_ref_frame.frame_name} was called, but frame does already exist! Information for {new_ref_frame.frame_name} updated!')
        
            for index, frame in enumerate(frame_list_to_append_to):
                frame: soi_msgs.RefFrame
                if frame.frame_name == new_ref_frame.frame_name:
                    del frame_list_to_append_to[index]
                    self.logger.warn(f'Service for creating {new_ref_frame.frame_name} was called, but frame does already exist! Information for {new_ref_frame.frame_name} updated!')
                
            frame_list_to_append_to.append(new_ref_frame)

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
            #self.logger.info(f"TF for '{ref_frame.frame_name}' published!")

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

            #self.logger.info(f"TF for object'{obj.obj_name}' published!")

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
                #self.logger.info(f"TF for '{ref_frame.frame_name}' published!")

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
    
    def check_ref_frame_exists(self,name_frame:str) -> Union[bool,str]:
        # this function checks if an frame esixts in the frame list
        # iterate over ref_frames
        for ref_frame in self.scene.ref_frames_in_scene:
            ref_frame:soi_msgs.RefFrame
            if ref_frame.frame_name == name_frame:
                return True, None
        
        # iterate over objects
        for obj in self.scene.objects_in_scene:
            obj: soi_msgs.Object
            # iterate over ref_frames in object
            for obj_ref_frame in obj.ref_frames:
                obj_ref_frame:soi_msgs.RefFrame
                if obj_ref_frame.frame_name == name_frame:
                    return True, obj.obj_name
                
        return False, None

    def create_axis(self, axis: soi_msgs.Axis) -> bool:

        try:
            if axis.axis_name == "":
                self.logger.error(f"Name of the axis should not be empty. Aboarted!")
                return False
            
            if not len(axis.point_names) == 2:
                self.logger.error(f"Not enough input arguments. Plane could not be created!")
                return False
            
            parent_frame_1 = self.get_parent_frame_for_ref_frame(axis.point_names[0])
            parent_frame_2 = self.get_parent_frame_for_ref_frame(axis.point_names[1])

            if (not (parent_frame_1 == parent_frame_2) or 
                parent_frame_1 is None or 
                parent_frame_2 is None):
                self.logger.error(f"Given frames do not have the same parent frame or ref frame does not exist. Axis could not be created!")
                return False

            # Check if frames form a valid plane
            try:
                test_axis:sp.Line3D = get_line3d_from_frame_names(axis.point_names, tf_buffer=self.tf_buffer)
            except ValueError as e:
                self.logger.error(f"Given frames do not form a valid plane. Plane could not be created!")
                return False

            list_to_append_axis= []

            # try to get parent object of ref frame 
            obj = self.get_obj_by_name(parent_frame_1)
            
            if obj is None:
                list_to_append_axis = self.scene.axis_in_scene
            else:
                list_to_append_axis = obj.ref_axis

            for ind, _axis in enumerate(list_to_append_axis):
                _axis:soi_msgs.Axis
                if _axis.axis_name == axis.axis_name:
                    list_to_append_axis[ind] = axis
                    return True
            
            # if above for loop executes without returning append the plane because it does not yet excist.
            list_to_append_axis.append(axis)
            self.logger.info(f"Axis '{axis.axis_name}' created! Axis is defined by frames 1.{axis.point_names[0]}, 2.{axis.point_names[1]}.")
            return True

        except Exception as e:
            self.logger.error(e)
            self.logger.error(f"Plane could not be created. Invalid message!")
            return False
        
    def create_ref_plane(self, plane: soi_msgs.Plane) -> bool:
        try:
            if plane.ref_plane_name == "":
                self.logger.error(f"Name of the plane should not be empty. Aboarted!")
                return False

            if (plane.axis_names[0]=='' and
                plane.point_names[0]!='' and 
                plane.point_names[1]!='' and 
                plane.point_names[2]!=''):
                mode = 'PlanePPP'
                parent_frame_1 = self.get_parent_frame_for_ref_frame(plane.point_names[0])
                parent_frame_2 = self.get_parent_frame_for_ref_frame(plane.point_names[1])
                parent_frame_3 = self.get_parent_frame_for_ref_frame(plane.point_names[2])

            elif (plane.axis_names[0]!='' and
                plane.point_names[0]!='' and 
                plane.point_names[1]=='' and 
                plane.point_names[2]==''):
                mode = 'PlaneAP'
                axis_msg = self.get_axis_from_scene(plane.axis_names[0])
                parent_frame_1 = self.get_parent_frame_for_ref_frame(axis_msg.point_names[0])
                parent_frame_2 = self.get_parent_frame_for_ref_frame(axis_msg.point_names[1])
                parent_frame_3 = self.get_parent_frame_for_ref_frame(plane.point_names[0])
            else:
                self.logger.error(f"Invalid input for creation of reference plane. Plane should be defined by 3 x frames or by 1 x axis + 1 x frame!")
                return False
            

            if (not (parent_frame_1 == parent_frame_2 == parent_frame_3) or 
                parent_frame_1 is None or 
                parent_frame_2 is None or 
                parent_frame_3 is None):
                self.logger.error(f"Given frames do not have the same parent frame or ref frame does not exist. Plane could not be created!")
                return False

            # Check if frames form a valid plane
            try:
                if mode == 'PlanePPP':
                    test_plane: sp.Plane = get_plane_from_frame_names(plane.point_names, tf_buffer=self.tf_buffer)
                    logger_message = f"Plane '{plane.ref_plane_name}' created! Plane is defined by frames 1.{plane.point_names[0]}, 2.{plane.point_names[1]}, 3.{plane.point_names[2]}."
                else:
                    test_plane: sp.Plane = self.get_plane_from_axis_and_frame(plane.axis_names[0],plane.point_names[0])
                    logger_message = f"Plane '{plane.ref_plane_name}' created! Plane is defined by axis '{plane.axis_names[0]}' and point '{plane.point_names[0]}'."

            except ValueError as e:
                self.logger.error(f"Given frames do not form a valid plane. Plane could not be created!")
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
            self.logger.info(logger_message)
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
    
    def create_assembly_instructions(self,instruction: soi_msgs.AssemblyInstruction)->bool:
        # Get plane msgs for object 1
        if instruction.id == "":
                self.logger.error(f"ID of the instruction shoud not be empty. Aboarted!")
                return False
            
        obj1_plane1_msg = self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1)
        obj1_plane2_msg = self.get_plane_from_scene(instruction.plane_match_2.plane_name_component_1)
        obj1_plane3_msg = self.get_plane_from_scene(instruction.plane_match_3.plane_name_component_1)

        # Get plane msgs for object 2
        obj2_plane1_msg = self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2)
        obj2_plane2_msg = self.get_plane_from_scene(instruction.plane_match_2.plane_name_component_2)
        obj2_plane3_msg = self.get_plane_from_scene(instruction.plane_match_3.plane_name_component_2) 

        # return false if planes do not exist in scene
        if (obj1_plane1_msg is None or
            obj1_plane2_msg is None or
            obj1_plane3_msg is None or
            obj2_plane1_msg is None or
            obj2_plane2_msg is None or
            obj2_plane3_msg is None):
            self.logger.error(f"At least one of the given plane names does not exist ('{obj1_plane1_msg}', '{obj1_plane2_msg}', '{obj1_plane3_msg}', '{obj2_plane1_msg}', '{obj2_plane2_msg}', '{obj2_plane3_msg}'). Invalid input!")
            return False
        
        # # Get parent frame of the first point of the respective planes to test for parent frames
        # obj1_plane1_p1_parent_frame = self.get_parent_frame_for_ref_frame(obj1_plane1_msg.point_names[0])
        # obj1_plane2_p1_parent_frame = self.get_parent_frame_for_ref_frame(obj1_plane2_msg.point_names[1])
        # obj1_plane3_p1_parent_frame = self.get_parent_frame_for_ref_frame(obj1_plane3_msg.point_names[2])

        # obj2_plane1_p1_parent_frame = self.get_parent_frame_for_ref_frame(obj2_plane1_msg.point_names[0])
        # obj2_plane2_p1_parent_frame = self.get_parent_frame_for_ref_frame(obj2_plane2_msg.point_names[1])
        # obj2_plane3_p1_parent_frame = self.get_parent_frame_for_ref_frame(obj2_plane3_msg.point_names[2])

        # self.logger.warn(f"{obj1_plane1_p1_parent_frame}, {obj1_plane2_p1_parent_frame}, {obj1_plane3_p1_parent_frame}, {obj2_plane1_p1_parent_frame}, {obj2_plane2_p1_parent_frame}, {obj2_plane3_p1_parent_frame},")
        # if obj1_plane1_p1_parent_frame != obj1_plane2_p1_parent_frame != obj1_plane3_p1_parent_frame:
        #     self.logger.error(f"Planes for component 1 do not belong to the same parent frame. Invalid input!")
        #     return False

        # if obj2_plane1_p1_parent_frame != obj2_plane2_p1_parent_frame != obj2_plane3_p1_parent_frame:
        #     self.logger.error(f"Planes for component 2 do not belong to the same parent frame. Invalid input!")
        #     return False

        try:
            transfrom = self.calculate_assembly_transformation(instruction)
        except ValueError as e:
            self.logger.error(str(e))
            return False    
        except Exception as e:
            self.logger.error(str(e))
            self.logger.error(f"Fatal Error")
            return False
        
        inst_exists=False
        for index, _inst in enumerate(self.scene.assembly_instructions):
            _inst : soi_msgs.AssemblyInstruction
            if _inst.id == instruction.id:
                del self.scene.assembly_instructions[index]
                self.scene.assembly_instructions.append(instruction)
                _inst = instruction
                inst_exists=True
                break
        if not inst_exists:
            self.scene.assembly_instructions.append(instruction)

        return True
    
    def _get_plane_obj_from_scene(self, plane_name:str)-> sp.Plane:
        plane_msg = self.get_plane_from_scene(plane_name)
        plane_msg: soi_msgs.Plane
        if (plane_msg.axis_names[0]=='' and
            plane_msg.point_names[0]!='' and 
            plane_msg.point_names[1]!='' and 
            plane_msg.point_names[2]!=''):
            
            plane = get_plane_from_frame_names(frames = plane_msg.point_names , tf_buffer=self.tf_buffer)

        elif (plane_msg.axis_names[0]!='' and
            plane_msg.point_names[0]!='' and 
            plane_msg.point_names[1]=='' and 
            plane_msg.point_names[2]==''):
            plane = self.get_plane_from_axis_and_frame(plane_msg.axis_names[0], plane_msg.point_names[0])
        else:
            plane = None

        return plane
    
    def get_assembly_instruction_by_id(self, instruction_id:str)->soi_msgs.AssemblyInstruction:
        for instruction in self.scene.assembly_instructions:
            instruction: soi_msgs.AssemblyInstruction
            if instruction.id == instruction_id:
                return instruction
        return None
    
    def get_assembly_transformation_by_id(self, instruction_id:str)->Pose:
        instruction = self.get_assembly_instruction_by_id(instruction_id)
        if instruction is None:
            return None
        else:
            assembly_transform = self.calculate_assembly_transformation(instruction)
            return assembly_transform
    
    def calculate_assembly_transformation(self, instruction:soi_msgs.AssemblyInstruction)->Pose:

        obj_1_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_1)
        obj_1_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_1)
        obj_1_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_1)

        obj_1_name = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_1).point_names[0])
        obj_2_name = self.get_parent_frame_for_ref_frame(self.get_plane_from_scene(instruction.plane_match_1.plane_name_component_2).point_names[0])
        
        obj_1_mate_plane_intersection: sp.Point3D = get_point_of_plane_intersection(obj_1_plane_1, obj_1_plane_2, obj_1_plane_3)
        
        obj_2_plane_1 = self._get_plane_obj_from_scene(instruction.plane_match_1.plane_name_component_2)
        obj_2_plane_2 = self._get_plane_obj_from_scene(instruction.plane_match_2.plane_name_component_2)
        obj_2_plane_3 = self._get_plane_obj_from_scene(instruction.plane_match_3.plane_name_component_2)
        obj_2_mate_plane_intersection: sp.Point3D = get_point_of_plane_intersection(obj_2_plane_1, obj_2_plane_2, obj_2_plane_3)

        #self.logger.warn(f"Intersection Obj1: {str(obj_1_mate_plane_intersection)}")
        #self.logger.warn(f"Intersection Obj2: {str(obj_2_mate_plane_intersection)}")
        assembly_transform = Pose()

        if instruction.component_1_is_moving_part:
            moving_component = obj_1_name
            static_component = obj_2_name
            moving_component_plane_intersection = obj_1_mate_plane_intersection
            static_component_plane_intersection = obj_2_mate_plane_intersection
        else:
            moving_component = obj_2_name
            static_component = obj_1_name
            moving_component_plane_intersection = obj_2_mate_plane_intersection
            static_component_plane_intersection = obj_1_mate_plane_intersection

        self.logger.warn(f"\nMoving component: '{moving_component}'\nStatic component: '{static_component}'")
        assembly_transform.position.x = float(static_component_plane_intersection.x - moving_component_plane_intersection.x)
        assembly_transform.position.y = float(static_component_plane_intersection.y - moving_component_plane_intersection.y)
        assembly_transform.position.z = float(static_component_plane_intersection.z - moving_component_plane_intersection.z)
    
        bvec_obj_1_1=sp.Matrix(obj_1_plane_1.normal_vector).normalized().evalf()
        #bvec_obj_1_2=-1*sp.Matrix(obj_1_plane_2.normal_vector).normalized().evalf()
        bvec_obj_1_2=sp.Matrix(obj_1_plane_2.normal_vector).normalized().evalf()
        bvec_obj_1_3=-1*sp.Matrix(obj_1_plane_3.normal_vector).normalized().evalf()

        basis_obj_1: sp.Matrix = sp.Matrix.hstack(bvec_obj_1_1, bvec_obj_1_2, bvec_obj_1_3)

        if bvec_obj_1_1.cross(bvec_obj_1_2) == bvec_obj_1_3:
            self.logger.warn("It is right hand")
        else:
            self.logger.warn(f"Not right hand {bvec_obj_1_1.cross(bvec_obj_1_2).evalf()} != {bvec_obj_1_3.evalf()}")

        #self.logger.warn(f"Eignevalues B2: {basis_obj_1.eigenvals()}")
        #self.logger.warn(f"Basis obj 1 is: {basis_obj_1.evalf()}")

        bvec_obj_2_1=-1*sp.Matrix(obj_2_plane_1.normal_vector).normalized().evalf()
        bvec_obj_2_2=sp.Matrix(obj_2_plane_2.normal_vector).normalized().evalf()
        bvec_obj_2_3=-1*sp.Matrix(obj_2_plane_3.normal_vector).normalized().evalf()

        basis_obj_2: sp.Matrix = sp.Matrix.hstack(bvec_obj_2_1, bvec_obj_2_2, bvec_obj_2_3)
        #self.logger.warn(f"Basis obj 2 is: {basis_obj_2.evalf()}")

        if instruction.component_1_is_moving_part:
            rot_matrix = basis_obj_2 * basis_obj_1.inv()
        else:
            rot_matrix = basis_obj_1 * basis_obj_2.inv()

        det_rot_matrix= rot_matrix.det()
        self.logger.warn(f"Rot obj 2 to 1: {rot_matrix.evalf()}")

        #self.logger.warn(f"Eigenvalues Rot: {rot_matrix.eigenvals()}")
        #self.logger.warn(f"Det Rot: {det_rot_matrix}")
               
        if not round(det_rot_matrix, 9) == 1.0:
            self.logger.warn(f"Invalid plane selection")
            #return False

        # Calculate the approx quaternion
        assembly_transform.orientation = self.calc_approx_quat_from_matrix(rot_matrix)

        #self.logger.info(f"Assembly transformation is: {assembly_transform.__str__()}")
        self.logger.info(f"""Assembly transformation is: \n
                         x: {assembly_transform.position.x},\n
                         y: {assembly_transform.position.y},\n
                         z: {assembly_transform.position.z},\n
                         w: {assembly_transform.orientation.w},\n
                         x: {assembly_transform.orientation.x},\n
                         y: {assembly_transform.orientation.y},\n
                         z: {assembly_transform.orientation.z}""")

        add_success = self.add_assembly_frames_to_scene(instruction.id,
                                                        moving_component,
                                                        static_component,
                                                        moving_component_plane_intersection,
                                                        static_component_plane_intersection,
                                                        assembly_transform)
        if not add_success:
            raise Exception
        
        return assembly_transform
    
    def add_assembly_frames_to_scene(   self,
                                        instruction_id:str,
                                        moving_component: str, 
                                        static_component:str, 
                                        moving_component_plane_intersection:sp.Point3D, 
                                        static_component_plane_intersection:sp.Point3D, 
                                        assembly_transform: Pose)-> bool:
        
        assembly_frame = soi_msgs.RefFrame()
        assembly_frame.frame_name = f"assembly_frame_{instruction_id}"
        assembly_frame.parent_frame = moving_component
        assembly_frame.pose.position.x = float(moving_component_plane_intersection.x)
        assembly_frame.pose.position.y = float(moving_component_plane_intersection.y)
        assembly_frame.pose.position.z = float(moving_component_plane_intersection.z)
        moving_component_world_transform = get_transform_for_frame_in_world(moving_component, self.tf_buffer)
        assembly_frame.pose.orientation = moving_component_world_transform.transform.rotation
        assembly_frame_matrix = get_transform_matrix_from_tf(assembly_frame.pose)
        moving_component_matrix = get_transform_matrix_from_tf(moving_component_world_transform)

        helper_pose = transform_matrix_to_pose(moving_component_matrix.inv()*assembly_frame_matrix)
        assembly_frame.pose = helper_pose
        add_assembly_frame_success = self.add_ref_frame_to_scene(assembly_frame)

        # Create frame for the static component
        target_frame = soi_msgs.RefFrame()
        target_frame.frame_name = f"target_frame_{instruction_id}"
        target_frame.parent_frame = static_component
        target_frame.pose.position.x = float(static_component_plane_intersection.x)
        target_frame.pose.position.y = float(static_component_plane_intersection.y)
        target_frame.pose.position.z = float(static_component_plane_intersection.z)
        #target_frame.pose.orientation = quaternion_multiply(assembly_transfrom.orientation, target_frame.pose.orientation)
        target_frame.pose.orientation = quaternion_multiply(assembly_transform.orientation, moving_component_world_transform.transform.rotation)

        static_component_world_transform = get_transform_for_frame_in_world(static_component, self.tf_buffer)
        target_frame_matrix = get_transform_matrix_from_tf(target_frame.pose)
        static_component_matrix = get_transform_matrix_from_tf(static_component_world_transform)
        helper_pose_2 = transform_matrix_to_pose(static_component_matrix.inv()*target_frame_matrix)
        target_frame.pose = helper_pose_2
        add_target_frame_success = self.add_ref_frame_to_scene(target_frame)
        result = add_assembly_frame_success and add_target_frame_success
        return result
        
    def calc_approx_quat_from_matrix(self, rot_mat: sp.Matrix) -> Quaternion:
        initial_guess = np.array([0, 0, 0])
        
        def cost_function(params, target_matrix):
            alpha, beta, gamma = params
            rotation_matrix = get_euler_rotation_matrix(alpha,beta,gamma)
            difference = (rotation_matrix - target_matrix).norm()
            return difference
        
        self.logger.warn(f"Starting calculating the transformation...")
        max_iter = 1000

        #result = minimize(cost_function, initial_guess, args=(rot_obj2_to_obj1),  method='L-BFGS-B', tol = 1e-10, options={'maxiter': 1000})
        result = minimize(cost_function, initial_guess, args=(rot_mat),  method='Nelder-Mead', tol = 1e-20, options={'maxiter': max_iter})

        if max_iter == result.nit:
            self.logger.info(f"Max iterations reached ({max_iter}). Measurement inacurate.")
        else:
            self.logger.info(f"Iterations ran: {result.nit}")
        self.logger.info(f"Residual error: {result.fun}")
        self.logger.info(f"Result (deg) is: \nalpha: {result.x[0]*(180/np.pi)}, \nbeta: {result.x[1]*(180/np.pi)}, \ngamma:{result.x[2]*(180/np.pi)}")

        quaternion = euler_to_quaternion(   roll   = result.x[2],
                                            pitch  = result.x[1],
                                            yaw    = result.x[0])
        return quaternion
    
    def get_plane_from_scene(self, plane_name:str)-> soi_msgs.Plane:
        plane_msg = None
        for plane in self.scene.planes_in_scene:
            plane: soi_msgs.Plane
            if plane_name == plane.ref_plane_name:
                plane_msg = plane
                break

        for obj in self.scene.objects_in_scene:
            obj:soi_msgs.Object
            for plane in obj.ref_planes:
                plane: soi_msgs.Plane
                if plane_name == plane.ref_plane_name:
                    plane_msg = plane
                    break

        return plane_msg
    
    def get_axis_from_scene(self, axis_name:str)-> soi_msgs.Axis:
        axis_msg = None
        for axis in self.scene.axis_in_scene:
            axis: soi_msgs.Axis
            if axis_name == axis.axis_name:
                axis_msg = axis
                break

        for obj in self.scene.objects_in_scene:
            obj:soi_msgs.Object
            for axis in obj.ref_axis:
                axis: soi_msgs.Axis
                if axis_name == axis.axis_name:
                    axis_msg = axis
                    break
        return axis_msg
    
    def get_plane_from_axis_and_frame(self, axis_name: str, frame_name: str)-> sp.Plane:
        axis_msg = self.get_axis_from_scene(axis_name)

        line3d:sp.Line3D = get_line3d_from_frame_names(axis_msg.point_names, tf_buffer=self.tf_buffer)

        t_point:TransformStamped = get_transform_for_frame_in_world(frame_name, self.tf_buffer)
        point = get_point_from_ros_obj(t_point.transform.translation)

        plane = sp.Plane(point, normal_vector = line3d.direction)

        return plane

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
