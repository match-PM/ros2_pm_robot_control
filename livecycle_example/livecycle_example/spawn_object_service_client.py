#!/usr/bin/env python3

import rclpy

import sys
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

from typing import Optional
from lifecycle_msgs.msg import Transition, State, TransitionDescription
from lifecycle_msgs.srv import GetState, ChangeState
from rclpy.node import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from spawn_object_interfaces.srv import SpawnObject
from spawn_object_interfaces.srv import DestroyObject

from launch import LaunchService
import subprocess
import time

class SpawnObjectServiceClient(Node):
    def __init__(self,**kwargs):
        super().__init__("SpawnObjectServiceClient",**kwargs)

        self.get_logger().info('SpawnObjectServiceClient node started! You can start spawning Objects now!')

        name_get_state = 'object_topics_publisher/get_state'
        name_change_state = 'object_topics_publisher/change_state'

        self.client_get_state = self.create_client(GetState,name_get_state)
        
        self.client_change_state = self.create_client(ChangeState,name_change_state)

        # while not self.client_get_state.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')

        # while not self.client_change_state.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')


        self.srv = self.create_service(SpawnObject,'spawn_objects',self.spawn_object_callback)
        #self.srv = self.create_service(DestroyObject,'destroy_objects',self.destroy_object_callback)
        # ros2 service call /spawn_objects spawn_object_interfaces/srv/SpawnObject "{obj_name: my_test_obj translation: [1.0, 2.0, 3.0], rotation: [1.0, 2.0,3.0,4.0]}"

        #self.req = GetState.Request()
        
        self.spawned_objects_dict_list=[]

        #result_stae = self.change_state()
        #result=self.send_request()
        
    def spawn_object_callback(self, request, response):
        # Access the vectors in the request object
        obj_name=request.obj_name
        stl_path=request.stl_path
        translation = request.translation
        rotation = request.rotation
        
        print(translation)
        print(rotation)
        translation_str=','.join(str(item) for item in translation)
        rotation_str=','.join(str(item) for item in rotation)

        name_get_state='objects/'+obj_name+'/get_state'
        name_change_state='objects/'+obj_name+'/change_state'

        self.spawned_objects_dict_list.append({'name':obj_name,
                                            'get_state_client':self.create_client(GetState,name_get_state),
                                            'change_state_client':self.create_client(ChangeState,name_change_state)})
        
        command=f'ros2 launch lc_spawn_object spawn_object.launch.py "obj_name:={obj_name}" "obj_translation:={translation_str}" "obj_rotation:={rotation_str}"'
        print(command)
        process = subprocess.Popen(command, shell=True)

        change_req = ChangeState.Request()
        t = Transition()
        t.id=Transition.TRANSITION_CONFIGURE
        change_req.transition=t
        # my_dict=self.spawned_objects_dict_list[0]
        # client=my_dict['get_state_client']
        # print(name_change_state)

        _change_state_client=self.create_client(ChangeState,name_change_state)
        #time.sleep(0.5)
        self.future = _change_state_client.call_async(change_req)
        rclpy.spin_until_future_complete(self, self.future)

        t.id=Transition.TRANSITION_ACTIVATE
        change_req.transition=t
        self.future = _change_state_client.call_async(change_req)
        rclpy.spin_until_future_complete(self, self.future)

        # Perform your desired operations on the vectors
        # Here's a simple example that calculates the sum of corresponding elements
        # Do something with the result if needed
        response.success = True
        # Return an empty response (since our service doesn't have a response message)
        return response
       
    def send_request(self):
        self.future = self.client_get_state.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def change_state(self):
        self.change_req = ChangeState.Request()
        t = Transition()

        t.id=Transition.TRANSITION_CONFIGURE

        self.ch_req.transition=t    
        self.future = self.client_change_state.call_async(self.ch_req)
        rclpy.spin_until_future_complete(self, self.future)
        time.sleep(5)

        t.id=Transition.TRANSITION_ACTIVATE

        self.ch_req.transition=t    
        self.future = self.client_change_state.call_async(self.ch_req)
        rclpy.spin_until_future_complete(self, self.future)
        time.sleep(5)

        t.id=Transition.TRANSITION_DEACTIVATE

        self.ch_req.transition=t    
        self.future = self.client_change_state.call_async(self.ch_req)
        rclpy.spin_until_future_complete(self, self.future)
        time.sleep(5)

        t.id=Transition.TRANSITION_CLEANUP

        self.ch_req.transition=t    
        self.future = self.client_change_state.call_async(self.ch_req)
        rclpy.spin_until_future_complete(self, self.future)
        time.sleep(5)


        t.id=Transition.TRANSITION_UNCONFIGURED_SHUTDOWN

        self.ch_req.transition=t    
        self.future = self.client_change_state.call_async(self.ch_req)
        rclpy.spin_until_future_complete(self, self.future)
        time.sleep(5)

        return self.future.result()
    






def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = SpawnObjectServiceClient()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
  main()