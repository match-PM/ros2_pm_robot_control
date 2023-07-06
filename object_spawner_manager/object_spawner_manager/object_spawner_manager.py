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
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("object_topics_publisher")

        #ros2 service call /object_manager/destroy_object spawn_object_interfaces/srv/DestroyObject "{obj_name: my_tessst_object}"
        #ros2 service call /object_manager/spawn_object spawn_object_interfaces/srv/SpawnObject "{obj_name: my_tessst_object, parent_frame: world, translation:[1.0,1.0,3.0], rotation:[1.0,2.0,3.0,4.0]}"
        # Callbacks run simutaniously
        self.callback_group = ReentrantCallbackGroup()

        # Callbacks can not run simutaniously
        #self.callback_group = MutuallyExclusiveCallbackGroup()

        self.object_topic_publisher_srv_spawn = self.create_service(SpawnObject,'object_manager/spawn_object',self.spawn_object_callback,callback_group=self.callback_group)
        self.object_topic_publisher_srv_destroy = self.create_service(DestroyObject,'object_manager/destroy_object',self.destroy_object_callback,callback_group=self.callback_group)

        self.object_topic_publisher_client_spawn = self.create_client(SpawnObject,'object_publisher/spawn_object') 
        self.object_topic_publisher_client_destroy = self.create_client(DestroyObject,'object_publisher/destroy_object') 

        self.moveit_object_spawner_client = self.create_client(SpawnObject,'moveit_object_handler/spawn_object') 
        self.moveit_object_destroyer_client = self.create_client(DestroyObject,'moveit_object_handler/destroy_object')    

        self.logger = self.get_logger()

        self.logger.info("Object spawner manager started!")



    def destroy_object_callback(self, request: DestroyObject.Request, response: DestroyObject.Response):
        self.logger.info('Destroy Object Service received')
        moveit_destroy_executed = False
        object_destroy_executed =  False
        request_forwarding = DestroyObject.Request()
        request_forwarding.obj_name         = request.obj_name 

        def done_callback(future):
            nonlocal event
            event.set()

        # Destroying part in topic publisher
        event = Event()

        if not self.object_topic_publisher_client_destroy.wait_for_service(timeout_sec=2.0):
            self.logger.info('Spawn Service not available')
            object_destroy_executed =  False

        future = self.object_topic_publisher_client_destroy.call_async(request_forwarding)
        future.add_done_callback(done_callback)
        event.wait()
        object_destroy_executed = future.result().success 

        response.success=True
        # # Destroy part in moveit
        # event = Event()

        # if not self.moveit_object_destroyer_client.wait_for_service(timeout_sec=2.0):
        #     self.logger.info('Spawn Service not available')
        #     moveit_destroy_executed =  False

        # future = self.moveit_object_destroyer_client.call_async(request_forwarding)
        # future.add_done_callback(done_callback)
        # event.wait()
        # moveit_destroy_executed = future.result().success 

        # response.success = object_destroy_executed and moveit_destroy_executed

        return response


    def spawn_object_callback(self, request :SpawnObject.Request, response :SpawnObject.Response):
        self.logger.info('Spawn Object Service received!')
        object_publish_executed =  False
        moveit_spawner_executed =  False
        request_forwarding = SpawnObject.Request()
        request_forwarding.obj_name         = request.obj_name 
        request_forwarding.parent_frame     = request.parent_frame 
        request_forwarding.translation      = request.translation 
        request_forwarding.rotation         = request.rotation
        request_forwarding.cad_data         = request.cad_data

        def done_callback(future):
            nonlocal event
            event.set()

        # Spawning part in topic publisher
        event = Event()

        if not self.object_topic_publisher_client_spawn.wait_for_service(timeout_sec=2.0):
            self.logger.info('Spawn Service not available')
            object_publish_executed =  False
        self.logger.info('Test1')
        future = self.object_topic_publisher_client_spawn.call_async(request_forwarding)
        self.logger.info('Test2')
        future.add_done_callback(done_callback)
        self.logger.info('Test3')
        event.wait()
        self.logger.info('Test4')

        object_publish_executed = future.result().success 

        self.logger.info(object_publish_executed)
        response.success = True
        # # spawning part in moveit
        # if object_publish_executed:
        #     event = Event()
        #     if not self.moveit_object_spawner_client.wait_for_service(timeout_sec=2.0):
        #         self.logger.info('Spawn Service not available')
        #         moveit_spawner_executed =  False

        #     future = self.moveit_object_spawner_client.call_async(request_forwarding)
        #     future.add_done_callback(done_callback)
        #     event.wait()
        #     moveit_spawner_executed = future.result().success 

        # response.success = object_publish_executed and moveit_spawner_executed

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()