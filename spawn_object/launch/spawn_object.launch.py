import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution, PythonExpression,EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa

def generate_launch_description():

    # Define Launch Description
    ld = LaunchDescription()
        
    parent_frame = 'world'
    obj_frame_name = 'exampleObject'
    obj_namespace = f'objects/{obj_frame_name}/'
    translation = [0, 0, 2.0]
    rotation = [0.0, 0.0, 0.0, 1.0]
    STL_path = '//home/mll/Desktop/Gonio_Right_Stage_2_Upper.STL'

    parent_frame_arg = DeclareLaunchArgument(
        name = 'obj_reference_frame',
        default_value = parent_frame,
        description = 'dd')
    ld.add_action(parent_frame_arg) # This allows argument proposal in terminal with --show-args

    obj_frame_name_arg = DeclareLaunchArgument(
        name = 'obj_frame_name',
        default_value = obj_frame_name,
        description ='dd')
    ld.add_action(obj_frame_name_arg)

    translation_arg = DeclareLaunchArgument(
        name = 'obj_translation',
        default_value = str(translation),
        description ='dd')
    ld.add_action(translation_arg)

    rotation_arg = DeclareLaunchArgument(
        name = 'obj_rotation',
        default_value = str(rotation),
        description ='dd')
    ld.add_action(rotation_arg)

    STL_path_arg = DeclareLaunchArgument(
        name = 'STL_path',
        default_value = STL_path,
        description ='dd')
    ld.add_action(STL_path_arg)


    point_name_arg = DeclareLaunchArgument(
        name = 'point_frame_name',
        default_value = '',
        description ='dd')
    ld.add_action(point_name_arg)

    point_translation_arg = DeclareLaunchArgument(
        name = 'point_translation',
        default_value = '',
        description ='dd')
    ld.add_action(point_translation_arg)

    point_rotation_arg = DeclareLaunchArgument(
        name = 'point_point_rotation',
        default_value = '',
        description ='dd')
    ld.add_action(point_rotation_arg)

    point_frame_names=[]
    point_translations=[]
    point_rotations=[]

    for arg in sys.argv:
        if arg.startswith("obj_reference_frame:="):
            parent_frame = arg.split(":=")[1]
        if arg.startswith("obj_frame_name:="):
            obj_frame_name = arg.split(":=")[1]
        if arg.startswith("obj_translation:="):
            translation = eval(arg.split(":=")[1])
        if arg.startswith("obj_rotation:="):
            rotation = eval(arg.split(":=")[1])
        if arg.startswith("STL_path:="):
            STL_path = arg.split(":=")[1]

        if arg.startswith('point_frame_name:='):
            point_frame_names.append(arg.split(":=")[1])

        if arg.startswith('point_translation:='):
            point_translations.append(eval(arg.split(":=")[1]))

        if arg.startswith('point_point_rotation:='):
            point_rotations.append(eval(arg.split(":=")[1]))
    
    topic_publisher = Node(
        name="object_spawner_id",
        package="spawn_object",
        executable="spawn_object_tf.py",
        namespace = obj_namespace,
        parameters=[
            {'parent_frame_name': parent_frame},
            {'child_frame_name': obj_frame_name},
            {'translation': translation},
            {'rotation': rotation},
        ]
    )


    if len(point_frame_names) == len(point_translations) == len(point_rotations) and len(point_frame_names) >0:
        for pnt_name, pnt_translation, pnt_rotation in zip (point_frame_names,point_translations,point_rotations):
            point_publisher = Node(
                name=pnt_name+'_publisher',
                package="spawn_object",
                executable="spawn_object_tf.py",
                namespace = obj_namespace+ pnt_name,
                parameters=[
                    {'parent_frame_name': obj_frame_name},
                    {'child_frame_name': pnt_name},
                    {'translation': pnt_translation},
                    {'rotation': pnt_rotation},
                ]
            )
            ld.add_action(point_publisher)



    moveit_object_publisher = Node(
        name="object_spawner_moveit",
        package="spawn_object",
        executable="spawn_object_in_moveit",
        parameters=[
            {'obj_frame_name': obj_frame_name},
            {'obj_namespace': obj_namespace},
            {'STL_path': STL_path},
        ]
    )

    ld.add_action(topic_publisher)
    ld.add_action(moveit_object_publisher)
    return ld

if __name__ == '__main__':
    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.

    sys.argv.append('point_frame_name:=test_point')
    sys.argv.append('point_translation:=[1.0,2.0,3.0]')
    sys.argv.append('point_point_rotation:=[0.0,0.0,0.0,1.0]')

    sys.argv.append('point_frame_name:=test_point2')
    sys.argv.append('point_translation:=[2.0,2.0,3.0]')
    sys.argv.append('point_point_rotation:=[0.0,0.0,0.0,1.0]')

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    print("this is a test")
    sys.exit(ls.run())
    