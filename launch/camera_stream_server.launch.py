import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # robot namespace
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
      'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
  )
    
  # general
  package = FindPackageShare('edu_camera')

  # stream server node
  stream_server_parameter = PathJoinSubstitution([
      package,
      'parameter',
      'stream_server.yaml'
  ])

  stream_server_node = Node(
    package='edu_camera',
    executable='camera_node',
    name='stream_server_node',
    namespace=edu_robot_namespace,
    parameters=[
      stream_server_parameter
    ],
    remappings=[
      ('subscribe_to_stream', 'stream_server/subscribe_to_stream'),
      ('unsubscribe_from_stream', 'stream_server/unsubscribe_from_stream')
    ],
    output='screen',
    # prefix=['gdbserver localhost:3000'],
  )

  return LaunchDescription([
      edu_robot_namespace_arg,
      stream_server_node
  ])
