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
    
  # stream client node
  # stream_client_parameter = PathJoinSubstitution([
      # package,
      # 'parameter',
      # 'stream_client.yaml'
  # ])
    
  stream_client_node = Node(
    package='edu_camera',
    executable='stream_client_node',
    name='stream_client_node',
    namespace=edu_robot_namespace,
    remappings=[
    ('image', 'camera/image'),
    ('distance', 'camera/set_focus_distance')
    ],
    output='screen',
    # prefix=['gdbserver localhost:3000'],
    # parameters=[
    #   stream_client_parameter
    # ]
  )
    
  return LaunchDescription([
      edu_robot_namespace_arg,
      stream_client_node
  ])
