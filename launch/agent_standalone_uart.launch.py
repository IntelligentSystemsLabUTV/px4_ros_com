from launch import LaunchDescription
from launch_ros.actions import Node

"""Generates a launch description for the microRTPS Agent."""
def generate_launch_description():
  ld = LaunchDescription()
  agent_node = Node(
    package='px4_ros_com',
    executable='micrortps_agent',
    namespace='fmu',
    shell=True,
    emulate_tty=True,
    output='both',
    log_cmd=True,
    arguments=[
      '-n',
      'fmu', # Everything will be under the "/fmu" namespace
      '-t',
      'UART', # UART transport,
      '-d',
      '/dev/ttyS5', # UART device
      '-b',
      '921600', # UART device baudrate
      '-h', # Hardware flow control
      '--' # ROS 2 options will come after this, so getopt must stop here
    ]
  )
  ld.add_action(agent_node)
  return ld
