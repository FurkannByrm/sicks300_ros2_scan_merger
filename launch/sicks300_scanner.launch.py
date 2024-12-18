#!/usr/bin/env python3

'''
    Launches a Sick S300 laser scanner node and a filter node.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import launch.events
import lifecycle_msgs.msg

def generate_launch_description():

    # Default filenames and where to find them
    sicks300_dir = get_package_share_directory('sicks300_ros2_scan_merger')

    # Read the YAML parameters file.
    default_sicks300_param_file = os.path.join(sicks300_dir, 'params', 'default.yaml')

    # Create the launch configuration variables.
    sicks300_param_file = LaunchConfiguration('sicks300_param_file', default=default_sicks300_param_file)

    # Map these variables to arguments: can be set from the command line or a default will be used
    sicks300_param_file_launch_arg = DeclareLaunchArgument(
        'sicks300_param_file',
        default_value=default_sicks300_param_file,
        description='Full path to the Sicks300 parameter file to use'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Prepare the sicks300_2 node.
    sicks300_node_0 = LifecycleNode(
        package = 'sicks300_ros2_scan_merger',
        namespace = '',
        executable = 'sicks300_ros2_scan_merger',
        name = 'laser_front',
        parameters = [sicks300_param_file],
        emulate_tty = True,
        output='screen', 
        arguments=[
            '--ros-args', 
            '--log-level', ['laser_front:=', LaunchConfiguration('log-level')]]
    )
    # When the sick node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_sick_reaches_inactive_state_0 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = sicks300_node_0,
            goal_state = 'inactive',
            entities = [
                EmitEvent(event = ChangeState(
                    lifecycle_node_matcher = launch.events.matches_action(sicks300_node_0),
                    transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    # Make the sicks300_2 node take the 'configure' transition.
    emit_event_to_request_that_sick_does_configure_transition = EmitEvent(
            event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(sicks300_node_0),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    # Prepare the sicks300_2 node.
    sicks300_node_1 = LifecycleNode(
        package = 'sicks300_ros2_scan_merger',
        namespace = '',
        executable = 'sicks300_ros2_scan_merger',
        name = 'laser_rear',
        parameters = [sicks300_param_file],
        emulate_tty = True,
        output='screen', 
        arguments=[
            '--ros-args', 
            '--log-level', ['laser_rear:=', LaunchConfiguration('log-level')]]
    )
    



    # # When the sick node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_sick_reaches_inactive_state_1 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = sicks300_node_1,
            goal_state = 'inactive',
            entities = [
                EmitEvent(event = ChangeState(
                    lifecycle_node_matcher = launch.events.matches_action(sicks300_node_1),
                    transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    #Make the sicks300_2 node take the 'configure' transition.
    emit_event_to_request_that_sick_does_configure_transition_2 = EmitEvent(
            event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(sicks300_node_1),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        sicks300_param_file_launch_arg,
        declare_log_level_arg,
        register_event_handler_for_sick_reaches_inactive_state_0,
        register_event_handler_for_sick_reaches_inactive_state_1,
        sicks300_node_0,
        sicks300_node_1,
        emit_event_to_request_that_sick_does_configure_transition,
        emit_event_to_request_that_sick_does_configure_transition_2,
    ])