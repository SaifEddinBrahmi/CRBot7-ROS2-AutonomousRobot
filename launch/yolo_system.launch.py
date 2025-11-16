#!/usr/bin/env python3
"""
Launch file for CRBot7 with YOLO object detection
Launches complete system + YOLO detector
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='arena.sdf',
        description='World file to load (arena.sdf, empty.sdf, bookstore.sdf)'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='YOLO model to use (yolov8n.pt, yolov8s.pt, yolov8m.pt, etc.)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for detections (0.0-1.0)'
    )
    
    # Get launch arguments
    world_file = LaunchConfiguration('world_file')
    yolo_model = LaunchConfiguration('yolo_model')
    confidence = LaunchConfiguration('confidence_threshold')
    
    # Include complete system launch
    complete_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crbot7'),
                'launch',
                'complete_system.launch.py'
            ])
        ]),
        launch_arguments={
            'world_file': world_file
        }.items()
    )
    
    # YOLO detector node
    yolo_detector = Node(
        package='crbot7',
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_name': yolo_model,
            'confidence_threshold': confidence,
            'camera_topic': '/robot_cam',
            'publish_annotated': True,
            'device': 'cpu',  # Change to 'cuda' if you have GPU
            'use_sim_time': True
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        world_file_arg,
        yolo_model_arg,
        confidence_arg,
        complete_system_launch,
        yolo_detector
    ])
