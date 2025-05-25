from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle_sim',
    )

    turtle1_circle = Node(
        package='software_training',
        executable='turtle1_circle_node',
        name='turtle1_circle',
    )
    
    spawn_turtle = Node(
        package='software_training',
        executable='spawn_turtle_node',
        name='spawn_turtle',
    )
    
    reset_moving_turtle = Node(
        package='software_training',
        executable='moving_turtle_server_node',
        name='moving_turtle_server',
    )
    
    distance_publisher = Node(
        package='software_training',
        executable='distance_publisher_node',
        name='distance_publisher',
    )
    
    moving_turtle_server = Node(
        package='software_training',
        executable='moving_turtle_server_node',
        name='moving_turtle_server',
    )
    

    launch_after_turtlesim = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim,
            on_start=[
                turtle1_circle,
                spawn_turtle,
                reset_moving_turtle,
                distance_publisher,
                moving_turtle_server
            ]
        )
    )

    return LaunchDescription([
        turtlesim,
        launch_after_turtlesim
    ])