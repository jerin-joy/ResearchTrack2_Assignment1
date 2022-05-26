import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='rt2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package = 'rt2_assignment1',
                    plugin = 'rt2_assignment1'+'::RandomPosition',
                    name='random_position_node'),
                ComposableNode(
                    package = 'rt2_assignment1',
                    plugin = 'rt2_assignment1'+'::StateMachine',
                    name='state_machine_node')
                
            ],
            output='screen',
        )
        
    return launch.LaunchDescription([container])


