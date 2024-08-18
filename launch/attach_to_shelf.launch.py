import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            # executable='component_container',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='components_attach_shelf',
                    plugin='my_components::PreApproach',
                    name='pre_approach',
                    parameters=[{'use_sim_time': True}]),
                ComposableNode(
                    package='components_attach_shelf',
                    plugin='my_components::AttachServer',
                    name='attach_server',                    
                    parameters=[{'use_sim_time': True}]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])