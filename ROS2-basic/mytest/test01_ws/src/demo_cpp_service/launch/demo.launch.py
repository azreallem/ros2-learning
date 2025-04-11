import launch
import launch_ros

def generate_launch_description():
    
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument('launch_arg_bg', default_value="150")

    action_node_turtlesim_node = launch_ros.actions.Node(package='turtlesim', executable='turtlesim_node', 
        parameters=[{'background_g': launch.substitutions.LaunchConfiguration('launch_arg_bg', default="150")}],
    output='screen')
 
    action_node_turtle_client = launch_ros.actions.Node(package='demo_cpp_service', executable='turtle_client', output='log')

    action_node_turtle_service = launch_ros.actions.Node(package='demo_cpp_service', executable='turtle_service', output='both')



    return launch.LaunchDescription([action_declare_arg_background_g, 
                                    action_node_turtlesim_node,
                                    action_node_turtle_client,
                                    action_node_turtle_service])
