import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='hadabot_driver', executable='hadabot_controller', output='screen',
            name=['hadabot_controller']),
        launch_ros.actions.Node(
            package='hadabot_tf2', executable='hadabot_tf2_broadcaster', output='screen',
            name=['hadabot_tf2_broadcaster']),			
    ])