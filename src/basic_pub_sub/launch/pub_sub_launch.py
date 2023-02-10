import launch
import launch_ros.actions
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, EnvironmentVariable, LaunchConfiguration

#File have to end launch.py!

def generate_launch_description():
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name= 'my_string',
            default_value='Hello from Launch File! ',
            description='Print String Prefix'
        ),

        launch.actions.GroupAction(
            [
                LogInfo(condition=IfCondition( #Use UnlessCondition for reverse logic
                    PythonExpression(["'",
                    EnvironmentVariable('USER'),
                    "' == 'rosi'" #Enter your username here
                    ])
                ),
                msg=['Hello! ', EnvironmentVariable('USER')]
                ) 
            ]
            ),
        launch.actions.TimerAction(
            period=5.0,
            actions=[
            
            launch_ros.actions.Node(
            package='basic_pub_sub',
            executable='pub_node',
            name='publisher',
            parameters = [{'my_string':  LaunchConfiguration('my_string')}]
            )
            ]
        ),

        launch.actions.TimerAction(
            period=10.0,
            actions=[
            launch_ros.actions.Node(
            package='basic_pub_sub',
            executable='sub_node',
            name='subscriber'),
            ]
        )

        
  ])