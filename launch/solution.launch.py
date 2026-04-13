import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    
    node = Node(
    	package= 'mpc_rbt_student',
    	executable='Localization',
    	name='LocalizationNode',
    	output='screen',
    	parameters=[{'use_sim_time' : True}] #usem_sim_time
    	)
    
    rviz = Node(
    	package='rviz2',
    	executable='rviz2',
    	name='rviz2',
    	arguments=['-d', rviz_config_path],
    	output='screen',
    	parameters=[{'use_sim_time' : True}]
    	)
    
    keyboard = Node(
    	package='mpc_rbt_student',
    	executable='keyboard_control',
    	name='KeyboardNode',
    	output='screen',
    	parameters=[{'use_sim_time' : True}]
    	)
    
    planning = Node(
    	package='mpc_rbt_student',
    	executable='Planning',
    	name='PlanningNode',
    	output='screen',
    	parameters=[{'use_sim_time' : True}]
    	)
    
    motion = Node(
    	package='mpc_rbt_student',
    	executable='MotionControl',
    	name='MotionControlNode',
    	output='screen',
    	parameters=[{'use_sim_time' : True}]
    	)

    
    return LaunchDescription([
	#node, rviz, keyboard, planning
	#planning
	node, rviz, planning, motion
    ])
