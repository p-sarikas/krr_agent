import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess,
    RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    manager_script = LaunchConfiguration('manager_script')

    manager_script_arg = DeclareLaunchArgument(
        'manager_script',
        default_value='task2_multiple_pddl_manager.py',
        description='Which manager script to use: task2_multiple_pddl_manager.py or task2_manager.py'
    )

    is_multi_pddl_str = PythonExpression([
        "'true' if 'multiple' in '", manager_script, "' else 'false'"
    ])

    krr_project_path = get_package_share_directory('krr_agent')
    plansys_path = get_package_share_directory('plansys2_bringup')

    plansys2_params_file = os.path.join(krr_project_path, 'config', 'plansys2_params.yaml')
    setup_db_script = os.path.join(krr_project_path, 'scripts', 'setup_database.py')

    # --- PLANSYS2 ---
    plansys2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            plansys_path, 'launch', 'plansys2_bringup_launch_distributed.py')),
        launch_arguments={
            'model_file':   krr_project_path + '/pddl/domain_t2.pddl',
            'problem_file': krr_project_path + '/pddl/problem_t2_template.pddl',
            'params_file':  plansys2_params_file,
        }.items()
    )

    # --- MIRTE SKILLS ---
    mirte_skills_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krr_mirte_skills'), 'launch', 'mirte_skills.launch.xml'
            ])
        ])
    )

    # --- DELAYED PLANSYS2 BRINGUP ---
    # Give Gazebo and Nav2 12 seconds to settle down before hitting the CPU with PlanSys2
    delayed_plansys2 = TimerAction(
        period=12.0, 
        actions=[plansys2_bringup]
    )

    # --- STEP 1: Init DB (loads schema + static data) ---
    init_typedb_process = ExecuteProcess(
        cmd=['python3', setup_db_script],
        additional_env={'USE_MULTI_PDDL': is_multi_pddl_str},
        output='screen'
    )

    # --- STEP 2: ROS_TYPEDB bridge node ---
    ros_typedb_node = Node(
        package='ros_typedb',
        executable='ros_typedb',
        name='ros_typedb',
        output='screen',
        parameters=[{
            'database_name': 'task_db',
            'address': 'localhost:1729',
            'force_database': False,
            'force_data': False,
            'infer': True,
        }]
    )

    # --- STEP 3: Initial Pose ---
    initial_pose_node = Node(
        package='krr_agent',
        executable='set_initial_pose',     
        name='initial_pose_publisher',     
        output='screen'
    )

    # --- STEP 4: Lifecycle management ---
    configure_typedb = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ros_typedb', 'configure'],
        output='screen'
    )

    activate_typedb = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/ros_typedb', 'activate'],
        output='screen'
    )

    # --- EVENT CHAIN ---

    # 1. DB script finishes → start ros_typedb node
    on_db_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=init_typedb_process,
            on_exit=[ros_typedb_node]
        )
    )

    # 2. ros_typedb process starts → wait 2s → configure
    on_typedb_start = RegisterEventHandler(
        OnProcessStart(
            target_action=ros_typedb_node,
            on_start=[
                TimerAction(period=2.0, actions=[configure_typedb])
            ]
        )
    )

    # 3. configure exits → activate
    on_configured = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_typedb,
            on_exit=[activate_typedb]
        )
    )

    # 4. activate exits → start all nodes that need TypeDB ready (Wait 10 seconds for PlanSys2)
    on_activated = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_typedb,
            on_exit=[
                TimerAction(period=10.0, actions=[
                    Node(
                        package='krr_agent',
                        executable=manager_script,
                        name='task_manager_node',
                        output='screen'
                    ),
                    Node(
                        package='krr_agent',
                        executable='task_controller',
                        output='screen',
                        parameters=[plansys2_params_file]
                    ),
                    Node(
                        package='krr_agent',
                        executable='action_move_to_object',
                        parameters=[{'action_name': 'move_to_object'}]
                    ),
                    Node(
                        package='krr_agent',
                        executable='action_move_to_drop_location_t2',
                        output='screen',
                        parameters=[{'action_name': 'move_to_drop_location'}]
                    ),
                    Node(
                        package='krr_agent',
                        executable='action_pick',
                        parameters=[{'action_name': 'pick'}]
                    ),
                    Node(
                        package='krr_agent',
                        executable='action_place',
                        parameters=[{'action_name': 'place'}]
                    ),
                    Node(
                        package='krr_agent',
                        executable='action_next_room',
                        parameters=[{'action_name': 'next_room'}]
                    ),
                ])
            ]
        )
    )

    return LaunchDescription([
        initial_pose_node,
        mirte_skills_launch,
        delayed_plansys2, 
        init_typedb_process,
        on_db_ready,
        on_typedb_start,
        on_configured,
        on_activated,
    ])