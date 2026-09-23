import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# World name -> (Gazebo world, URDF of the same obstacles for Foxglove to draw)
WORLDS = {
    'default': ('robot_env.sdf', 'env.urdf'),
    'warehouse': ('warehouse.sdf', 'warehouse_env.urdf'),
    'watonomous': ('watonomous.sdf', 'watonomous_env.urdf'),
}


def launch_setup(context):
    launch_dir = os.path.join(get_package_share_directory('gazebo'), 'launch')
    world = LaunchConfiguration('world').perform(context)
    if world not in WORLDS:
        raise RuntimeError(f"Unknown world '{world}', expected one of: {', '.join(WORLDS)}")
    sdf_file, urdf_file = WORLDS[world]

    gazebo_sim_ign = os.path.join(launch_dir, 'sim.ign')
    sdf_file_path = os.path.join(launch_dir, sdf_file)
    with open(os.path.join(launch_dir, urdf_file)) as f:
        env_urdf = f.read()

    gz_sim = ExecuteProcess(cmd=['ign', 'launch', '-v 4', f'{gazebo_sim_ign}'])
    gz_sim_server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-v 4', '-r', f'{sdf_file_path}'])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/model/robot/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                #    '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[
            ('/model/robot/pose', '/tf'),
            ('/model/robot/pose_static', '/tf')
        ]
    )

    # Publish the world's obstacles (as URDF) on /env_description, latched, so
    # Foxglove draws whichever world is actually running. Foxglove works out
    # where each obstacle goes from the URDF itself, so this publisher's TF
    # output is sent to a topic nobody listens to.
    env_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='env_description_publisher',
        parameters=[{'robot_description': env_urdf}],
        remappings=[
            ('robot_description', 'env_description'),
            ('/tf', '/env_description/tf'),
            ('/tf_static', '/env_description/tf_static'),
        ]
    )

    return [gz_sim, gz_sim_server, bridge, env_description]


def generate_launch_description():
    return LaunchDescription([
        # Pick with `ros2 launch gazebo sim.launch.py world:=warehouse`, or with
        # WORLD in watod-config.sh (passed into the container by docker compose)
        DeclareLaunchArgument(
            'world',
            default_value=os.environ.get('WORLD', 'default'),
            description='World to simulate: ' + ', '.join(WORLDS)),
        OpaqueFunction(function=launch_setup),
    ])
