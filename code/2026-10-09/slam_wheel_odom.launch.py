import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ==========================================
# 라이다 장착 위치 (base_footprint 기준, 직접 재서 수정)
# base_footprint = 두 바퀴 축의 가운데, 바닥 높이
# ==========================================
LASER_X = '0.0'      # 바퀴 축 중심에서 라이다 중심까지 앞(+)/뒤(-) 거리 (m)
LASER_Y = '0.0'      # 왼쪽(+)/오른쪽(-) 거리 (m)
LASER_Z = '0.10'     # 바닥에서 라이다 스캔 면까지 높이 (m)
LASER_YAW = '3.14159'    # 라이다 정면이 차 뒤쪽을 보면 3.14159


def generate_launch_description():
    params_file = os.path.expanduser('~/slam_custom_params.yaml')

    # 1. RPLIDAR C1 노드
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )

    # 2. base_footprint -> laser Static TF (실제 장착 위치)
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[LASER_X, LASER_Y, LASER_Z, LASER_YAW, '0', '0',
                   'base_footprint', 'laser'],
        output='screen'
    )

    # 3. 바퀴 오도메트리 (엔코더 + 자이로) : odom -> base_footprint TF, /odom 발행, /cmd_vel 구독
    #    laser_odom.py 대신 사용
    wheel_odom_proc = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/wheel_odom_node.py')],
        output='screen'
    )

    # 4. SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[params_file],
        output='screen'
    )

    # 5. Rosbridge WebSocket
    rosbridge_launch_file = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file)
    )

    return LaunchDescription([
        rplidar_node,
        tf_laser_node,
        wheel_odom_proc,
        slam_node,
        rosbridge_launch
    ])
