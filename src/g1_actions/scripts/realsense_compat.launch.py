"""Map the older RealSense driver's relative topics to the business interface."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    original=Path(get_package_share_directory('realsense2_camera'))/'launch'/'rs_launch.py'
    topics=('color/image_raw','color/camera_info','color/metadata',
            'depth/image_rect_raw','depth/camera_info','depth/metadata',
            'aligned_depth_to_color/image_raw','aligned_depth_to_color/camera_info',
            'extrinsics/depth_to_color','extrinsics/depth_to_depth')
    return LaunchDescription([GroupAction([
        *(SetRemap(src=topic,dst='camera/'+topic) for topic in topics),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(original))),
    ])])
