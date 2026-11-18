from setuptools import setup
from glob import glob

package_name = 'b31ys_kalman'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.rviz')
        ),
        (
            # saved SLAM maps (.yaml + .pgm)
            'share/' + package_name + '/maps',
            glob('maps/*.yaml') + glob('maps/*.pgm')
        ),
        (
            # lets you run: ros2 run b31ys_kalman build_map.sh
            'lib/' + package_name,
            ['scripts/build_map.sh']
        ),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='B31YS Teaching Team',
    maintainer_email='instructor@example.com',
    description='B31YS Kalman Filter Assignment ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_sensors = b31ys_kalman.fake_sensors:main',
            'student_trajectory = b31ys_kalman.student_trajectory:main',
            'kalman_filter_template = '
            'b31ys_kalman.kalman_filter_template:main',
            'prm_planner = b31ys_kalman.prm_planner:main',
            'path_follower = b31ys_kalman.path_follower:main',
        ],
    },
)
