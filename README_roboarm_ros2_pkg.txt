Create the ROS 2 Package

format:
~/roboarm_control_ros2_ws/
└── src/
    └── main/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── main/
            ├── __init__.py
            ├── control_can.py      ← the rewritten node
            └── ARM.urdf

mkdir -p ~/roboarm_control_ros2_ws/src
cd ~/roboarm_control_ros2_ws
source /opt/ros/humble/setup.bash
cd src
ros2 pkg create --build-type ament_python main \
    --dependencies rclpy sensor_msgs

put control.py and ARM.urdf intro the src/main/main and change ARM.urdf path in control.py

package.xml
-----------------------
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>main</name>
  <version>0.1.0</version>
  <description>Robotic arm controller — ODrive CAN bus</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>


setup.py
----------------------------
from setuptools import setup
import os
from glob import glob

package_name = 'main'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install URDF so it is accessible at runtime
        (os.path.join('share', package_name, 'urdf'), glob('main/*.urdf')),
        # Install any launch files if you add them later
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robotic arm controller — ODrive CAN bus',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package.module:function'
            'controller = main.control:main',
        ],
    },
)

setup.cfg
------------------------------------
[develop]
script_dir=$base/lib/main
[install]
install_scripts=$base/lib/main


-------------------------------------------
pip3 install python-can ikpy numpy pyserial
cd ~/roboarm_control_ros2_ws
colcon build
source install/setup.bash
ros2 run main controller
