from setuptools import find_packages, setup

package_name = 'lane_segmentation_in_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='biswash',
    maintainer_email='077bme014.biswash@pcampus.edu.np',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_segmenation_node_normal = lane_segmentation_in_ros2.lane_Segmentation:main',
            'lane_segmenation_node_venv = lane_segmentation_in_ros2.lane_Segmentation_node:main',
            'camera_publisher = lane_segmentation_in_ros2.camerapublisher:main',        
        ],
    },
)
