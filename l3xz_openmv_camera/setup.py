from setuptools import setup

package_name = 'l3xz_openmv_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    package_dir={"l3xz_openmv_camera.openmv": "l3xz_openmv_camera/openmv/tools/rpc"},
    zip_safe=True,
    maintainer='Jonas Wuehr',
    maintainer_email='jonaswuehrmaintainer@gmail.com',
    description='ROS2 node for openmv camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'l3xz_openmv_camera = l3xz_openmv_camera.l3xz_openmv_camera:main',
        ],
    },
)
