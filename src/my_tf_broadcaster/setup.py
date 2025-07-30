from setuptools import find_packages, setup

package_name = 'my_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Dodaj ten wpis, aby instalować pliki launch
        ('share/' + package_name + '/launch', ['launch/my_tf_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='r4h',
    maintainer_email='r4h@todo.todo',
    description='Static TF broadcaster for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = my_tf_broadcaster.odom_to_tf:main',
        ],
    },
)