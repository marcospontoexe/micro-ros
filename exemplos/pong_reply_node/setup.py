from setuptools import find_packages, setup

package_name = 'pong_reply_node'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Simple ROS 2 node that replies to ping messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pong = pong_reply_node.pong:main',
        ],
    },
)