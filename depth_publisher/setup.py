from setuptools import setup

package_name = 'depth_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools==58.2.0', 'depthai'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for publishing depth data using DepthAI',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_publisher = depth_publisher.depth_publisher:main',
        ],
    },
)
