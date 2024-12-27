from setuptools import setup

package_name = 'my_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Node to broadcast TF from odometry',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = my_tf_broadcaster.odom_to_tf:main',
        ],
    },
)

