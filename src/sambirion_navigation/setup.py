from setuptools import setup, find_packages

package_name = 'sambirion_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Automatically find all packages in your source folder
    data_files=[
        ('share/' + package_name + '/launch', [
            'launch/nav2_bringup.launch.py',
            'launch/slam_toolbox.launch.py'
        ]),
        ('share/' + package_name + '/params', [
            'params/nav2_params.yaml'
        ]),
        ('share/' + package_name + '/maps', [
            'maps/map.yaml',
            'maps/map.pgm'
        ]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrii Biliak',
    maintainer_email='andrii@example.com',
    description='Navigation2 launch and config for Sambirion mecanum robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_state_publisher = sambirion_navigation.joint_state_publisher:main',
        ],
    },
)
