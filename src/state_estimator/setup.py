from setuptools import setup
import os
from glob import glob

package_name = 'state_estimator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='State estimator for obstacle prediction using Kalman Filter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kf_state_estimator = state_estimator.kf_state_estimator:main',
            'ekf_state_estimator = state_estimator.ekf_state_estimator:main',
            'ukf_state_estimator = state_estimator.ukf_state_estimator:main',
            'costmap_merger = state_estimator.costmap_merger:main',
        ],
    },
)