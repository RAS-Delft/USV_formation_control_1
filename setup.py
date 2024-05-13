from setuptools import setup
import os
from glob import glob

package_name = 'usv_formation_control_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # import marker file and package.xml
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # import all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bart',
    maintainer_email='bartboogmans@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vessel_reference_generator = '+package_name+'.vessel_reference_generator:main',
            'formation_trajectory_planner = '+package_name+'.formation_trajectory_planner:main',
            'formation_configuration_broadcaster_1 = '+package_name+'.formation_configuration_broadcaster_1:main',
            
        ],
    },
)
