from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_control'

setup(
    name=package_name,
    version='0.0.1',
    # grab go2_control _and_ any sub-packages
    packages=find_packages(include=['go2_control', 'go2_control.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukas',
    maintainer_email='lukas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'walk_forward = go2_control.walk_forward:main',
            'walk_forward_bt = go2_control.walk_forward_bt:main',
            'walk_forward_bt_with_permission = go2_control.walk_forward_bt_with_permission:main',
            'walk_forward_bt_parallel = go2_control.walkBT:main',
            'scan2d = go2_control.scan2d:main',
        ],
    },
)

