from setuptools import setup
import glob

package_name = 'go2_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='lukas',
    author_email='you@example.com',
    description='GO2 SLAM launch + nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
    data_files=[
	    ('share/ament_index/resource_index/packages',
	      ['resource/' + package_name]),
	    # install launch files
	    ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
	    # install config files
	    ('share/' + package_name + '/config', 
	    glob.glob('config/*.yaml') +
	    glob.glob('config/*.rviz') +
      glob.glob('config/*.rviz2')),
    ],
)

