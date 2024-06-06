from setuptools import setup
from setuptools import find_packages

package_name = 'map_frame'

setup(
    name='example_package',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['map_frame/srv/CostMap.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map = map_frame.cost_map:main'
        ],
    },
)
