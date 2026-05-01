import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'position_topic'

# 递归收集 models 目录，保留子目录结构
model_data_files = []
for root, dirs, files in os.walk('models'):
    if files:
        dest = os.path.join('share', package_name, root)
        sources = [os.path.join(root, f) for f in files]
        model_data_files.append((dest, sources))

# 递归收集 worlds 目录
world_data_files = []
for root, dirs, files in os.walk('worlds'):
    if files:
        dest = os.path.join('share', package_name, root)
        sources = [os.path.join(root, f) for f in files]
        world_data_files.append((dest, sources))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + model_data_files + world_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljq',
    maintainer_email='2453124@tongji.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_publisher = position_topic.position_publisher:main',
            'position_subscriber = position_topic.position_subscriber:main',
        ],
    },
)
