from setuptools import find_packages, setup

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kaushik',
    maintainer_email='kaushikraghupathruni@gmail.com',
    description='3D global path planner using A* on an OctoMap.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = global_planner.planner_node:main',
        ],
    },
)
