from setuptools import find_packages, setup

package_name = 'hand_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilia',
    maintainer_email='user@todo.todo',
    description='Follow person raising hand for 3 seconds',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower_node = hand_follower.follower_node:main',
        ],
    },
)
