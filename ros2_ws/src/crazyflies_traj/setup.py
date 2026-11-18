from glob import glob

from setuptools import setup

package_name = 'crazyflies_traj'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claus Rosito',
    maintainer_email='clausqr@proton.me',
    description='CSV trajectory player client of the crazyflies-gs plant',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traj_player = crazyflies_traj.traj_player_node:main',
            'traj = crazyflies_traj.cli:main',
        ],
    },
)
