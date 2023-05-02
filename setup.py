import os
from glob import glob
from setuptools import setup

package_name = 'rov_app'

components = "rov_app/components"
nodes = "rov_app/nodes"
utils = "rov_app/utils"

setup(
    name=package_name,
    version='0.0.0',
    packages=[ package_name, nodes, components, utils ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ManOAR',
    maintainer_email='contact@manoar.com',
    description='base nodes for the rover application',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat = rov_app.nodes.__heartbeats:main',
            'videostream = rov_app.nodes.__videostream:main',
            'movement = rov_app.nodes.__movement:main'
        ],
    }
)
