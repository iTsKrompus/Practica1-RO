from setuptools import find_packages, setup

package_name = 'super_lane_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), 
            ('share/super_lane_pkg/launch', ['launch/super_lane.launch.py']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/CitroenCZero.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/city_traffic.wbt']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'lane_detector = super_lane_pkg.lane_detector:main',
        'lane_controller = super_lane_pkg.lane_controller:main',
    ],
    
},



)
