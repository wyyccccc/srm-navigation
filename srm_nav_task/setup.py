from setuptools import find_packages, setup

package_name = 'srm_nav_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yycccc',
    maintainer_email='1690493093@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    	'console_scripts': [
        	'send_goal = srm_nav_task.send_goal:main',
        	'follow_waypoints = srm_nav_task.follow_waypoints:main',
                'serial_bridge = srm_nav_task.serial_bridge:main',
                'figure8_navigation = srm_nav_task.figure8_navigation:main',
    	],
},
)
