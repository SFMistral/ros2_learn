from setuptools import find_packages, setup

package_name = 'turtle_controller'

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
    maintainer='zzxl',
    maintainer_email='ant14ctual@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # 注册写好的包为可执行程序
    entry_points={
        'console_scripts': [
            'turtle_circle = turtle_controller.turtle_circle:main',
            'turtle_pose_sub = turtle_controller.turtle_pose_sub:main',
            'turtle_follower = turtle_controller.turtle_follower:main',
            'clear_client = turtle_controller.clear_client:main',
            'teleport_client = turtle_controller.teleport_client:main',
            'follow_path_service = turtle_controller.follow_path_service:main',
            'follow_path_client = turtle_controller.follow_path_client:main',
            'rotate_client = turtle_controller.rotate_client:main',
            'move_path_server = turtle_controller.move_path_server:main',
            'move_path_client = turtle_controller.move_path_client:main',
        ],
    },
)
