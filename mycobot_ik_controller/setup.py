from setuptools import find_packages, setup

package_name = 'mycobot_ik_controller'

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
    maintainer='carrobo2024',
    maintainer_email='yoshimura.wataru281@mail.kyutech.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_end_effector_goal = mycobot_ik_controller.send_end_effector_goal:main',
            'mycobot_action_server = mycobot_ik_controller.mycobot_action_server:main',
        ],
    },
)
