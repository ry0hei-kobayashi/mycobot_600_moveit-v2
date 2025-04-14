from setuptools import find_packages, setup

package_name = 'f100_gripper_pkg'

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
            'client_example = f100_gripper_pkg.client_example:main',
            'gripper_server = f100_gripper_pkg.gripper_server:main',
        ],
    },
)
