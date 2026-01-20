from setuptools import find_packages, setup

package_name = 'dynamic_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py', 'launch/ia_dwa.launch.py', 'launch/navigation.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/dynamic_world.sdf']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml', 'config/teb_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nebiyu, Yonas',
    maintainer_email='nebadan7@gmail.com',
    description='TODO: Package descriptiogit remote add origin git@github.com:nebadan/dynamic_planner.gitn',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ia_dwa_planner = dynamic_planner.ia_dwa_planner:main',
        ],
    },
)
