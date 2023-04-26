from setuptools import setup

package_name = 'formation_control'

data_files=[]
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/test_launch.py']))
data_files.append(('share/' + package_name + '/resource', ['resource/mavic_webots.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/ground_robot.urdf']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/test_world.wbt']))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nevil',
    maintainer_email='nevil.sandaradura@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavic_driver = formation_control.mavic_driver:main',
            'formation_controller = formation_control.formation_controller:main',
            'ground_robot_driver = formation_control.ground_robot_driver:main'
        ],
    },
)
