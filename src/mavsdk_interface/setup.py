from setuptools import find_packages, setup

package_name = 'mavsdk_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','mavsdk'],
    zip_safe=True,
    maintainer='jazz',
    maintainer_email='34100523+janithcyapa@users.noreply.github.com',
    description='onnect to a PX4 Drone using mavsdk and provide a interface between mavsdk and ros',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'mavsdk_node = mavsdk_interface.mavsdk_node:main',
        ],
    },
)
