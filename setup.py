from setuptools import find_packages, setup

package_name = 'serassignment1'

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
    maintainer='deepika',
    maintainer_email='kandlapallideepika6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'smach_node = serassignment1.smach_node:main',
            'bt_node = serassignment1.bt_node:main',
        ],
    },
)
