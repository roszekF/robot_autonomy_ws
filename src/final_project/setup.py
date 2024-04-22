from setuptools import find_packages, setup

package_name = 'final_project'

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
    maintainer='TODO: maintainer name',
    maintainer_email='TODO: maintainer email',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'bt_publisher = final_project.bt_publisher:main',	 
            'map_publisher = final_project.map_publisher:main',
        ],
    },
)
