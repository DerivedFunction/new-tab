from setuptools import find_packages, setup

package_name = 'depth_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='del226@lehigh.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_subscriber = depth_sub.depth_subscriber:main'
        ],
    },
)
