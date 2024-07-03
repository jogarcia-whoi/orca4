from setuptools import find_packages, setup

package_name = 'orca_extend'

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
    maintainer='orca4',
    maintainer_email='orca4@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest', 'pytest-mock'],
    test_suite='pytest',
    entry_points={
        'console_scripts': [
            'example_node = orca_extend.example_node:main'
        ],
    },
)
