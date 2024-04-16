from setuptools import setup

package_name = 'final_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='runlongupenn',
    maintainer_email='runlong@seas.upenn.edu',
    description='f1tenth final_pkg',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = final_pkg.planner_node:main',
        ],
    },
)
