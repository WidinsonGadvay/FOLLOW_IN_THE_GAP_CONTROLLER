from setuptools import find_packages, setup

package_name = 'f1tenth_follow_gap'

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
    maintainer='widinsong',
    maintainer_email='widinsong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'FollowTheGapNode=f1tenth_follow_gap.FollowTheGapNode:main',
        'LapCounterNode=f1tenth_follow_gap.LapCounterNode:main',

        ],
    },
)
