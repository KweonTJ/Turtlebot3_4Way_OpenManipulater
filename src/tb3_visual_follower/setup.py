from setuptools import find_packages, setup

package_name = 'tb3_visual_follower'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
               'follower = tb3_visual_follower.follower:main',
               'follower_v5 = tb3_visual_follower.follower_v5:main',
               'follower_v5_1 = tb3_visual_follower.follower_v5_1:main',
        ],
    },
)
