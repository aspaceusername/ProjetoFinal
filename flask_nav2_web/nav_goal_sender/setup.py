from setuptools import find_packages, setup

package_name = 'nav_goal_sender'

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
    maintainer='ruben',
    maintainer_email='110482980+aspaceusername@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_state_listener.py = nav_goal_sender.goal_state_listener:main',
            'send_goal.py = nav_goal_sender.send_goal:main',
        ],
    },
)
