from setuptools import setup

package_name = 'mirobot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Mirobot Python ROS 2 Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mirobot_node = mirobot_package.mirobot_node:main',
        ],
    },
)
