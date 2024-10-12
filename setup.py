from setuptools import setup, find_packages

package_name = 'wd_task_1a'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    py_modules=[
        'task_1a_2392',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@domain.com',
    description='ROS2 package for Task 1A',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'task_1a_2392 = task_1a_2392:main',
        ],
    },
    package_data={
        package_name: ['package.xml'],  # Include package.xml
    },
)
