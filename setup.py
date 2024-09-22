from setuptools import setup

package_name = 'wd_task_1a'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Task 1A for Warehouse Drone',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_1a_2392 = wd_task_1a.task_1a_2392:main',  # Main entry point for your script
        ],
    },
)
