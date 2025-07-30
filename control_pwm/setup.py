from setuptools import find_packages, setup

package_name = 'control_pwm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','pyserial'],
    zip_safe=True,
    maintainer='nextauv',
    maintainer_email='nextauv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control_pwm.control_node:main',
	    'esp32_node = control_pwm.esp32_node:main'
        ],
    },
)
