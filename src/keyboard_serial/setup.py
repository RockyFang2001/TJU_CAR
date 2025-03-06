from setuptools import setup

package_name = 'keyboard_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Keyboard to serial control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_serial_node = keyboard_serial.keyboard_serial_node:main',
        ],
    },
    install_requires=[
    'setuptools',
    'pynput',  # 明确指定最低版本
    'pyserial',
    ],
)
