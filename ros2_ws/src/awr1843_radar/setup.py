from glob import glob

from setuptools import setup

package_name = 'awr1843_radar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wenqian Zhang',
    maintainer_email='212663095+WenqianZhang001@users.noreply.github.com',
    description='ROS 2 driver for the TI AWR1843BOOST radar (mmWave SDK 3.x demo firmware)',
    license='TODO',
    entry_points={
        'console_scripts': ['awr1843_driver = awr1843_radar.driver_node:main'],
    },
)
