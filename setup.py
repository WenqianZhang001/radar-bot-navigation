from glob import glob
from setuptools import setup, find_packages
setup(
    name='radar_navigation', version='0.1.0', packages=find_packages(),
    py_modules=['radar_test', 'radar_viewer'],
    install_requires=['setuptools', 'pyserial==3.5', 'PyYAML'],
    extras_require={'analysis': ['numpy', 'matplotlib']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/radar_navigation']),
        ('share/radar_navigation', ['package.xml', 'README.md', 'PROGRESS.md', 'CONTRIBUTORS.md']),
        ('share/radar_navigation/launch', glob('launch/*.launch.py')),
        ('share/radar_navigation/config', glob('config/*')),
        ('share/radar_navigation/docs', glob('docs/*.md')),
        ('share/radar_navigation/scripts', glob('scripts/*.py') + glob('scripts/*.sh'))],
    entry_points={'console_scripts': [
        'radar=radar_navigation.cli:main',
        'awr1843_driver=radar_navigation.driver.driver_node:main',
        'radar_viewer=radar_viewer:main',
        'radar_node=radar_navigation.ros_node:main',
        'radar_control=radar_navigation.control_node:main',
        'radar_probe=radar_navigation.probe:main',
        'radar_fusion=radar_navigation.fusion_node:main',
        'radar_evaluate=radar_navigation.evaluate:main']})
