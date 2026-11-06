from setuptools import setup, find_packages
setup(
    name='radar_navigation', version='0.1.0', packages=find_packages(),
    py_modules=['radar_test', 'radar_viewer'],
    install_requires=['pyserial==3.5'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/radar_navigation']),
        ('share/radar_navigation', ['package.xml', 'awr1843_profile.cfg']),
        ('share/radar_navigation/launch', ['launch/radar.launch.py']),
        ('share/radar_navigation/config', ['config/nav2_radar.yaml'])],
    entry_points={'console_scripts': [
        'radar=radar_navigation.cli:main',
        'radar_node=radar_navigation.ros_node:main',
        'radar_fusion=radar_navigation.fusion_node:main',
        'radar_evaluate=radar_navigation.evaluate:main']})
