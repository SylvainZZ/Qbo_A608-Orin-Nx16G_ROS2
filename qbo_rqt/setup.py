from setuptools import setup, find_packages

package_name = 'qbo_rqt'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/pan_tilt_plugin_v1.ui']),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        # ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description='RQt plugin collection for the QBO platform, providing GUI tools to monitor and control subsystems like Pan & Tilt actuators, diagnostics, and motor states in ROS 2.',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    entry_points={
    'console_scripts': [],
    'rqt_gui_py.plugin': [
        'pan_tilt_plugin = qbo_rqt.pan_tilt_plugin:PanTiltPlugin',
        ],
    },
)
