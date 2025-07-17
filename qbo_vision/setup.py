from setuptools import setup

package_name = 'qbo_vision'
python_module = 'qbo_vision_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[python_module],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'onnxruntime', 'faiss-cpu', 'cv_bridge', 'rclpy'],
    zip_safe=True,
    maintainer='qbo',
    maintainer_email='qbo@example.com',
    description='Face recognition node using ArcFace and FAISS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_capture_node = qbo_vision_py.face_capture_node:main',
        ],
    },
)
