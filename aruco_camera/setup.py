from setuptools import find_packages, setup

package_name = 'aruco_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pytest'],
    zip_safe=True,
    maintainer='shu',
    maintainer_email='shu.xiao@students.iaac.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "aruco_detection = aruco_camera.aruco_detection:main",
            "multi_arucos_detection = aruco_camera.multi_aruco_detection:main",
            "camera_static = aruco_camera.camera_static:main",
            "execution = aruco_camera.execution:main",
            "execution_test = aruco_camera.execution_test:main"
        ],
    },
)
