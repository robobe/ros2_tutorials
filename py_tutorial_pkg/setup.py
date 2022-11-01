from setuptools import setup
import os 
from glob import glob

package_name = 'py_tutorial_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "config"), glob('config/*.rviz'))  
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='user',
    maintainer_email='robo2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "minimal_dia=py_tutorial_pkg.diagnostic_demo:main",
            'publisher_qos=py_tutorial_pkg.publisher_custom_minimal_qos:main',
            'subscriber_qos=py_tutorial_pkg.subscriber_custom_minimal_qos:main',
            "turtle_goto=py_tutorial_pkg.turtle_goto:main",
            "turtle_goto_tf=py_tutorial_pkg.turtle_goto_tf:main",
            "turtle_goto_rviz=py_tutorial_pkg.turtle_goto_rviz:main"
        ],
    },
)
