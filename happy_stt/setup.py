from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'happy_stt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # パッケージリソース定義
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniil',
    maintainer_email='daniil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt = happy_stt.speech_to_text:main',
            'launch_stt_server = happy_stt.launch_stt_server:main'

        ],
    },
)
