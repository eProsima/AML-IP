"""Setup file to create amlip_tensorflow_inference_rosbot2r_demo library."""
from setuptools import setup

package_name = 'amlip_tensorflow_inference_rosbot2r_demo'

description = 'eProsima AML-IP TensorFlow Inference Demo'
long_description = description

file_packages = [
    package_name,
]

setup(
    name=package_name,
    version='0.2.0',
    packages=file_packages,
    long_description=long_description,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eprosima',
    maintainer_email='irenebandera@eprosima.com',
    description=description,
    license='Apache License, Version 2.0',
    entry_points={},
)
