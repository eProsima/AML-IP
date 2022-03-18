"""Setup file to create amlip_aml_py library."""
from setuptools import setup

package_name = 'amlip_aml_py'

description = "AML-IP AML Tool"
long_description = description
# with open('README.md', 'r') as f:
#     long_description = f.read()

file_packages = [
    package_name,
]

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer_email='javierparis@eprosima.com',
    description=description,
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    test_suite='tests',
    entry_points={
        'console_scripts': [
            'amlip_aml_py = amlip_aml_py.main:main',
        ],
    },
)
