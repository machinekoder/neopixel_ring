from setuptools import find_packages
from setuptools import setup

setup(
    name='neopixel_ring',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[],
    install_requires=['setuptools'],
    author='Alexander Roessler',
    author_email='alex@machinekoder.com',
    maintainer='Alexander Roessler',
    maintainer_email='alex@machinekoder.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='NeoPixel Ring control fro ROS2.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neopixel_node = neopixel_ring.neopixel_node:main',
        ],
    },
)
