from setuptools import setup

setup(
    name='neopixel_ring',
    version='1.0.0',
    packages=['neopixel_ring'],
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
    test_suite='test',
    entry_points={
        'console_scripts': [
            'neopixel_node = neopixel_ring.neopixel_node:main',
        ],
    },
)
