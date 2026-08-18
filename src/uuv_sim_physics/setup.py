import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uuv_sim_physics'

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # The world is a build product of config/, so both are installed: a
        # deployed copy must carry the parameters that generated its world,
        # otherwise the provenance record cannot be reproduced from it.
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Christos Alexandris',
    maintainer_email='calexandris@uniwa.gr',
    description=(
        'Physics-capable execution path for the AUV Simulator, additive to the '
        'existing deterministic reduced-order simulation.'
    ),
    license='MIT',
    tests_require=['pytest'],
    # No console_scripts at M1. Entry points arrive with the runner (M3) and
    # the sensor bridges (M4); an empty block now keeps the reduced backend
    # importable without registering anything that could shadow an existing
    # command.
    entry_points={'console_scripts': []},
)
