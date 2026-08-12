from glob import glob
import os


def _external_trees(package_name):
    """(destination, [files]) for every file under models/external."""
    out = []
    for root, _dirs, files in os.walk(os.path.join('models', 'external')):
        if not files:
            continue
        rel = os.path.relpath(root, 'models')
        out.append((os.path.join('share', package_name, 'models', rel),
                    [os.path.join(root, f) for f in files]))
    return out

from setuptools import find_packages, setup

package_name = 'uuv_mode_aware_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.json')),
        (os.path.join('share', package_name, 'models', 'meshes'),
         glob('models/meshes/*.obj') + glob('models/meshes/*.dae')
         + glob('models/meshes/*.png')),
        (os.path.join('share', package_name, 'models', 'textures'),
         glob('models/textures/*.png')),
        # Downloaded scenery. Installed as whole trees: a glTF references its
        # .bin and its textures/ by relative path, so a flat glob silently
        # produces a model that loads with no geometry and no error.
        *[(os.path.join('share', package_name, 'models', 'external',
                        os.path.basename(d)), glob(os.path.join(d, '*')))
          for d in glob('models/external/*') if os.path.isdir(d)],
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Christos Alexandris',
    maintainer_email='calexandris@uniwa.gr',
    description=(
        'Mode-aware adaptive navigation for underwater vehicles under '
        'multi-modal sensing degradation.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv-sim = uuv_mode_aware_navigation.cli:main',
            'water_column = uuv_mode_aware_navigation.nodes.water_column_node:main',
            'optical_feedback = '
            'uuv_mode_aware_navigation.nodes.optical_feedback_node:main',
            'mode_manager = uuv_mode_aware_navigation.nodes.mode_manager_node:main',
            'vehicle = uuv_mode_aware_navigation.nodes.vehicle_node:main',
            'status_display = uuv_mode_aware_navigation.nodes.status_display:main',
            'teleop = uuv_mode_aware_navigation.nodes.teleop_node:main',
            'scenario_director = uuv_mode_aware_navigation.nodes.scenario_node:main',
            'control_panel = uuv_mode_aware_navigation.nodes.control_panel:main',
            'study3_control = uuv_mode_aware_navigation.study3.control_window:main',
            'fish_school = uuv_mode_aware_navigation.nodes.fish_school:main',
        ],
    },
)
