from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'pathfinder',
        'pathfinder.world',
        'pathfinder.planning',
        'pathfinder.safety',
        'pathfinder.robot',
        'pathfinder.ros',
        'pathfinder.client',
        'pathfinder.simulation',
    ],
    package_dir={'': 'src'},
)
setup(**d)
