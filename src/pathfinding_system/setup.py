from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'pathfinding_system',
        'pathfinding_system.world',
        'pathfinding_system.planning',
        'pathfinding_system.safety',
        'pathfinding_system.robot',
        'pathfinding_system.client',
    ],
    package_dir={'': 'src'},
)
setup(**d)
