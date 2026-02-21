from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['approx_manifold_prm'],
    package_dir={'': 'src'}
)

setup(**d)
