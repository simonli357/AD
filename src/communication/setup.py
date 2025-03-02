from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['python_server', 'python_server.service_calls', 'python_server.msg'],
    package_dir={'': 'src'}
)

setup(**setup_args)
