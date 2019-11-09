from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['state_machine','state_machine.planner',
                  'state_machine.env_interface', 
                  'state_machine.controller',
                  'state_machine.gate_estimator',
                  'state_machine.utils'
                  ],
        package_dir={'':'src'}
)

setup(**d)
