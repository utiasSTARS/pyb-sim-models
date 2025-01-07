from setuptools import setup

setup(
    name='pyb-sim-models',
    version='0.1.1',
    description='Simulation Models for PyBullet',
    author='Philippe Nadeau',
    author_email='philippe.nadeau@robotics.utias.utoronto.ca',
    license='MIT',
    packages=['pbsm', 'pbsm.ur5_2f85', 'pbsm.ur5', 'pbsm.Robotiq2f85'],
    install_requires=['pybullet==3.2.6', 'numpy==2.1.3', 'scipy==1.14.1'],
    include_package_data=True
)
