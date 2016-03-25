from setuptools import setup

setup(name='andorEmccd',
    version='0.1',
    packages=['andorEmccd'],
    entry_points={
        "console_scripts": ["andorEmccd_controller=andorEmccd.controller:main"],
    }
)