from setuptools import setup

setup(
    name = 'ppr',
    version = '0.1.0',
    packages = ['ppr'],
    description = 'Planar Python Robotics',
    author = 'Jeroen De Maeyer',
    author_email = 'jeroen.demaeyer@kuleuven.be',
    url = 'https://u0100037.pages.mech.kuleuven.be/planar_python_robotics/',
    download_url = 'https://gitlab.mech.kuleuven.be/u0100037/planar_python_robotics/raw/master/dist/ppr-0.1.0.tar.gz',
    keywords = ['robotics', 'motion planning'],
    classifiers = [],
    install_requires=['scipy', 'matplotlib'],
    python_requires='>=3',
)
