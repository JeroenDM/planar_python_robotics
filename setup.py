from setuptools import setup, find_packages, Extension

setup(
    name = 'ppr',
    version = '0.1.2',
    packages = find_packages(),
    description = 'Planar Python Robotics',
    long_description=('Software tool to experiment with 2D motion planning problems' +
    'for robot manipulators.'),
    author = 'Jeroen De Maeyer',
    author_email = 'jeroen.demaeyer@kuleuven.be',
    url = 'https://u0100037.pages.mech.kuleuven.be/planar_python_robotics/',
    download_url = 'https://gitlab.mech.kuleuven.be/u0100037/planar_python_robotics/raw/master/dist/ppr-0.1.1.tar.gz',
    keywords = ['robotics', 'motion planning'],
    classifiers = [],
    install_requires=['scipy', 'matplotlib'],
    python_requires='>=3',
    ext_package='ppr.cpp',
    ext_modules=[Extension('ppr.cpp', ['ppr/cpp/graph.i',
                                       'ppr/cpp/include/graph.h',
                                       'ppr/cpp/src/graph.cxx'],
                            swig_opts=['-c++', 'python', '-Ippr/cpp/include'])],
)
