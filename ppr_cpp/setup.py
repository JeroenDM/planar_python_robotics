#!/usr/bin/env python
"""
 This script is not working now!
 Some problems with including Eigen
"""

from distutils.core import setup, Extension

geometry_module = Extension('_geometry',
                            sources=['geometry_wrap.cxx', 'geometry.cxx'],
                            include_dirs=['I /home/jeroen/Eigen3/'])

setup(name='geometry',
      version='0.1',
      description='Geometric shapes and some util functions.',
      author='Jeroen De Maeyer',
      author_email='jeroendemaeyer@kuleuven.be',
      packages=['distutils', 'distutils.command'],
      ext_modules = [geometry_module]
     )