# DOCUMENT FOUND AT https://nesi.github.io/perf-training/python-scatter/ctypes
#  
# run:
# python share_setup_build.py build
# to compile share object
#

from setuptools import setup, Extension

# Compile *.cpp* into a shared library 
setup(
    #...
    ext_modules=[Extension('cgal_intersection_tri', ['share_CGAL_intersection.cpp'],),],
    #ext_modules=[Extension('square', ['share_hello_square.c'],),],
    
)

