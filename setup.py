#!/usr/bin/env python

from distutils.core import setup
import setuptools

setup(name='vs068-pb',
      version='0.1',
      description='VS068 arm demo',
      author='Sam Wilcock',
      author_email='el18sw@leeds.ac.uk',
      url='https://swilcock0.github.io/',
      packages=setuptools.find_packages('.'),
      package_dir={'': '.'},
     )