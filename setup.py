#!/usr/bin/env python

from setuptools import setup, find_packages

# Prevent "TypeError: 'NoneType' object is not callable" error
# when running python setup.py test
# (see http://www.eby-sarna.com/pipermail/peak/2010-May/003357.html)
try:
    import multiprocessing
except ImportError:
    pass

setup(name='catkinize',
      version='0.1',
      description='Scripts to convert rosbuild projects to catkin',
      author='Issac Trotts',
      author_email='itrotts@willowgarage.com',
      url='https://github.com/ijt/catkinize',
      scripts=['scripts/catkinize_manifest_xml_to_package_xml.py',
               'scripts/catkinize_cmakelists.py',
               'scripts/catkinize',
               'scripts/catkinize_stack'],
      packages=find_packages(exclude=("tests",)),
      test_suite='nose.collector',
      tests_require=['nose'])
