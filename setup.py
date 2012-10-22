#!/usr/bin/env python

from setuptools import setup

setup(name='catkinize',
      version='0.1',
      description='Scripts to convert rosbuild projects to catkin',
      author='Issac Trotts',
      author_email='itrotts@willowgarage.com',
      url='https://github.com/ijt/catkinize',
      scripts=['scripts/catkinize_manifest_xml_to_package_xml.py',
               'scripts/catkinize_cmakelists.py'],
      test_suite='nose.collector',
      tests_require=['nose'])
