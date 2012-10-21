import os
import sys
import unittest
import tempfile
import shutil

import mock


import imp
imp.load_source('catkinize_cmakelists',
                os.path.join(os.path.dirname(__file__),
                             '..', 'scripts', 'catkinize_cmakelists.py'))

from catkinize_cmakelists import main, convert_cmakelists, add_header_if_needed, make_header_lines, convert_line, convert_boost, LINK_BOOST_RX

class CatkinizeCmakeTest(unittest.TestCase):

    def test_convert_line(self):
        self.assertEqual('foo catkin_add_gtest(bar)', convert_line('foo rosbuild_add_gtest(bar)'))
        self.assertEqual('foo bla(bar)', convert_line('foo rosbuild_bla(bar)'))

    def test_make_header_lines(self):
        lines = make_header_lines('foo')
        self.assertTrue('# catkin_package(' in lines, lines)
        self.assertTrue('project(foo)' in lines, lines)

    def test_add_header_if_needed(self):
        lines = ['foo', 'bar']
        result_lines = add_header_if_needed(lines, ['head'])
        self.assertEqual(['head', '', 'foo', 'bar'], result_lines)
        lines = ['cmake_minimum_required(VERSION 2.8.3)', 'project(foo)', 'find_package(catkin REQUIRED)', 'catkin_package()']
        result_lines = add_header_if_needed(lines, ['head'])
        self.assertEqual(lines, result_lines)

    def test_link_boost_re(self):
        orig = "rosbuild_link_boost(foo bar)"
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)
        orig = """rosbuild_link_boost(foo
bar
baz)"""
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)
        orig = """rosbuild_link_boost(foo
bar
baz
)"""
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)
        orig = """  rosbuild_link_boost(foo
    bar
    baz
  )"""
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)

    def test_convert_boost(self):
        lines = ['foo', 'rosbuild_add_boost_directories()', 'baz']
        result_lines = list(convert_boost(lines))
        self.assertEqual(['foo', 'baz'], result_lines)
        lines = ['foo', 'rosbuild_link_boost(foo bar baz)', 'baz']
        result_lines = list(convert_boost(lines))
        expect = ['foo',
                  'find_package(Boost REQUIRED COMPONENTS bar baz)',
                  'include_directories(${Boost_INCLUDE_DIRS})',
                  'target_link_libraries(foo ${Boost_LIBRARIES})',
                  'baz']
        self.assertEqual(expect, result_lines)
        lines = ['foo', 'rosbuild_link_boost(foo', 'bar' 'baz)']
        line_gen = convert_boost(lines)
        self.assertRaises(ValueError, list, line_gen)

    def test_main(self):
        pass
