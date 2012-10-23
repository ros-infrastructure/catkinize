import os
import StringIO
import unittest

import imp
imp.load_source('catkinize_cmakelists',
                os.path.join(os.path.dirname(__file__),
                             '..', 'scripts', 'catkinize_cmakelists.py'))

from catkinize_cmakelists import convert_cmakelists, \
    make_header_lines, convert_line, convert_boost, LINK_BOOST_RX, main
from utils import create_temp_file


class CatkinizeCmakeTest(unittest.TestCase):

    def test_convert_line(self):
        self.assertEqual('foo catkin_add_gtest(bar)', convert_line('foo rosbuild_add_gtest(bar)'))
        self.assertEqual('foo bla(bar)', convert_line('foo rosbuild_bla(bar)'))

    def test_convert_cmakelists(self):
        lines = ['foo',
                 'rosbuild_add_boost_directories(foo bar)',
                 'baz',
                 'rosbuild_add_gtest(bar)',
                 'bum',
                 'rosbuild_link_boost(fooz baaz)',
                 'endtest']
        result_lines = convert_cmakelists('myprojecttest', lines)
        self.assertTrue('catkin_add_gtest(bar)' in result_lines, result_lines)
        self.assertFalse('rosbuild_add_gtest(bar)' in result_lines, result_lines)
        # don't care too much about header contents in this test
        expect_end = ['foo',
                      'baz',
                      'catkin_add_gtest(bar)',
                      'bum',
                      'find_package(Boost REQUIRED COMPONENTS baaz)',
                      'include_directories(${Boost_INCLUDE_DIRS})',
                      'target_link_libraries(fooz ${Boost_LIBRARIES})',
                      'endtest']
        self.assertEqual(expect_end, result_lines[-8:])

    def test_convert_cmakelists_handles_commented_lines_with_rosbuild_link_boost(self):
        line = '#rosbuild_link_boost(${PROJECT_NAME} thread)'
        result_lines = convert_cmakelists('project_name', [line])
        self.assertEqual('#link_boost(${PROJECT_NAME} thread)', result_lines[-1])

    def test_make_header_lines(self):
        lines = make_header_lines('foo', '')
        self.assertTrue('catkin_package(' in lines, lines)
        self.assertTrue('project(foo)' in lines, lines)

    def test_link_boost_re_single_line(self):
        orig = "rosbuild_link_boost(foo bar)"
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)

    def test_link_boost_re_multi_line(self):
        orig = """\
rosbuild_link_boost(foo
bar
baz)"""
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)
        orig = """\
rosbuild_link_boost(foo
bar
baz
)"""
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)

    def test_link_boost_re_indent(self):

        orig = """\
  rosbuild_link_boost(foo
    bar
    baz
  )"""
        m = LINK_BOOST_RX.match(orig)
        self.assertEqual(orig, m.string)

    def test_convert_boost_none(self):
        lines = ['foo', 'rosbuild_add_boost_directories()', 'baz']
        result_lines = list(convert_boost(lines))
        self.assertEqual(['foo', 'baz'], result_lines)

    def test_convert_boost_many(self):
        lines = ['foo', 'rosbuild_link_boost(foo bar baz)', 'baz']
        result_lines = list(convert_boost(lines))
        expect = ['foo',
                  'find_package(Boost REQUIRED COMPONENTS bar baz)',
                  'include_directories(${Boost_INCLUDE_DIRS})',
                  'target_link_libraries(foo ${Boost_LIBRARIES})',
                  'baz']
        self.assertEqual(expect, result_lines)

    def test_convert_boost_multiline(self):
        lines = ['foo', 'rosbuild_link_boost(foo', 'bar' 'baz)']
        line_gen = convert_boost(lines)
        self.assertRaises(ValueError, list, line_gen)

    def test_that_manifest_dependencies_are_included(self):
        package_name = 'my_package'

        # Make a CMakeLists.txt file.
        cmakelists_path = create_temp_file(contents='')

        # Make a manifest.xml file.
        manifest_path = create_temp_file(contents='''
<package>
  <depend package="pkg_a"/>
  <depend package="pkg_b"/>
  <rosdep name="pkg_c" />
  <rosdep name="pkg_d" />
</package>
''')
        output = StringIO.StringIO()

        # Run the catkinize script.
        main([package_name, cmakelists_path, manifest_path], output)

        # Check that dependencies are listed.
        out_lines = output.getvalue().splitlines()
        cpkg_lines = [l for l in out_lines if 'catkin_package' in l]
        self.assertEqual(1, len(cpkg_lines), str(cpkg_lines))
        cpkg_line = cpkg_lines[0]
        pattern = 'DEPENDS pkg_a pkg_b pkg_c pkg_d'
        msg = '''
    Failed to find package dependencies in the catkin_package() call:
    Expected to find a line containing
    %s
    but got
    %s
''' % (pattern, cpkg_line)
        self.assertTrue(pattern in cpkg_line, msg)

