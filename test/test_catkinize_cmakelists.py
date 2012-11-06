import os
import StringIO
import unittest

import imp
imp.load_source('catkinize_cmakelists',
                os.path.join(os.path.dirname(__file__),
                             '..', 'scripts', 'catkinize_cmakelists.py'))

from catkinize_cmakelists import make_header_lines, convert_snippet, convert_boost_snippet, ARGUMENT_SPLITTER, FUNCALL_PATTERN


class CatkinizeCmakeTest(unittest.TestCase):

    def test_convert_snippet(self):
        self.assertEqual(' catkin_add_gtest(bar)', convert_snippet(' rosbuild_add_gtest', '(bar)'))
        self.assertEqual(' catkin_add_nosetests(bar)', convert_snippet(' rosbuild_add_pyunit', '(bar)'))

    def test_make_header_lines(self):
        lines = make_header_lines('foo', '')
        self.assertTrue('catkin_package(' in lines, lines)
        self.assertTrue('project(foo)' in lines, lines)


    def test_argument_splitter_single_line(self):
        orig = "(foo bar)"
        m = ARGUMENT_SPLITTER.match(orig)
        self.assertEqual(orig, m.string)

    def test_funcall_pattern(self):
        content = """\
#link_boost(${PROJECT_NAME} thread)
link_boost(${PROJECT_NAME} system)
"""
        tokens = FUNCALL_PATTERN.split(content)
        self.assertEqual(['#',
                          'link_boost',
                          '(${PROJECT_NAME} thread)',
                          '\n',
                          'link_boost',
                          '(${PROJECT_NAME} system)',
                          '\n'], tokens)

    def test_argument_splitter_multi_line(self):
        orig = """\
(foo
 bar
 baz)"""
        m = ARGUMENT_SPLITTER.match(orig)
        self.assertEqual(orig, m.string)
        orig = """\
(
  foo
  bar
  baz
)"""
        m = ARGUMENT_SPLITTER.match(orig)
        self.assertEqual(orig, m.string)

    def test_argument_splitter_indent(self):

        orig = """\
  (foo
    bar
    baz
  )"""
        m = ARGUMENT_SPLITTER.match(orig)
        self.assertEqual(orig, m.string)

    def test_convert_boost_snippet_none(self):
        replacement, comps = convert_boost_snippet('rosbuild_add_boost_directories', '()')
        self.assertEqual((None, None), (replacement, comps))

    def test_convert_boost_snippet_many(self):
        replacement, comps = convert_boost_snippet('rosbuild_link_boost', '(foo bar baz)')
        self.assertEqual('target_link_libraries(foo ${Boost_LIBRARIES})', replacement)
        self.assertEqual(['bar', 'baz'], comps)

    def test_convert_boost_snippet_multiline(self):
        replacement, comps = convert_boost_snippet('rosbuild_link_boost', '(foo\nbar\nbaz)')
        self.assertEqual('target_link_libraries(foo ${Boost_LIBRARIES})', replacement)
        self.assertEqual(['bar', 'baz'], comps)
