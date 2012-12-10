import os
import StringIO
import unittest
import tempfile
import shutil

from catkinize.main import catkinize_package, catkinize_stack, _create_changesets, perform_changes


class CatkinizeTest(unittest.TestCase):

    def setUp(self):
        self.root_dir = tempfile.mkdtemp()
        self.foo_stack = os.path.join(self.root_dir, "foostack")
        os.makedirs(self.foo_stack)
        self.foo_stack_xml = os.path.join(self.foo_stack, 'stack.xml')
        with open(self.foo_stack_xml, "w") as fhand:
            fhand.write("<stack/>")
        self.oldfile = os.path.join(self.foo_stack, 'oldfile')
        with open(self.oldfile, "w") as fhand:
            fhand.write("oldfile")
        with open(self.oldfile + ".backup", "w") as fhand:
            fhand.write("oldfile")
        self.foo_pkg = os.path.join(self.foo_stack, "foopkg")
        os.makedirs(self.foo_pkg)
        self.foo_pkg_xml = os.path.join(self.foo_pkg, 'manifest.xml')
        with open(self.foo_pkg_xml, "w") as fhand:
            fhand.write("<package/>")
        self.foo_pkg_cmake = os.path.join(self.foo_pkg, 'CMakeLists.txt')
        with open(self.foo_pkg_cmake, "w") as fhand:
            fhand.write("")

    def tearDown(self):
        shutil.rmtree(self.root_dir)

    def test_create_changesets(self):
        self.assertEqual([], _create_changesets('.', []))
        self.assertEqual([], _create_changesets('.', [], [], []))
        self.assertEqual([], _create_changesets('.', ['/foo'], [], []))
        self.assertEqual([], _create_changesets(self.foo_stack, ['stack'], [], []))
        self.assertEqual([(self.foo_stack_xml, self.foo_stack_xml + '.backup', None, None)],
                         _create_changesets(self.foo_stack, ['stack.xml'], [], []))
        self.assertEqual([(self.foo_stack_xml, self.foo_stack_xml + '.backup', None, None)],
                         _create_changesets(self.foo_stack, ['stack.xml']))
        self.assertEqual([(None, None, 'foo', 'hello')],
                         _create_changesets('.', ['baz'], ['foo'], ['hello']))
        self.assertEqual([(self.foo_stack_xml, self.foo_stack_xml + '.backup', os.path.join(self.foo_stack, 'foo'), 'hello')],
                         _create_changesets(self.foo_stack, ['stack.xml'], ['foo'], ['hello']))
        self.assertEqual([(None, None, 'foo', 'hello'), (None, None, 'fooz', 'hello2')],
                         _create_changesets('.', ['baz', 'bam'], ['foo', 'fooz'], ['hello', 'hello2']))
        raised = False
        try:
            _create_changesets(self.foo_stack, ['oldfile'])
        except ValueError:
            raised = True
        self.assertTrue(raised, "Expected ValueError")

    def test_perform_changes(self):
        targetfile = os.path.join(self.foo_stack, 'targetfile')
        with open(targetfile, "w") as fhand:
            fhand.write("targetfile")
        targetfile2 = os.path.join(self.foo_stack, 'targetfile2')
        with open(targetfile2, "w") as fhand:
            fhand.write("targetfile2")
        foofile = os.path.join(self.foo_stack, 'foo')
        barfile = os.path.join(self.foo_stack, 'bar')
        subfile = os.path.join(self.foo_stack, 'subdir', 'bip')
        perform_changes([(targetfile, targetfile + '.backup', None, None),
                         (targetfile2, targetfile2 + '.backup', foofile, 'foo'),
                         (None, None, barfile, 'bar'),
                         (None, None, subfile, 'pip')])
        self.assertTrue(os.path.exists(foofile))
        self.assertTrue(os.path.exists(barfile))
        self.assertFalse(os.path.exists(targetfile))
        self.assertFalse(os.path.exists(targetfile2))
        self.assertTrue(os.path.exists(targetfile + '.backup'))
        self.assertTrue(os.path.exists(targetfile2 + '.backup'))
        self.assertTrue(os.path.exists(subfile))

    def test_catkinize_package(self):
        self.assertEqual([
                (self.foo_pkg_cmake, self.foo_pkg_cmake + '.backup', self.foo_pkg_cmake, '# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html\ncmake_minimum_required(VERSION 2.8.3)\nproject(foopkg)\n# Load catkin and all dependencies required for this package\n# TODO: remove all from COMPONENTS that are not catkin packages.\nfind_package(catkin REQUIRED )\n\n# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})\n# TODO: fill in what other packages will need to use this package\n## LIBRARIES: libraries you create in this project that dependent projects also need\n## CATKIN_DEPENDS: catkin_packages dependent projects also need\n## DEPENDS: system dependencies of this project that dependent projects also need\ncatkin_package(\n    DEPENDS  # TODO\n    CATKIN-DEPENDS # TODO\n    INCLUDE_DIRS # TODO include\n    LIBRARIES # TODO\n)'),
                (self.foo_pkg_xml, self.foo_pkg_xml + '.backup', os.path.join(self.foo_pkg, 'package.xml'), '<package>\n  <name>foopkg</name>\n  <version>0.1.2</version>\n  <description></description>\n  <!-- <maintainer></maintainer> -->\n\n  <license></license>\n\n  <url type="website"></url>\n  <!-- <url type="bugtracker"></url> -->\n\n  <author></author>\n\n  <export>\n\n  </export>\n</package>')],
                         catkinize_package(self.foo_pkg, '0.1.2'))

    def test_catkinize_stack(self):
        self.assertEqual([
                (self.foo_stack_xml, self.foo_stack_xml + '.backup', os.path.join(self.foo_stack, 'foostack/package.xml'), '<package>\n  <name>foostack</name>\n  <version>0.1.2</version>\n  <description></description>\n  <!-- <maintainer></maintainer> -->\n\n  <license></license>\n\n  <url type="website"></url>\n  <!-- <url type="bugtracker"></url> -->\n\n  <author></author>\n\n\n  <run_depend>foopkg</run_depend>\n\n\n\n\n\n  <export>\n    <metapackage/>\n  </export>\n</package>\n'),
                (self.foo_pkg_cmake, self.foo_pkg_cmake + '.backup', self.foo_pkg_cmake, '# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html\ncmake_minimum_required(VERSION 2.8.3)\nproject(foopkg)\n# Load catkin and all dependencies required for this package\n# TODO: remove all from COMPONENTS that are not catkin packages.\nfind_package(catkin REQUIRED )\n\n# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})\n# TODO: fill in what other packages will need to use this package\n## LIBRARIES: libraries you create in this project that dependent projects also need\n## CATKIN_DEPENDS: catkin_packages dependent projects also need\n## DEPENDS: system dependencies of this project that dependent projects also need\ncatkin_package(\n    DEPENDS  # TODO\n    CATKIN-DEPENDS # TODO\n    INCLUDE_DIRS # TODO include\n    LIBRARIES # TODO\n)'),
                (self.foo_pkg_xml, self.foo_pkg_xml + '.backup', os.path.join(self.foo_pkg, 'package.xml'), '<package>\n  <name>foopkg</name>\n  <version>0.1.2</version>\n  <description></description>\n  <!-- <maintainer></maintainer> -->\n\n  <license></license>\n\n  <url type="website"></url>\n  <!-- <url type="bugtracker"></url> -->\n\n  <author></author>\n\n  <export>\n\n  </export>\n</package>')],
                         catkinize_stack(self.foo_stack, '0.1.2'))
