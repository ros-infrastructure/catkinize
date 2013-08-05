import os
import unittest
import tempfile
import shutil

from catkinize.main import _create_changesets, perform_changes


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
        self.assertEqual(
            [],
            _create_changesets(self.foo_stack, ['stack'], [], [])
        )
        self.assertEqual(
            [(self.foo_stack_xml, self.foo_stack_xml + '.backup', None, None)],
            _create_changesets(self.foo_stack, ['stack.xml'], [], [])
        )
        self.assertEqual(
            [(self.foo_stack_xml, self.foo_stack_xml + '.backup', None, None)],
            _create_changesets(self.foo_stack, ['stack.xml'])
        )
        self.assertEqual(
            [(None, None, 'foo', 'hello')],
            _create_changesets('.', ['baz'], ['foo'], ['hello'])
        )
        self.assertEqual(
            [(self.foo_stack_xml, self.foo_stack_xml + '.backup', os.path.join(
                self.foo_stack, 'foo'), 'hello')],
            _create_changesets(
                self.foo_stack, ['stack.xml'], ['foo'], ['hello'])
        )
        self.assertEqual(
            [(None, None, 'foo', 'hello'), (None, None, 'fooz', 'hello2')],
            _create_changesets(
                '.', ['baz', 'bam'], ['foo', 'fooz'], ['hello', 'hello2'])
        )
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
                         (targetfile2, targetfile2 +
                          '.backup', foofile, 'foo'),
                         (None, None, barfile, 'bar'),
                         (None, None, subfile, 'pip')])
        self.assertTrue(os.path.exists(foofile))
        self.assertTrue(os.path.exists(barfile))
        self.assertFalse(os.path.exists(targetfile))
        self.assertFalse(os.path.exists(targetfile2))
        self.assertTrue(os.path.exists(targetfile + '.backup'))
        self.assertTrue(os.path.exists(targetfile2 + '.backup'))
        self.assertTrue(os.path.exists(subfile))
