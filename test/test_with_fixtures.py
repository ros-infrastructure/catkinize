import os
import unittest
# workaround: StringIO now works with py2 and py3
try:
    from io import StringIO
except ImportError:
    from cStringIO import StringIO

import imp
imp.load_source('catkinize_cmakelists',
                os.path.join(os.path.dirname(__file__),
                             '..', 'scripts', 'catkinize_cmakelists.py'))

from catkinize_cmakelists import main


class CatkinizeCmakeFixturesTest(unittest.TestCase):

    def compare_contents(self, filename, str1, str2):
        for count, (line1, line2) in enumerate(zip(str1.splitlines(), str2.splitlines())):
            self.assertEqual(line1, line2, "%s:%s: '%s'!='%s'" % (filename, count, line1, line2))
        self.assertEqual(len(str1.splitlines()), len(str2.splitlines()))

    def run_with_fixture(self, case):
        infile = os.path.join(os.path.dirname(__file__), 'fixtures', 'CMakeLists.%s.txt.in' % case)
        manfile = os.path.join(os.path.dirname(__file__), 'fixtures', 'manifest.%s.xml' % case)
        outfile = os.path.join(os.path.dirname(__file__), 'fixtures', 'CMakeLists.%s.txt.out' % case)
        result_buf = StringIO.StringIO()
        try:
            main(['foo', infile, manfile], outstream=result_buf)
        except SystemExit:
            pass
        with open(outfile, 'r') as fhand:
            expect = fhand.read()
        # self.assertEqual(expect, result_buf.getvalue(), "%s\n!=\n%s" % (expect, result_buf.getvalue()))
        self.compare_contents(outfile, expect, result_buf.getvalue())

    def test_stage(self):
        self.run_with_fixture('stage')

    def test_rpekf(self):
        self.run_with_fixture('rpekf')

    def test_navfn(self):
        self.run_with_fixture('navfn')
