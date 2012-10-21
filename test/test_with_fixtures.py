import os
import sys
import unittest
import tempfile
import shutil

import mock

import StringIO


import imp
imp.load_source('catkinize_cmakelists',
                os.path.join(os.path.dirname(__file__),
                             '..', 'scripts', 'catkinize_cmakelists.py'))

from catkinize_cmakelists import main


class CatkinizeCmakeFixturesTest(unittest.TestCase):

    def compare_contents(self, filename, str1, str2):
        for count, (line1, line2) in enumerate(zip(str1.splitlines(), str2.splitlines())):
            self.assertEqual(line1, line2, "%s:%s: %s!=%s" % (filename, count, line1, line2))
        self.assertEqual(len(str1.splitlines()), len(str2.splitlines()))
    
    def test_main(self):
        for case in ['rpekf', 'stage']:
            infile = os.path.join(os.path.dirname(__file__), 'fixtures', 'CMakeLists.%s.txt.in' % case)
            outfile = os.path.join(os.path.dirname(__file__), 'fixtures', 'CMakeLists.%s.txt.out' % case)
            result_buf = StringIO.StringIO()
            main(['foo', infile], outstream=result_buf)
            with open(outfile, 'r') as fhand:
                expect = fhand.read()
            self.compare_contents(outfile, expect, result_buf.getvalue())
