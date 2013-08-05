import unittest
from catkinize.utils import is_valid_version


class TestVersionValidator(unittest.TestCase):

    def test_valid_versions(self):
        self.assertTrue(is_valid_version('0.1.0'))
        self.assertTrue(is_valid_version('0.12.0'))
        self.assertTrue(is_valid_version('0.123.0'))
        self.assertTrue(is_valid_version('5.123.9'))

    def test_invalid_versions(self):
        self.assertFalse(is_valid_version('0.1'))
        self.assertFalse(is_valid_version('0.12.a'))
        self.assertFalse(is_valid_version('0.'))
        self.assertFalse(is_valid_version('123.9.5.2'))
