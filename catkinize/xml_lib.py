"""
This module contains functions for XML parsing and generation.
"""

import re


def xml_find(tree, tag_name):
    """
    Return the first node with the given tag name, or Empty if none is found.
    """
    item = tree.find(tag_name)
    return item if item is not None else Empty()


class Empty(object):
    """Empty result of a find operation on an XML tree."""
    def __init__(self):
        self.text = ''

    def getchildren(self):
        return []


def comment_out_tags_named(xml, tag_name):
    """
    Comment out all tags with a given name.

    >>> comment_out_tags_named('<a/><b/><c></c><b q="foo">bop</b><d>', 'b')
    '<a/><!-- <b/> --><c></c><!-- <b q="foo">bop</b> --><d>'
    """
    rx1 = re.compile('(\\b%s/>)' % tag_name)
    rx2 = re.compile('(<%s\\b)' % tag_name)
    rx3 = re.compile('(</%s>)' % tag_name)
    xml = rx1.sub(r'\1 -->', xml)
    xml = rx2.sub(r'<!-- \1', xml)
    xml = rx3.sub(r'\1 -->', xml)
    return xml


