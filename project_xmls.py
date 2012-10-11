def create_project_xml(package_name, version, description, maintainers,
                       licenses, website_url, bugtracker_url, authors,
                       build_depends, run_depends, test_depends, replaces,
                       conflicts, exports, architecture_independent,
                       metapackage):
    """
    Generate the contents of project.xml from some parameters.

    >>> desc = 'ROS communications-related packages, including client libs...'
    >>> pxml = create_project_xml(  # doctest: +ELLIPSIS
    ...     package_name='my_package',
    ...     version='1.2.3',
    ...     description=desc,
    ...     maintainers=[('Joe Smith', dict(email='joe.smith@gmail.com'))],
    ...     licenses=['BSD', 'GPL'],
    ...     website_url='http://wiki.ros.org/my_package',
    ...     bugtracker_url='http://www.github.com/my_org/my_package/issues',
    ...     authors=['John Doe',
    ...              ('Jane Doe', dict(email='jane.doe@example.com'))],
    ...     build_depends=['catkin',
    ...                    ('genmsg', dict(version_gte='1.1',
    ...                                    version_lt='2.0'))],
    ...     run_depends=['libbost-thread'],
    ...     test_depends=['gtest'],
    ...     conflicts=['my_old_package'],
    ...     replaces=['that_other_package'],
    ...     exports=[
    ...       ('rviz',
    ...        dict(plugin="${prefix}/plugin_description.xml"))
    ...     ],
    ...     architecture_independent=False,
    ...     metapackage=False)
    >>> # Check that it's valid XML:
    >>> import xml.etree.ElementTree as ET
    >>> tree = ET.XML(pxml)

    """
    maintainers_part = make_section('maintainer', maintainers)
    licenses_part = '\n'.join(
        indent('<license>%s</license>' % l)
        for l in licenses)
    authors_part = make_section('author', authors)
    build_depends_part = make_section('build_depend', build_depends)
    run_depends_part = make_section('run_depend', run_depends)
    test_depends_part = make_section('test_depend', test_depends)
    replaces_part = make_section('replace', replaces)
    conflicts_part = make_section('conflict', conflicts)
    exports_part = make_exports_section(exports, architecture_independent,
                                        metapackage)
    return '''\
<package>
  <name>%(package_name)s</name>
  <version>%(version)s</version>
  <description>%(description)s</description>
%(maintainers_part)s

%(licenses_part)s

  <url type="website">%(website_url)s</url>
  <url type="bugtracker">%(bugtracker_url)s</url>

%(authors_part)s

%(build_depends_part)s
%(run_depends_part)s
%(test_depends_part)s

%(replaces_part)s
%(conflicts_part)s

%(exports_part)s
</package>
''' % vars()

def make_section(tag_name, rows):
    """
    Make a string in XML format for a section with a given tag name.
    """
    return '\n'.join(make_tag_from_row(tag_name, r) for r in rows)

def make_tag_from_row(name, row):
    """
    Make an XML tag from a row.

    >>> make_tag_from_row('foo', 'bar')
    '<foo>bar</foo>'
    >>> make_tag_from_row('foo', ('bar', dict(baz='buzz')))
    '<foo baz="buzz">bar</foo>'
    """
    if isinstance(row, basestring):
        return make_tag(name, attrs_dict={}, contents=row)
    if isinstance(row, tuple):
        return make_tag(name, attrs_dict=row[1], contents=row[0])

def make_tag(name, attrs_dict, contents):
    return '<%s>%s</%s>' % (space_join([name, dict_to_attrs(attrs_dict)]),
                            contents,
                            name)

def make_empty_tag(name, attrs_dict):
    return '<%s/>' % space_join([name, dict_to_attrs(attrs_dict)])

def space_join(words):
    return ' '.join(w for w in words if w)

def indent(s):
    return '  ' + s

def dict_to_attrs(d):
    """
    Convert a dictionary to a string containing attributes in XML format.
    """
    return ' '.join('%s="%s"' % (k, v) for k, v in d.items())

def make_exports_section(exports, architecture_independent, metapackage):
    parts = [indent(make_empty_tag(name, attrs_dict))
             for name, attrs_dict in exports]
    if architecture_independent:
        parts.append('<architecture_independent/>')
    if metapackage:
        parts.append('<metapackage/>')
    return '\n'.join(parts)

