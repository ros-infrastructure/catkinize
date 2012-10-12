#!/usr/bin/env python

'''Functions for generating package.xml files.'''

import xml.etree.ElementTree as ET
from optparse import OptionParser
import re
import sys

SPACE_COMMA_RX = re.compile(r'[, ]+')

def main():
    usage = 'usage: %prog [options] manifest_xml_path package_name version'
    parser = OptionParser(usage)
    parser.add_option('-a', '--architecture_independent',
                      dest='architecture_independent',
                      action='store_true', default=False,
                      help='Whether the package runs on all platforms')
    parser.add_option('-m', '--metapackage',
                      dest='metapackage',
                      action='store_true', default=False,
                      help='Whether this package is a metapackage')
    parser.add_option('-b', '--bugtracker_url',
                      dest='bugtracker_url',
                      type='string', default='',
                      help='URL for issues related to this package')
    parser.add_option('-r', '--replaces',
                      dest='replaces',
                      type='string', default='',
                      help='Comma-separated list of packages replaced by ' +
                           'this package')
    parser.add_option('-c', '--conflicts',
                      dest='conflicts',
                      type='string', default='',
                      help='Comma-separated list of pkgs conflicting ' +
                           'with this package')
    options, args = parser.parse_args()
    if len(args) != 3:
        parser.error('wrong number of arguments')

    manifest_xml_path = args[0]
    package_name = args[1]
    version = args[2]
    with open(manifest_xml_path) as f:
        manifest_xml_str = f.read()
        pkg_xml = make_from_manifest(manifest_xml_str,
                                     package_name,
                                     version,
                                     options.architecture_independent,
                                     options.metapackage,
                                     options.bugtracker_url,
                                     options.replaces,
                                     options.conflicts)
        sys.stdout.write(pkg_xml)

def make_from_manifest(manifest_xml_str,
                       package_name,
                       version,
                       architecture_independent, metapackage,
                       bugtracker_url, replaces, conflicts):
    """
    Make the contents of a project.xml file from the string contents of
    manifest.xml.

    >>> manifest_xml_str = '\
    <package>\
      <description brief="one line of text">\
        long description goes here, \
        <em>XHTML is allowed</em>\
      </description>\
      <author>Alice/alice@somewhere.bar, Bob/bob@nowhere.foo</author>\
      <license>BSD</license>\
      <url>http://pr.willowgarage.com/</url>\
      <logo>http://pr.willowgarage.com/blog/photos/sensor_head1_500.jpg</logo>\
      <depend package="pkgname"/>\
      <depend package="common"/>\
      <rosdep name="python" />\
      <versioncontrol type="svn"\
          url="https://playerstage.svn.sourceforge.net/svnroot/playerstage/code/player/trunk"/>\
      <export>\
        <cpp cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lros"/>\
        <cpp os="osx" cflags="-I${prefix}/include" lflags="-L${prefix}/lib\
            -Wl,-rpath,-L${prefix}lib -lrosthread -framework CoreServices"/>\
      </export>\
    </package>\
    '
    >>> pkg_xml = make_from_manifest(  # doctest: +ELLIPSIS
    ...     manifest_xml_str,
    ...     package_name='my_pkg', version='0.1.2', 
    ...     architecture_independent=False,
    ...     metapackage=False,
    ...     bugtracker_url='https://github.com/ros/my_pkg/issues',
    ...     replaces=[], conflicts=[])
    >>> import xml.etree.ElementTree as ET
    >>> pkg = ET.XML(pkg_xml)
    """
    manifest = ET.XML(manifest_xml_str)
    description = manifest.find('description').text
    authors_str = manifest.find('author').text
    authors = parse_authors_field(authors_str)
    licenses_str = manifest.find('license').text
    licenses = SPACE_COMMA_RX.split(licenses_str)
    website_url = manifest.find('url').text
    maintainers = authors
    depend_tags = manifest.findall('depend')
    depends = [d.attrib['package'] for d in depend_tags]
    export_tags = manifest.find('export').getchildren()
    exports = [(e.tag, e.attrib) for e in export_tags]

    xml = create_project_xml(package_name=package_name,
                             version=version,
                             description=description,
                             maintainers=maintainers,
                             licenses=licenses,
                             website_url=website_url,
                             bugtracker_url=bugtracker_url,
                             authors=authors,
                             build_depends=depends,
                             run_depends=depends,
                             test_depends=depends,
                             replaces=replaces,
                             conflicts=conflicts,
                             exports=exports,
                             architecture_independent=architecture_independent,
                             metapackage=metapackage)

    for name in 'maintainer build_depend run_depend test_depend'.split():
        xml = comment_out_tags_named(xml, name)

    return xml

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

def parse_authors_field(authors_str):
    """
    Extract author names and email addresses from free-form text in the <author>
    tag of manifest.xml.

    >>> parse_authors_field('Alice/alice@somewhere.bar, Bob')
    [('Alice', {'email': 'alice@somewhere.bar'}), 'Bob']
    """
    authors = []
    for s in SPACE_COMMA_RX.split(authors_str):
        parts = s.split('/')
        if len(parts) == 1:
            authors.append(parts[0])
        elif len(parts) == 2:
            pair = (parts[0], dict(email=parts[1]))
            authors.append(pair)
    return authors

def create_project_xml(package_name, version, description, maintainers,
                       licenses, website_url, bugtracker_url, authors,
                       build_depends, run_depends, test_depends, replaces,
                       conflicts, exports, architecture_independent,
                       metapackage):
    """
    Generate the contents of project.xml from some parameters.

    :returns: XML-formatted string
    :rtype: str

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

def comment_out(xml):
    return '<!-- %s -->' % xml

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

if __name__ == '__main__':
    main()

