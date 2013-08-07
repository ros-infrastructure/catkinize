#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


from __future__ import print_function
import os
import logging
import re
import xml.etree.ElementTree as ET

from catkinize import xml_lib


##############################################################################
SPACE_COMMA_RX = re.compile(r',\s*')

# The final package is going to look like the PACKAGE_TEMPLATE
PACKAGE_TEMPLATE = '''\
<package>
  <name>%(package_name)s</name>
  <version>%(version)s</version>
  <description>%(description)s</description>
%(maintainers_part)s

%(licenses_part)s

  <url type="website">%(website_url)s</url>
%(bugtracker_part)s

%(authors_part)s

  <!-- Dependencies which this package needs to build itself. -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- Dependencies needed to compile this package. -->
%(build_depends_part)s

  <!-- Dependencies needed after this package is compiled. -->
%(run_depends_part)s

  <!-- Dependencies needed only for running tests. -->
%(test_depends_part)s

%(replaces_part)s
%(conflicts_part)s

%(exports_part)s

</package>
'''


##############################################################################
# Main Logic
##############################################################################
def convert_manifest(package_path,
                     manifest_xml_path,
                     version,
                     architecture_independent=False,
                     metapackage=False,
                     bugtracker_url='',
                     replaces=None,
                     conflicts=None):
    """
    Convert the given manifest.xml to a catkinized package.xml file (still
    a string representation).
    """
    if conflicts is None:
        conflicts = []
    if replaces is None:
        replaces = []

    package_name = os.path.basename(os.path.abspath(package_path))
    logging.basicConfig(format='%(levelname)s - %(message)s')
    try:
        with open(manifest_xml_path) as f:
            manifest_xml_str = f.read()
            pkg_xml = make_from_manifest(manifest_xml_str,
                                         package_name,
                                         version,
                                         architecture_independent,
                                         metapackage,
                                         bugtracker_url,
                                         replaces,
                                         conflicts)
            pkg_xml = '\n'.join(merge_adjacent_dups(pkg_xml.splitlines()))
            return pkg_xml

    except ET.ParseError as exc:
        line_num = int(re.compile(r'.*line (\d+).*').match(str(exc)).group(1))
        line = manifest_xml_str.splitlines()[line_num - 1]
        logging.error('%s\n"%s"\n', exc, line)


def make_from_manifest(manifest_xml_str,
                       package_name,
                       version,
                       architecture_independent, metapackage,
                       bugtracker_url, replaces, conflicts):
    """
    Return a package.xml sturcture filled with the data from the given
    manifest_xml_str.

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
    # collect and save infos from the manifest.xml file
    manifest = ET.XML(manifest_xml_str)
    description = xml_lib.xml_find(manifest, 'description').text.strip()
    authors_str = xml_lib.xml_find(manifest, 'author').text
    authors = parse_authors_field(authors_str)
    licenses_str = xml_lib.xml_find(manifest, 'license').text
    licenses = SPACE_COMMA_RX.split(licenses_str)
    website_url = xml_lib.xml_find(manifest, 'url').text
    maintainers = [(a, {'email': ''})
                   if isinstance(a, str)
                   else a for a in authors]
    depend_tags = manifest.findall('depend')
    depends = [d.attrib['package'] for d in depend_tags]
    export_tags = xml_lib.xml_find(manifest, 'export').getchildren()
    exports = [(e.tag, e.attrib) for e in export_tags]

    # put the collected infos into a new (package.)xml structure
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

    # Most dependencies are build and run depends. Comment out only the
    # test_depend dependencies.
    for name in ['test_depend']:
        xml = xml_lib.comment_out_tags_named(xml, name)

    return xml


def make_from_stack_manifest(manifest_xml_str,
                             package_name,
                             packages,
                             version):
    """
    Return a package.xml sturcture for metapackages filled with the data from
    the given manifest_xml_str.

    See http://ros.org/wiki/catkin/package.xml#Metapackages for more.

    """
    # TODO This function is very similar to make_from_manifest. Unify both
    #      functions.

    # collect and save infos from the manifest.xml file
    manifest = ET.XML(manifest_xml_str)
    description = xml_lib.xml_find(manifest, 'description').text.strip()
    authors_str = xml_lib.xml_find(manifest, 'author').text
    authors = parse_authors_field(authors_str)
    licenses_str = xml_lib.xml_find(manifest, 'license').text
    licenses = SPACE_COMMA_RX.split(licenses_str)
    website_url = xml_lib.xml_find(manifest, 'url').text
    maintainers = [(a, {'email': ''})
                   if isinstance(a, str)
                   else a for a in authors]

    # put the collected infos into a new (package.)xml structure
    xml = create_project_xml(package_name=package_name,
                             version=version,
                             description=description,
                             maintainers=maintainers,
                             licenses=licenses,
                             website_url=website_url,
                             bugtracker_url='',
                             authors=authors,
                             build_depends=[],
                             run_depends=packages,
                             test_depends=[],
                             replaces=[],
                             conflicts=[],
                             exports=[],
                             architecture_independent=False,
                             metapackage=True)

    return xml


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
    subs = {}
    subs['maintainers_part'] = make_section('maintainer', maintainers)
    subs['licenses_part'] = '\n'.join(
        indent('<license>%s</license>' % l)
        for l in licenses)

    bugtracker_part = '<url type="bugtracker">%s</url>' % bugtracker_url
    if not bugtracker_url:
        bugtracker_part = comment_out(bugtracker_part)
    subs['bugtracker_part'] = indent(bugtracker_part)

    subs['authors_part'] = make_section('author', authors)
    subs['build_depends_part'] = make_section('build_depend', build_depends)
    subs['run_depends_part'] = make_section('run_depend', run_depends)
    subs['test_depends_part'] = make_section('test_depend', test_depends)
    subs['replaces_part'] = make_section('replace', replaces)
    subs['conflicts_part'] = make_section('conflict', conflicts)
    subs['version'] = version
    subs['package_name'] = package_name
    subs['description'] = description
    subs['website_url'] = website_url
    subs['exports_part'] = make_exports_section(
        exports,
        architecture_independent,
        metapackage
    )
    return PACKAGE_TEMPLATE % subs


##############################################################################
# Utility functions
##############################################################################
def merge_adjacent_dups(lines):
    """
    Remove adjacent duplicate lines from a list of strings.

    >>> merge_adjacent_dups(['a', 'b', 'b'])
    ['a', 'b']
    >>> merge_adjacent_dups(['a', 'b', 'b', 'a'])
    ['a', 'b', 'a']
    >>> merge_adjacent_dups(['a'])
    ['a']
    >>> merge_adjacent_dups([])
    []
    """
    return [l1 for l1, l2 in zip(lines, lines[1:] + [None]) if l1 != l2]


def parse_authors_field(authors_str):
    """
    Extract author names and email addresses from free-form text in the
    <author> tag of manifest.xml.

    >>> parse_authors_field('Alice/alice@somewhere.bar, Bob')
    [('Alice', {'email': 'alice@somewhere.bar'}), 'Bob']
    >>> parse_authors_field(None)
    []
    """
    if authors_str is None:
        return []

    authors = []
    for s in SPACE_COMMA_RX.split(authors_str):
        parts = s.split('/')
        if len(parts) == 1:
            authors.append(parts[0])
        elif len(parts) == 2:
            pair = (parts[0], dict(email=parts[1]))
            authors.append(pair)
    return authors


def comment_out(xml):
    return '<!-- %s -->' % xml


def make_section(tag_name, rows):
    """
    Make a string in XML format for a section with a given tag name.
    """
    return '\n'.join(indent(make_tag_from_row(tag_name, r)) for r in rows)


def make_tag_from_row(name, row):
    """
    Make an XML tag from a row.

    >>> make_tag_from_row('foo', 'bar')
    '<foo>bar</foo>'
    >>> make_tag_from_row('foo', ('bar', dict(baz='buzz')))
    '<foo baz="buzz">bar</foo>'
    """
    if isinstance(row, str):
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


def indent(strg, amount=1):
    return (amount * '  ') + strg


def dict_to_attrs(values):
    """
    Convert a dictionary to a string containing attributes in XML format.
    """
    return ' '.join('%s="%s"' % (k, v) for k, v in values.items())


def make_exports_section(exports, architecture_independent, metapackage):
    if len(exports) > 0:
        parts=['<export>']
        parts += [make_empty_tag(name, attrs_dict)
                 for name, attrs_dict in exports]
        if architecture_independent:
            parts.append('<architecture_independent/>')
        if metapackage:
            parts.append('<metapackage/>')
        parts = [indent(p, 2) for p in parts if p!= '<export>']
        parts.append('</export>')
        
        return '\n'.join(parts)
    else:
        return ""
