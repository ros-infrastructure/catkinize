#!/usr/bin/env python

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


'''Script to generate package.xml from manifest.xml'''
import os
from optparse import OptionParser

from catkinize.convert_manifest import convert_manifest
from catkinize.utils import is_valid_version


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
        parser.error('wrong number of arguments %s' % len(args))

    manifest_xml_path = args[0]
    version = args[2]

    if not is_valid_version(version):
        parser.error("The version must have the format: \d.\d.\d")

    print(convert_manifest(os.path.dirname(manifest_xml_path),
                           manifest_xml_path,
                           version,
                           options.architecture_independent,
                           options.metapackage,
                           options.bugtracker_url,
                           options.replaces,
                           options.conflicts))


if __name__ == '__main__':
    main()
