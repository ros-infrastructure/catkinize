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

'''
This script partially converts a CMakeLists.txt file from rosbuild to catkin.
'''

from __future__ import print_function
import os
import sys
import argparse
from catkinize.convert_cmake import convert_cmake


def main(argv, outstream):
    """
    reads file and prints converted file to stdout
    """
    parser = argparse.ArgumentParser(description='writes converted version of CMakeLists.txt to stdout')
    parser.add_argument('project_path',
                        nargs='?',
                        default=None,
                        help='The path to the package')
    parser.add_argument('cmakelists_path',
                        nargs='?',
                        default=None,
                        help='path to the CMakeLists.txt')
    parser.add_argument('manifest_xml_path',
                        nargs='?',
                        default=None,
                        help='path to the manifest.xml')
    # Parse args
    args = parser.parse_args(argv)
    project_path = args.project_path or os.getcwd()

    # Convert CMakeLists.txt
    print(
        convert_cmake(
            project_path,
            args.cmakelists_path,
            args.manifest_xml_path),
        file=outstream)


if __name__ == '__main__':
    main(argv=sys.argv[1:], outstream=sys.stdout)
