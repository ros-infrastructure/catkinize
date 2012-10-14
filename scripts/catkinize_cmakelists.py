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
import re
import sys

conversions = [
    ('rosbuild_add_library', 'add_library'),
    ('rosbuild_add_executable', 'add_executable'),
    ('rosbuild_add_gtest', 'add_gtest'),
    ('rosbuild_add_pyunit', 'catkin_add_nosetests'),
    ('rosbuild_add_rostest', 'add_rostest')
]

def main():
    # Parse args
    args = sys.argv[1:]
    if len(args) != 2:
        print('usage:', sys.argv[0], 'project_name CMakeLists.txt')
        sys.exit(1)
    project_name, cmakelists_path = args

    # Convert CMakeLists.txt
    print('Converting %s' % cmakelists_path, file=sys.stderr)
    with open(cmakelists_path, 'r') as f_in:
        lines = f_in.read().splitlines()
    for line in convert_cmakelists(project_name, lines):
        print(line)

def convert_cmakelists(project_name, lines):
    """
    Catkinize the lines of a file as much as we can without manual intervention.
    """
    lines = map(convert_line, lines)
    lines = list(convert_boost(lines))
    lines = add_header_if_needed(lines, make_header_lines(lines, project_name))
    return lines

def add_header_if_needed(lines, header):
    if not [l for l in lines if 'catkin_project' in l]:
        return header + [''] + lines
    return lines

def make_header_lines(lines, project_name):
    """
    Make top lines of CMakeLists file according to
    http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
    """
    header = '''
cmake_minimum_required(VERSION 2.8.3)
project(%s)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_project(${PROJECT_NAME})
''' % project_name
    return header.strip().splitlines()

def convert_line(line):
    """
    Do all replacements that can be done for a single line without looking at
    anything else.
    """
    for a, b in conversions:
        line = line.replace(a, b)
    return line

def convert_boost(lines):
    """
    convert_cmakelists Boost sections.
    """
    link_boost_rx = re.compile(r'rosbuild_link_boost\(([^ ]+)\s+([^)]+)\)')
    for l in lines:
        if 'rosbuild_add_boost_directories' in l:
            # These lines are no longer needed.
            continue
        elif 'rosbuild_link_boost' in l:
            # rosbuild_link_boost lines expand to multiple statements.
            m = link_boost_rx.match(l)
            target = m.group(1)
            components = m.group(2)
            yield 'find_package(Boost REQUIRED COMPONENTS %s)' % components
            yield 'include_directories(${Boost_INCLUDE_DIRS})'
            yield 'target_link_libraries(%s ${Boost_LIBRARIES})' % target
        else:
            # All other lines pass through unchanged.
            yield l 

if __name__ == '__main__':
    main()

