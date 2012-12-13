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
import sys

from catkinize.convert_manifest import convert_manifest, make_from_stack_manifest
from catkinize.convert_cmake import convert_cmake


class Ui(object):

    def get_input(self, prompt):
        """Helper function to proived python2 + py3k compatible get_input"""
        if sys.hexversion > 0x03000000:
            return input(prompt)
        else:
            return raw_input(prompt)

DEFAULT_UI = Ui


def _create_changesets(path, filenames, newfiles=None, contents=None):
    """
    creates 4 tupels depending on the 4 input lists.
    look up filenames in path, add changeset to rename to xyz.backup, if newfile is given, adds action to create with given contents
    """
    oldfiles = [os.path.join(path, filename) for filename in filenames]
    backup_files = [oldfile + '.backup' for oldfile in oldfiles]
    changeset = []  # 4-tupels of oldfile, backup, newfile, contents
    for oldfile, backup_file in zip(oldfiles, backup_files):
        if os.path.isfile(oldfile) and os.path.isfile(backup_file):
            raise ValueError('Cannot write backup file %s, operation aborted without changes' % backup_file)

    if not newfiles:
        newfiles = [None for _ in oldfiles]
        contents = [None for _ in oldfiles]

    for oldfile, backup_file, newfile, content in zip(oldfiles, backup_files, newfiles, contents):
        if os.path.exists(oldfile):
            if newfile:
                changeset.append((oldfile,
                                  backup_file,
                                  os.path.join(path, newfile),
                                  content))
            else:
                changeset.append((oldfile,
                                  backup_file,
                                  None, None))
        elif newfile:
            changeset.append((None, None, newfile, content))

    return changeset


def catkinize_package(path, version):
    """
    Calculates a list of changes for one package. changes are 4-tupels of oldfile, backupfile, newfile, contents.
    This comes before execution so that the user may confirm or reject changes.
    """
    if not os.path.isdir(path):
        raise ValueError('No directory found at %s' % path)
    manifest_path = os.path.join(path, 'manifest.xml')

    if not os.path.isfile(manifest_path):
        raise ValueError("No rosbuild package at %s, missing manifest.xml" % manifest_path)
    new_manifest = convert_manifest(path, manifest_path, version)
    new_cmake = convert_cmake(path)

    filenames = ['CMakeLists.txt', 'manifest.xml', 'Makefile']
    newfiles = ['CMakeLists.txt', 'package.xml', None]
    contents = [new_cmake, new_manifest, None]

    return _create_changesets(path, filenames, newfiles, contents)


def catkinize_stack(path, version):
    """
    Calculates a list of changes for one stack. changes are 4-tupels of oldfile, backupfile, newfile, contents.
    This comes before execution so that the user may confirm or reject changes.
    """
    stack_manifest_path = os.path.join(path, 'stack.xml')
    if not os.path.isfile(stack_manifest_path):
        raise ValueError('Path is not a rosbuild stack, missing stack.xml at %s' % path)
    with open(stack_manifest_path) as fhand:
        stack_manifest = fhand.read()
    changeset = []

    if os.path.isfile(os.path.join(path, 'manifest.xml')):
        # unary stack
        packages = [path]
        changeset.extend(_create_changesets(path, ['stack.xml', 'Makefile', 'CMakeLists.txt']))
    else:
        packages = []
        for (parentdir, subdirs, files) in os.walk(path):
            # print(files)
            if 'manifest.xml' in files:
                packages.append(parentdir)
                del subdirs[:]
            elif os.path.basename(parentdir) in ['.svn', 'CVS', '.hg', '.git']:
                del subdirs[:]
        meta_package_name = os.path.basename(path)
        meta_manifest = os.path.join(meta_package_name, 'package.xml')
        package_names = [os.path.basename(package) for package in packages]
        meta_contents = make_from_stack_manifest(stack_manifest, meta_package_name, package_names, version)
        changeset.extend(_create_changesets(path,
                                            ['stack.xml', 'Makefile', 'CMakeLists.txt'],
                                            [meta_manifest, None, None],
                                            [meta_contents, None, None]))

    # print(packages)

    for package in packages:
        changeset.extend(catkinize_package(package, version))
    return changeset


def prompt_changes(changeset, ui_class=DEFAULT_UI):
    """
    interactive function, displays a list of planned changes for the user to confirm.
    :returns: True if the user confirmed, false else
    """
    ui = ui_class()
    abort = False
    details = False
    prompt = "Perform these changes ((y)es / (n)o / (d)etails):"
    while not abort:
        for oldfile, backup_file, newfile, content in changeset:
            if oldfile:
                print('Backup file %s  ==>  %s' % (oldfile, backup_file))
            if newfile:
                print('Create converted file %s' % newfile)
                if details:
                    print('-' * 80)
                    print(content)
                    print('-' * 80)

        user_input = ui.get_input(prompt)
        if user_input == 'y':
            return True
        elif user_input == 'd':
            details = not details
        elif user_input == 'n':
            abort = True
    print("User aborted")
    return False


def perform_changes(changeset):
    """
    Performs a set of changes as calculated by the other methods in this module
    """
    for oldfile, backup_file, newfile, content in changeset:
        if oldfile:
            os.rename(oldfile, backup_file)
            print('Backed up file: %s  ==>  %s ' % (oldfile, backup_file))
        if newfile:
            # for new meta packages
            if not os.path.isdir(os.path.dirname(newfile)):
                os.mkdir(os.path.dirname(newfile))
            with open(newfile, "w") as fhand:
                fhand.write(content)
                print("Wrote new file %s" % newfile)
