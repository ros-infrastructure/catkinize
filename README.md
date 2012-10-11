Catkinize
=========

Catkinize converts ROS stacks to catkin projects.  In cases where it is not
clear how to do this, Catkinize makes a note of what needs to be done.  After
doing the conversion, Catkinize outputs a list of issues encountered so the user
can decide how to proceed with manual changes.

Simple usage
------------

    cd /directory/of/a/rosbuild/stack/
    catkinize

Background
----------

[This wiki page](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/rosbuild_migration.html)
gives some instruction on how to catkinize.
More general information on writing CMakeLists.txt files using Catkin can
be found
[here](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html)

